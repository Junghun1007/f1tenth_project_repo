"""31 training-compatible features and timestamp-based causal context.

At regular 30/10 FPS this matches the frozen training feature extractor.
For irregular delivery, rates use source elapsed time, smoothing uses the past
0.5 s and context selects the latest available sample at/before each target.
Missing source frames are counted by the node, not invented by interpolation.
"""
from bisect import bisect_right
from collections import deque
from functools import lru_cache
import cv2
import numpy as np

LEVELS = np.arange(256, dtype=np.float32) / 255.


@lru_cache(maxsize=2)
def radial_masks(height=400, width=640):
    yy, xx = np.mgrid[:height, :width]
    radius = np.sqrt(((xx - (width - 1) / 2) / (width / 2)) ** 2 +
                     ((yy - (height - 1) / 2) / (height / 2)) ** 2)
    edges = np.linspace(0, np.sqrt(2) + 1e-6, 9)
    return tuple((radius >= edges[j]) & (radius < edges[j + 1]) for j in range(8))


def histograms(gray):
    if gray.shape != (400, 640) or gray.dtype != np.uint8:
        raise ValueError('Expected grayscale uint8[400,640]')
    h, w = gray.shape
    ys, xs = [0, h // 3, 2 * h // 3, h], [0, w // 3, 2 * w // 3, w]
    tiles = [np.bincount(gray[ys[r]:ys[r + 1], xs[c]:xs[c + 1]].ravel(), minlength=256)
             for r in range(3) for c in range(3)]
    radial = [np.bincount(gray[m], minlength=256) for m in radial_masks()]
    return np.asarray(tiles + radial, dtype=np.uint32)


def static_features(hist):
    h = np.asarray(hist[:9], dtype=np.float32)
    counts = h.sum(-1)
    means = (h * LEVELS).sum(-1) / counts
    whole = h.sum(0)
    total = whole.sum()
    mean = (whole * LEVELS).sum() / total
    std = np.sqrt(np.maximum((whole * (LEVELS - mean) ** 2).sum() / total, 0))
    cdf = np.cumsum(whole)
    quantiles = [LEVELS[np.argmax(cdf >= total * q)] for q in [.1, .5, .9]]
    surround = (means[[1, 3, 5]] * counts[[1, 3, 5]]).sum() / counts[[1, 3, 5]].sum()
    contrast = (surround - means[4]) / (surround + means[4] + 1e-6 / 255.)
    fractions = [(whole * m).sum() / total for m in
                 [LEVELS < 32 / 255., LEVELS < 64 / 255., LEVELS > 192 / 255., LEVELS > 224 / 255.]]
    radial = np.asarray(hist[9:], dtype=np.float32)
    radial_means = (radial * LEVELS).sum(-1) / radial.sum(-1)
    return np.asarray([mean, std, *quantiles, *means, surround, contrast,
                       *fractions, *radial_means], dtype=np.float32)


def preprocess(image):
    if image.dtype != np.uint8 or image.ndim != 2:
        raise ValueError('Camera input must be mono8')
    return cv2.resize(image, (640, 400), interpolation=cv2.INTER_AREA)


class FeatureStream:
    def __init__(self, warmup_seconds=2.5, max_gap_seconds=.5):
        self.warmup_seconds = float(warmup_seconds)
        self.max_gap_seconds = float(max_gap_seconds)
        self.reset()

    def reset(self):
        self.samples = deque()
        self.means = deque()
        self.started = None
        self.last_time = None
        self.last_mean = None
        self.last_smooth = None
        self.ready = False
        self.discontinuity = False

    def push(self, gray, timestamp):
        return self.push_static(static_features(histograms(preprocess(gray))), timestamp)

    def push_static(self, static, timestamp):
        timestamp = float(timestamp)
        if not np.isfinite(timestamp) or np.shape(static) != (28,):
            raise ValueError('Invalid feature sample')
        gap = self.last_time is not None and (timestamp <= self.last_time or
                                              timestamp - self.last_time > self.max_gap_seconds)
        if gap:
            self.reset()
        self.discontinuity = gap
        if self.started is None:
            self.started = timestamp
        self.means.append((timestamp, np.float32(static[0])))
        while self.means and self.means[0][0] <= timestamp - .5 + 1e-7:
            self.means.popleft()
        smooth = np.mean([v for _, v in self.means], dtype=np.float64)
        dt = None if self.last_time is None else timestamp - self.last_time
        rate = 0. if dt is None else float(np.float32(static[0] - self.last_mean)) / dt
        smooth_rate = 0. if dt is None else (smooth - self.last_smooth) / dt
        feature = np.concatenate((static, np.asarray([smooth, rate, smooth_rate], dtype=np.float32)))
        if not np.isfinite(feature).all():
            raise ValueError('Non-finite features')
        self.samples.append((timestamp, feature))
        # Keep one older sample for zero-order hold at the earliest target.
        while len(self.samples) > 1 and self.samples[1][0] < timestamp - 2.1:
            self.samples.popleft()
        times = [t for t, _ in self.samples]
        indices = [max(0, bisect_right(times, timestamp - offset / 10. + 1e-7) - 1)
                   for offset in range(20, -1, -1)]
        context = np.stack([self.samples[i][1] for i in indices])[None].astype(np.float32)
        self.last_time, self.last_mean, self.last_smooth = timestamp, static[0], smooth
        self.ready = timestamp - self.started >= self.warmup_seconds - 1e-7
        return context


class EventDecoder:
    def __init__(self, threshold=.8, cooldown=.1):
        self.threshold, self.cooldown = threshold, cooldown
        self.reset()

    def reset(self):
        self.state = 'unknown'
        self.active = 0
        self.last = {1: -np.inf, 2: -np.inf}

    def pause(self):
        """Forget signal edge/cooldown on a stream break; preserve tunnel state."""
        self.active = 0
        self.last = {1: -np.inf, 2: -np.inf}

    def step(self, probabilities, timestamp):
        p = np.asarray(probabilities)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError('Expected finite probabilities [none, entry, exit]')
        c = int(p.argmax())
        if c == 0 or p[c] < self.threshold:
            self.active = 0
            return None
        fresh = c != self.active
        self.active = c
        if not fresh or timestamp - self.last[c] < self.cooldown - 1e-7:
            return None
        if (self.state == 'inside' and c == 1) or (self.state == 'outside' and c == 2):
            return None
        self.state = 'inside' if c == 1 else 'outside'
        self.last[c] = timestamp
        return {'type': 'entry' if c == 1 else 'exit', 'score': float(p[c]), 'time_s': timestamp}


def softmax(logits):
    z = np.asarray(logits, dtype=np.float32).reshape(3)
    e = np.exp(z - z.max())
    return e / e.sum()
