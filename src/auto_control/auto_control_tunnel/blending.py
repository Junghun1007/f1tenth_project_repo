"""Metric lane validation and short RGB/IR handover blending, without ROS."""
from bisect import bisect_left
from copy import deepcopy
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PathRules:
    frame_id: str = 'front_axle_bev'
    minimum_points: int = 8
    minimum_span: float = .12
    minimum_x: float = .05
    maximum_x: float = 2.20
    maximum_gap: float = .15
    x_max: float = 3.
    y_max: float = .6
    mpp: float = .01

    def __post_init__(self):
        if (not self.frame_id or self.minimum_points < 3 or
                not all(math.isfinite(x) and x > 0 for x in (
                    self.minimum_span, self.maximum_x, self.maximum_gap, self.x_max, self.y_max, self.mpp)) or
                not math.isfinite(self.minimum_x) or not 0 <= self.minimum_x < self.maximum_x <= self.x_max):
            raise ValueError('Invalid tunnel path geometry rules')


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def metric_path(message, rules):
    """Use the controller's canvas, support and contiguous-path requirements."""
    try:
        width, height = message.source_width, message.source_height
        sx = message.centerline_bev_width_m / width
        sy = message.centerline_bev_height_m / height
        close = lambda a, b: math.isclose(a, b, rel_tol=1e-4, abs_tol=0.)
        if (not message.centerline_valid or message.centerline_sample_limit_reached or
                message.state not in (1, 2, 3) or message.header.frame_id != rules.frame_id or
                width <= 0 or height <= 0 or
                not all(math.isfinite(v) and v > 0 for v in (sx, sy)) or
                not close(sx, rules.mpp) or not close(sy, rules.mpp) or
                not close(width * sx, 2 * rules.y_max) or not close(height * sy, rules.x_max) or
                not 2 <= len(message.centerline_points) <= 10000 or
                len(message.centerline_points) != len(message.centerline_support)):
            return None
        raw = []
        for p, support in zip(message.centerline_points, message.centerline_support):
            if (support not in (1, 2, 3, 4) or
                    not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or abs(p.z) > 1e-6 or
                    not 0 <= p.x < width + message.padding_left + message.padding_right or
                    not 0 <= p.y < height):
                return None
            raw.append((rules.x_max - (p.y + .5) * sy,
                        rules.y_max - (p.x - message.padding_left + .5) * sx, support))
        points, span = [], 0.
        for p in raw:
            if not rules.minimum_x <= p[0] <= rules.maximum_x:
                if points:
                    break
                continue
            if points:
                gap = math.hypot(p[0] - points[-1][0], p[1] - points[-1][1])
                if gap < 1e-5:
                    continue
                if gap > rules.maximum_gap:
                    break
                span += gap
            points.append(p)
        return points if len(points) >= rules.minimum_points and span >= rules.minimum_span else None
    except (AttributeError, TypeError, ValueError, ZeroDivisionError, OverflowError):
        return None


def mix_paths(old, new, old_path, new_path, weight, rules, max_offset, max_heading):
    """Resample at common forward distances; never extrapolate either curve.

    Timestamps remain conservative: the output carries the older contributor's
    stamp, so caching cannot make an old observation look freshly captured.
    """
    if any(b[0] - a[0] <= 1e-6 for path in (old_path, new_path)
           for a, b in zip(path, path[1:])):
        return None, 'nonmonotonic_forward_path'
    lower, upper = max(old_path[0][0], new_path[0][0]), min(old_path[-1][0], new_path[-1][0])
    if upper - lower < rules.minimum_span:
        return None, 'insufficient_overlap'
    count = max(rules.minimum_points, math.ceil((upper - lower) / rules.mpp) + 1)
    xs = [lower + (upper - lower) * i / (count - 1) for i in range(count)]

    def sample(path):
        abscissae = [p[0] for p in path]
        values = []
        for x in xs:
            i = min(len(path) - 1, max(1, bisect_left(abscissae, x)))
            a, b = path[i-1], path[i]
            t = (x - a[0]) / (b[0] - a[0])
            values.append((a[1] + t * (b[1] - a[1]), a[2] if t < .5 else b[2]))
        return values

    a, b = sample(old_path), sample(new_path)
    if max(abs(p[0] - q[0]) for p, q in zip(a, b)) > max_offset:
        return None, 'lateral_disagreement'
    window = max(1, round(.15 / ((upper - lower) / (count - 1))))
    for i in range(count - 1):
        j = min(count - 1, i + window)
        ha = math.atan2(a[j][0] - a[i][0], xs[j] - xs[i])
        hb = math.atan2(b[j][0] - b[i][0], xs[j] - xs[i])
        if abs(math.atan2(math.sin(ha - hb), math.cos(ha - hb))) > max_heading:
            return None, 'heading_disagreement'
    result = deepcopy(new)
    result.header.stamp = deepcopy(min((old, new), key=stamp_ns).header.stamp)
    result.centerline_points = []
    result.centerline_support = []
    prototype = new.centerline_points[0]
    for x, pa, pb in zip(xs, a, b):
        lateral = (1 - weight) * pa[0] + weight * pb[0]
        p = deepcopy(prototype)
        p.x = (rules.y_max - lateral) / rules.mpp + result.padding_left - .5
        p.y = (rules.x_max - x) / rules.mpp - .5
        p.z = 0.
        result.centerline_points.append(p)
        result.centerline_support.append(pa[1] if weight < .5 else pb[1])
    # A mixed centerline does not provide an independently observed stop line.
    result.stop_line_present = False
    result.stop_line_distance_m = float('nan')
    if metric_path(result, rules) is None:
        return None, 'mixed_geometry_invalid'
    return result, 'blending'
