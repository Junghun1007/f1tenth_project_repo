"""ROS-independent image decoding and timestamp pairing helpers."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class FrameRecord:
    index: int
    header_ns: int
    bag_ns: int
    filename: str
    width: int
    height: int
    encoding: str


def decode_image(*, data, width, height, step, encoding):
    """Decode supported sensor_msgs/Image storage into a tightly packed BGR image."""
    if width <= 0 or height <= 0 or step <= 0:
        raise ValueError("image dimensions and step must be positive")
    raw = np.frombuffer(data, dtype=np.uint8)
    normalized = encoding.lower()
    if normalized == "nv12":
        if width % 2 or height % 2 or step < width:
            raise ValueError("NV12 requires even dimensions and step >= width")
        required = step * (height + height // 2)
        if raw.size < required:
            raise ValueError("undersized NV12 message")
        planes = raw[:required].reshape(height + height // 2, step)[:, :width]
        return cv2.cvtColor(np.ascontiguousarray(planes), cv2.COLOR_YUV2BGR_NV12)
    if normalized in ("bgr8", "rgb8"):
        row_bytes = width * 3
        if step < row_bytes or raw.size < step * height:
            raise ValueError("undersized 8-bit color message")
        packed = raw[: step * height].reshape(height, step)[:, :row_bytes]
        image = np.ascontiguousarray(packed.reshape(height, width, 3))
        if normalized == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image
    if normalized in ("mono8", "8uc1"):
        if step < width or raw.size < step * height:
            raise ValueError("undersized mono8 message")
        gray = np.ascontiguousarray(raw[: step * height].reshape(height, step)[:, :width])
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported image encoding: {encoding}")


def pair_frames(rgb_frames, bev_frames, maximum_delta_ns):
    """Return monotonic, one-to-one nearest timestamp pairs within the tolerance."""
    if maximum_delta_ns < 0:
        raise ValueError("maximum_delta_ns must be nonnegative")
    rgb = sorted(rgb_frames, key=lambda frame: frame.header_ns)
    bev = sorted(bev_frames, key=lambda frame: frame.header_ns)
    candidates = []
    left = 0
    for rgb_index, rgb_frame in enumerate(rgb):
        while left < len(bev) and bev[left].header_ns < rgb_frame.header_ns - maximum_delta_ns:
            left += 1
        bev_index = left
        while bev_index < len(bev) and bev[bev_index].header_ns <= rgb_frame.header_ns + maximum_delta_ns:
            candidates.append(
                (
                    abs(rgb_frame.header_ns - bev[bev_index].header_ns),
                    rgb_index,
                    bev_index,
                )
            )
            bev_index += 1
    used_rgb = set()
    used_bev = set()
    pairs = []
    for delta, rgb_index, bev_index in sorted(candidates):
        if rgb_index in used_rgb or bev_index in used_bev:
            continue
        used_rgb.add(rgb_index)
        used_bev.add(bev_index)
        pairs.append((rgb[rgb_index], bev[bev_index], delta))
    return sorted(pairs, key=lambda pair: pair[0].header_ns)
