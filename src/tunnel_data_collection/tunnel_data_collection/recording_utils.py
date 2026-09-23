"""ROS-independent helpers for three-stream AVI recording."""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


VIDEO_STREAMS = {
    "/camera/image_rect": "rgb.avi",
    "/camera/image_stereo_ir": "stereo_ir.avi",
    "/camera/image_bev_ir": "bev_ir.avi",
}


def new_session_path(output_root, now=None):
    """Return a unique, timestamped recording directory."""
    timestamp = (now or datetime.now().astimezone()).strftime(
        "tunnel_%Y%m%d_%H%M%S_%f%z"
    )
    return Path(output_root).expanduser() / timestamp


def validate_recording_settings(fps, codec):
    value = float(fps)
    normalized_codec = str(codec).strip().upper()
    if not 0.1 <= value <= 120.0:
        raise ValueError("recording_fps must be in [0.1, 120.0]")
    if len(normalized_codec) != 4 or not normalized_codec.isascii():
        raise ValueError("avi_codec must contain exactly four ASCII characters")
    return value, normalized_codec


@dataclass
class FrameRateGate:
    """Timestamp-based maximum-rate gate that does not accumulate drift."""

    fps: float
    next_stamp_ns: int | None = None

    def __post_init__(self):
        self.period_ns = max(1, round(1_000_000_000 / float(self.fps)))
        # Allow normal camera timestamp jitter around an exact frame boundary.
        self.tolerance_ns = min(2_000_000, self.period_ns // 20)

    def accept(self, stamp_ns):
        stamp = int(stamp_ns)
        if (
            self.next_stamp_ns is None
            or stamp + self.tolerance_ns >= self.next_stamp_ns
        ):
            self.next_stamp_ns = stamp + self.period_ns
            return True
        return False
