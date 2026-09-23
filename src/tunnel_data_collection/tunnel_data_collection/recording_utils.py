"""ROS-independent helpers for three-stream H.264 MP4 recording."""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


VIDEO_STREAMS = {
    "/camera/image_rect": "rgb.mp4",
    "/camera/image_stereo_ir": "stereo_ir.mp4",
    "/camera/image_bev_ir": "bev_ir.mp4",
}

H264_PRESETS = (
    "ultrafast",
    "superfast",
    "veryfast",
    "faster",
    "fast",
    "medium",
)


def new_session_path(output_root, now=None):
    """Return a unique, timestamped recording directory."""
    timestamp = (now or datetime.now().astimezone()).strftime(
        "tunnel_%Y%m%d_%H%M%S_%f%z"
    )
    return Path(output_root).expanduser() / timestamp


def validate_recording_settings(fps, crf, preset):
    value = float(fps)
    crf_value = int(crf)
    normalized_preset = str(preset).strip().lower()
    if not 0.1 <= value <= 120.0:
        raise ValueError("recording_fps must be in [0.1, 120.0]")
    if not 0 <= crf_value <= 51:
        raise ValueError("h264_crf must be in [0, 51]")
    if normalized_preset not in H264_PRESETS:
        supported = ", ".join(H264_PRESETS)
        raise ValueError(f"h264_preset must be one of: {supported}")
    return value, crf_value, normalized_preset


def ffmpeg_mp4_command(
    ffmpeg_binary, output_path, width, height, fps, crf, preset
):
    """Build a deterministic raw-BGR-to-H.264-MP4 FFmpeg command."""
    width = int(width)
    height = int(height)
    fps, crf, preset = validate_recording_settings(fps, crf, preset)
    if width <= 0 or height <= 0 or width % 2 or height % 2:
        raise ValueError("MP4 width and height must be positive even integers")
    return [
        str(ffmpeg_binary),
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pixel_format",
        "bgr24",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        f"{fps:.6f}",
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        preset,
        "-crf",
        str(crf),
        "-pix_fmt",
        "yuv420p",
        str(output_path),
    ]


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
