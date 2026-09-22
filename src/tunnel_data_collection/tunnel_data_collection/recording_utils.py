"""ROS-independent helpers for tunnel rosbag session recording."""

from datetime import datetime
from pathlib import Path


RECORDED_TOPICS = ("/camera/image_rect", "/camera/image_bev_ir")


def new_session_path(output_root, now=None):
    """Return a unique, timestamped bag path without creating it."""
    timestamp = (now or datetime.now().astimezone()).strftime(
        "tunnel_%Y%m%d_%H%M%S_%f%z"
    )
    return Path(output_root).expanduser() / timestamp


def rosbag_command(output_path, max_bag_size):
    """Build the command that records exactly the two approved image topics."""
    size = int(max_bag_size)
    if size <= 0:
        raise ValueError("max_bag_size must be positive")
    return [
        "ros2",
        "bag",
        "record",
        "--storage",
        "sqlite3",
        "--output",
        str(output_path),
        "--max-bag-size",
        str(size),
        *RECORDED_TOPICS,
    ]
