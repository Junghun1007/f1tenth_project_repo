"""Read a LaneResult capture stamp while preserving the serialized output."""
import struct


class RawLane:
    """Routing needs only the capture stamp; forwarding reuses the same bytes."""

    def __init__(self, data, decode):
        self.raw = bytes(data)
        # Classic CDR: 4-byte encapsulation, then std_msgs/Header.stamp.
        if len(self.raw) >= 12 and self.raw[0] == 0 and self.raw[1] in (0, 1):
            sec, nanosec = struct.unpack_from('<iI' if self.raw[1] else '>iI', self.raw, 4)
        else:
            # Unknown encapsulation: never guess offsets, decode normally.
            stamp = decode(self.raw).header.stamp
            sec, nanosec = stamp.sec, stamp.nanosec
        self.capture_ns = sec * 1_000_000_000 + nanosec
