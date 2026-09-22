from datetime import datetime, timezone
import unittest

from tunnel_data_collection.recording_utils import (
    RECORDED_TOPICS,
    new_session_path,
    rosbag_command,
)


class RecordingUtilsTest(unittest.TestCase):
    def test_rosbag_command_records_exactly_two_approved_topics(self):
        command = rosbag_command("/tmp/run", 1024)
        self.assertEqual(command[-2:], list(RECORDED_TOPICS))
        self.assertNotIn("/camera/image_bev", command)
        self.assertEqual(command[command.index("--max-bag-size") + 1], "1024")

    def test_new_session_path_is_timestamped(self):
        now = datetime(2026, 9, 22, 12, 34, 56, 123456, tzinfo=timezone.utc)
        path = new_session_path("/tmp/recordings", now)
        self.assertEqual(path.parent.name, "recordings")
        self.assertEqual(path.name, "tunnel_20260922_123456_123456+0000")

    def test_invalid_bag_size_is_rejected(self):
        with self.assertRaises(ValueError):
            rosbag_command("/tmp/run", 0)


if __name__ == "__main__":
    unittest.main()
