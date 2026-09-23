from datetime import datetime, timezone
import unittest

from tunnel_data_collection.recording_utils import (
    FrameRateGate,
    VIDEO_STREAMS,
    ffmpeg_mp4_command,
    new_session_path,
    validate_recording_settings,
)


class RecordingUtilsTest(unittest.TestCase):
    def test_three_mp4_streams_have_distinct_filenames(self):
        self.assertEqual(
            set(VIDEO_STREAMS),
            {
                "/camera/image_rect",
                "/camera/image_stereo_ir",
                "/camera/image_bev_ir",
            },
        )
        self.assertEqual(len(set(VIDEO_STREAMS.values())), 3)
        self.assertTrue(all(name.endswith(".mp4") for name in VIDEO_STREAMS.values()))

    def test_new_session_path_is_timestamped(self):
        now = datetime(2026, 9, 22, 12, 34, 56, 123456, tzinfo=timezone.utc)
        path = new_session_path("/tmp/recordings", now)
        self.assertEqual(path.parent.name, "recordings")
        self.assertEqual(path.name, "tunnel_20260922_123456_123456+0000")

    def test_invalid_recording_settings_are_rejected(self):
        with self.assertRaises(ValueError):
            validate_recording_settings(0, 18, "ultrafast")
        with self.assertRaises(ValueError):
            validate_recording_settings(30, 52, "ultrafast")
        with self.assertRaises(ValueError):
            validate_recording_settings(30, 18, "slow")

    def test_ffmpeg_command_has_explicit_input_pixel_format(self):
        command = ffmpeg_mp4_command(
            "/usr/bin/ffmpeg", "/tmp/rgb.mp4", 1280, 800, 30, 18, "ultrafast"
        )
        self.assertEqual(command[command.index("-pixel_format") + 1], "bgr24")
        self.assertEqual(command[command.index("-c:v") + 1], "libx264")
        self.assertEqual(command[command.index("-pix_fmt") + 1], "yuv420p")
        self.assertEqual(command[-1], "/tmp/rgb.mp4")

    def test_frame_rate_gate_limits_timestamps_without_drift(self):
        gate = FrameRateGate(10.0)
        self.assertTrue(gate.accept(0))
        self.assertFalse(gate.accept(90_000_000))
        self.assertTrue(gate.accept(100_000_000))
        self.assertTrue(gate.accept(450_000_000))
        self.assertFalse(gate.accept(500_000_000))
        self.assertTrue(gate.accept(550_000_000))


if __name__ == "__main__":
    unittest.main()
