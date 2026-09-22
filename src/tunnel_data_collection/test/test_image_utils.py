import unittest

import cv2
import numpy as np

from tunnel_data_collection.image_utils import FrameRecord, decode_image, pair_frames


class ImageUtilsTest(unittest.TestCase):
    def test_decode_nv12_with_padding(self):
        bgr = np.full((4, 6, 3), (20, 100, 220), np.uint8)
        i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).reshape(-1)
        y_size = 4 * 6
        quarter = y_size // 4
        y = i420[:y_size].reshape(4, 6)
        u = i420[y_size : y_size + quarter].reshape(2, 3)
        v = i420[y_size + quarter :].reshape(2, 3)
        uv = np.empty((2, 6), np.uint8)
        uv[:, 0::2] = u
        uv[:, 1::2] = v
        padded = np.zeros((6, 8), np.uint8)
        padded[:4, :6] = y
        padded[4:, :6] = uv
        decoded = decode_image(
            data=padded.tobytes(), width=6, height=4, step=8, encoding="nv12"
        )
        self.assertEqual(decoded.shape, (4, 6, 3))
        self.assertLess(np.abs(decoded.astype(int) - bgr.astype(int)).mean(), 4.0)

    def test_decode_bgr8_with_padding(self):
        expected = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        padded = np.zeros((2, 12), np.uint8)
        padded[:, :9] = expected.reshape(2, 9)
        actual = decode_image(
            data=padded.tobytes(), width=3, height=2, step=12, encoding="bgr8"
        )
        np.testing.assert_array_equal(actual, expected)

    def test_rejects_unsupported_or_short_images(self):
        with self.assertRaises(ValueError):
            decode_image(data=b"", width=2, height=2, step=2, encoding="nv12")
        with self.assertRaises(ValueError):
            decode_image(data=bytes(4), width=2, height=2, step=2, encoding="16UC1")

    def test_pairing_is_nearest_and_one_to_one(self):
        def frame(index, stamp, prefix):
            return FrameRecord(index, stamp, stamp, f"{prefix}{index}.png", 1, 1, "bgr8")

        rgb = [frame(0, 100, "r"), frame(1, 200, "r"), frame(2, 300, "r")]
        bev = [frame(0, 105, "b"), frame(1, 190, "b"), frame(2, 500, "b")]
        pairs = pair_frames(rgb, bev, 20)
        self.assertEqual([(a.index, b.index, d) for a, b, d in pairs], [(0, 0, 5), (1, 1, 10)])

    def test_negative_pair_tolerance_is_rejected(self):
        with self.assertRaises(ValueError):
            pair_frames([], [], -1)


if __name__ == "__main__":
    unittest.main()
