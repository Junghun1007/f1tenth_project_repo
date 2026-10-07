import unittest
from types import SimpleNamespace
import numpy as np
from tunnel_tcn_test.features import FeatureStream, EventDecoder, histograms, static_features
from tunnel_tcn_test.display import mono_image, footer


class FeatureTests(unittest.TestCase):
    def test_black_image(self):
        actual = static_features(histograms(np.zeros((400, 640), np.uint8)))
        expected = np.zeros(28, np.float32)
        expected[16:18] = 1.
        np.testing.assert_array_equal(actual, expected)

    def test_uniform_gray(self):
        actual = static_features(histograms(np.full((400, 640), 128, np.uint8)))
        np.testing.assert_allclose(actual[[0, 2, 3, 4, *range(5, 15), *range(20, 28)]], 128 / 255., atol=1e-7)
        self.assertAlmostEqual(float(actual[1]), 0., places=6)
        self.assertEqual(actual[15], 0.)

    def test_nominal_context_and_rates(self):
        for fps in [10., 30.]:
            stream = FeatureStream()
            for frame in range(int(fps * 3)):
                row = np.full(28, frame / fps / 10., dtype=np.float32)
                result = stream.push_static(row, frame / fps)
                expected = np.maximum(frame - np.rint(np.arange(20, -1, -1) * fps / 10), 0) / fps / 10
                np.testing.assert_allclose(result[0, :, 0], expected, atol=1e-7)
                self.assertEqual(stream.ready, frame / fps >= 2.5)
            self.assertAlmostEqual(float(result[0, -1, 29]), .1, places=5)
            n = round(.5 * fps)
            self.assertAlmostEqual(float(result[0, -1, 28]), np.mean(np.arange(frame - n + 1, frame + 1) / fps / 10), places=6)

    def test_missing_sample_stays_causal_and_rate_uses_elapsed_time(self):
        stream = FeatureStream(warmup_seconds=0)
        stream.push_static(np.zeros(28), 0.)
        result = stream.push_static(np.full(28, .02), .2)
        self.assertAlmostEqual(float(result[0, -1, 29]), .1, places=6)
        self.assertEqual(result[0, -2, 0], 0.)  # No future sample for t=0.1.

    def test_large_gap_rebuilds_history(self):
        stream = FeatureStream()
        stream.push_static(np.zeros(28), 0.)
        result = stream.push_static(np.ones(28), 1.)
        self.assertTrue(stream.discontinuity)
        self.assertFalse(stream.ready)
        np.testing.assert_array_equal(result[0, :, 29:], 0.)

    def test_duplicate_time_starts_new_history(self):
        stream = FeatureStream()
        stream.push_static(np.zeros(28), 2.)
        stream.push_static(np.ones(28), 2.)
        self.assertTrue(stream.discontinuity)
        self.assertEqual(len(stream.samples), 1)


class DecoderTests(unittest.TestCase):
    def test_first_exit_allowed_then_alternating(self):
        d = EventDecoder()
        self.assertEqual(d.step([.01, .01, .98], 0)['type'], 'exit')
        d.step([1., 0., 0.], .1)
        self.assertIsNone(d.step([0., 0., 1.], .2))
        self.assertEqual(d.step([0., 1., 0.], .3)['type'], 'entry')
        self.assertEqual(d.state, 'inside')

    def test_pause_preserves_state_and_reset_clears(self):
        d = EventDecoder()
        d.step([0., 1., 0.], 0)
        d.pause()
        self.assertIsNone(d.step([0., 1., 0.], 10))
        self.assertEqual(d.state, 'inside')
        d.reset()
        self.assertEqual(d.state, 'unknown')
        self.assertIsNotNone(d.step([0., 1., 0.], 0))

    def test_low_confidence_does_not_erase_state(self):
        d = EventDecoder()
        d.step([0., 1., 0.], 0)
        self.assertIsNone(d.step([.1, .3, .6], 9))
        self.assertEqual(d.state, 'inside')


class DisplayTests(unittest.TestCase):
    def test_padded_mono_message(self):
        m = SimpleNamespace(encoding='mono8', width=2, height=2, step=3, data=bytes([1, 2, 99, 3, 4, 99]))
        np.testing.assert_array_equal(mono_image(m), [[1, 2], [3, 4]])
        m.data = b''
        with self.assertRaises(ValueError):
            mono_image(m)

    def test_footer_preserves_camera_pixels(self):
        frame = np.full((600, 960), 83, np.uint8)
        rendered = footer(frame, [.01, .98, .01], 'inside', 'ENTRY 0.980', 'LIVE', 30., .3, 5., 0, '30000us ISO800 dot1.00')
        self.assertEqual(rendered.shape, (760, 960, 3))
        np.testing.assert_array_equal(rendered[:600], np.full((600, 960, 3), 83, np.uint8))
        self.assertGreater(int(rendered[600:].max()), 200)


if __name__ == '__main__':
    unittest.main()
