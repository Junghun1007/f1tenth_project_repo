import unittest

from auto_control_tunnel.remote_frame import RateWindow, build_frame, centerline_metric, tunnel_state


class CenterlineTests(unittest.TestCase):
    def test_centerline_matches_controller_formula(self):
        points = centerline_metric([(30.5, 299.5), (30.5, 0.5)], 120, 300, 0, 1.2, 3.0)
        for got, expected in zip(points, [[0.0, 0.29], [2.99, 0.29]]):
            self.assertAlmostEqual(got[0], expected[0], places=3)
            self.assertAlmostEqual(got[1], expected[1], places=3)

    def test_centerline_downsamples_keeping_endpoints(self):
        raw = [(60.0, 299.0 - 3 * i) for i in range(100)]
        points = centerline_metric(raw, 120, 300, 0, 1.2, 3.0)
        self.assertEqual(len(points), 24)
        self.assertEqual(points[0], centerline_metric(raw[:2], 120, 300, 0, 1.2, 3.0)[0])
        self.assertEqual(points[-1], centerline_metric(raw[-2:], 120, 300, 0, 1.2, 3.0)[-1])

    def test_centerline_invalid_returns_empty(self):
        self.assertEqual(centerline_metric([(1, 1)], 120, 300, 0, 1.2, 3.0), [])
        self.assertEqual(centerline_metric([(1, 1), (2, 2)], 120, 0, 0, 1.2, 3.0), [])
        self.assertEqual(centerline_metric([(1, 1), (2, 2)], 120, 300, 0, float('nan'), 3.0), [])


class RateTests(unittest.TestCase):
    def test_rate_window_fps_and_mean(self):
        window = RateWindow()
        for i in range(10):
            window.add(i * 0.1, 2.0)
        self.assertAlmostEqual(window.fps(0.9), 10.0, delta=1.2)
        self.assertEqual(window.mean(0.9), 2.0)
        self.assertAlmostEqual(window.age_ms(1.0), 100.0)
        self.assertIsNone(window.fps(2.95))


class FrameTests(unittest.TestCase):
    def test_build_frame_marks_stale_values_none(self):
        now = 100.0
        frame = build_frame(1, now, {'vehicle': {'speed_mps': (1.2, now - 0.6)}})
        self.assertIsNone(frame['vehicle']['speed_mps'])
        frame = build_frame(2, now, {'vehicle': {'speed_mps': (1.2, now - 0.1)}})
        self.assertEqual(frame['vehicle']['speed_mps'], 1.2)
        for key in ('v', 'seq', 't', 'vehicle', 'lane', 'fps', 'traffic', 'tunnel', 'system',
                    'startup', 'link'):
            self.assertIn(key, frame)
        self.assertEqual(frame['lane']['points'], [])

    def test_tunnel_state_parsed_from_preview_status(self):
        self.assertEqual(tunnel_state('Tunnel: INSIDE\nTCN avg 12.0FPS\nTCN LIVE'), 'INSIDE')
        self.assertIsNone(tunnel_state('garbage'))


if __name__ == '__main__':
    unittest.main()
