from copy import deepcopy
from types import SimpleNamespace as NS
import unittest
from auto_control_tunnel.selection import LaneSelector, lane_ready


def ns(seconds):
    return int(seconds * 1e9)


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.selector = LaneSelector()

    def lane(self, source, seconds, ready=True, delay=.01):
        return self.selector.lane(source, ns(seconds), ns(seconds + delay), ready)

    def test_outside_starts_with_rgb_only(self):
        self.assertEqual(self.selector.compute, 'RGB')
        self.assertTrue(self.lane('RGB', 1.))
        self.assertFalse(self.lane('IR', 1.1))

    def test_entry_requires_three_fresh_post_request_results(self):
        self.assertTrue(self.selector.event('entry', ns(1), ns(1.05)))
        self.assertEqual(self.selector.compute, 'BOTH')
        self.assertEqual(self.selector.camera, 'IR')
        self.assertFalse(self.lane('IR', 1.02))  # cached standby result
        self.assertTrue(self.lane('RGB', 1.06))
        self.assertFalse(self.lane('IR', 1.08))
        self.assertFalse(self.lane('IR', 1.12))
        self.assertTrue(self.lane('IR', 1.16))
        self.assertEqual(self.selector.state, 'INSIDE')
        self.assertFalse(self.lane('RGB', 1.2))

    def test_exit_keeps_ir_exposure_and_path_until_rgb_ready(self):
        self.selector.selected = self.selector.requested = 'IR'
        self.selector.state = 'INSIDE'
        self.assertTrue(self.selector.event('exit', ns(2), ns(2.01)))
        self.assertEqual(self.selector.camera, 'IR')
        self.assertEqual(self.selector.compute, 'BOTH')
        self.assertTrue(self.lane('IR', 2.02))
        for stamp in (2.04, 2.08):
            self.assertFalse(self.lane('RGB', stamp))
        self.assertTrue(self.lane('RGB', 2.12))
        self.assertEqual(self.selector.state, 'OUTSIDE')
        self.assertEqual(self.selector.camera, 'RGB')

    def test_invalid_path_resets_candidate_confirmation(self):
        self.selector.event('entry', ns(1), ns(1))
        self.lane('IR', 1.04)
        self.lane('IR', 1.08, ready=False)
        self.assertFalse(self.lane('IR', 1.12))
        self.assertFalse(self.lane('IR', 1.16))
        self.assertEqual(self.selector.selected, 'RGB')
        self.assertTrue(self.lane('IR', 1.20))

    def test_candidate_gap_resets_confirmation(self):
        self.selector.event('entry', ns(1), ns(1))
        self.lane('IR', 1.04)
        self.lane('IR', 1.40)
        self.lane('IR', 1.44)
        self.assertEqual(self.selector.selected, 'RGB')
        self.assertTrue(self.lane('IR', 1.48))

    def test_stale_future_duplicate_results_never_refresh_controller(self):
        self.assertFalse(self.lane('RGB', 1., delay=.3))
        self.assertFalse(self.lane('RGB', 1., delay=-.1))
        self.assertTrue(self.lane('RGB', 2.))
        self.assertFalse(self.lane('RGB', 2.))
        self.assertFalse(self.lane('RGB', 1.9))

    def test_cross_camera_timestamps_never_go_backwards(self):
        self.selector = LaneSelector(confirmations=1)
        self.selector.event('entry', ns(1), ns(1))
        self.assertTrue(self.lane('RGB', 1.1))
        self.assertFalse(self.lane('IR', 1.09))
        self.assertTrue(self.lane('IR', 1.12))

    def test_opposite_event_cancels_unfinished_entry(self):
        self.selector.event('entry', ns(1), ns(1))
        self.lane('IR', 1.04)
        self.selector.event('exit', ns(2), ns(2))
        self.assertEqual(self.selector.state, 'OUTSIDE')
        self.assertEqual(self.selector.camera, 'RGB')
        self.assertFalse(self.lane('IR', 2.04))

    def test_replayed_and_stale_events_do_not_change_mode(self):
        self.assertFalse(self.selector.event('entry', ns(1), ns(2)))
        self.assertFalse(self.selector.event('entry', ns(3), ns(2)))
        self.assertTrue(self.selector.event('entry', ns(2), ns(2)))
        self.assertFalse(self.selector.event('exit', ns(2), ns(2)))
        self.assertEqual(self.selector.state, 'ENTERING')


class CandidateGeometryTests(unittest.TestCase):
    def setUp(self):
        self.rules = dict(minimum_x=.01, maximum_x=2.98, minimum_points=8,
                          maximum_gap=.15, minimum_span=.12)
        self.message = NS(centerline_valid=True, centerline_sample_limit_reached=False, state=3,
                          header=NS(frame_id='front_axle_bev'), source_width=120, source_height=300,
                          centerline_bev_width_m=1.2, centerline_bev_height_m=3.,
                          padding_left=20, padding_right=20,
                          centerline_points=[NS(x=79.5, y=290. - i * 3., z=0.) for i in range(10)],
                          centerline_support=[2]*10)

    def test_accepts_metric_contiguous_path_with_padding(self):
        self.assertTrue(lane_ready(self.message, self.rules))

    def test_rejects_wrong_geometry_nonfinite_points_and_missing_support(self):
        for field, value in [('centerline_valid', False), ('centerline_sample_limit_reached', True),
                             ('state', 0), ('source_width', 300), ('centerline_bev_width_m', 3.),
                             ('centerline_support', [])]:
            message = deepcopy(self.message)
            setattr(message, field, value)
            self.assertFalse(lane_ready(message, self.rules), field)
        self.message.centerline_points[0].x = float('nan')
        self.assertFalse(lane_ready(self.message, self.rules))

    def test_does_not_join_disconnected_fragments(self):
        self.message.centerline_points[4].y -= 40
        self.assertFalse(lane_ready(self.message, self.rules))


if __name__ == '__main__':
    unittest.main()
