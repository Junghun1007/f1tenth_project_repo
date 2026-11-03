from copy import deepcopy
import math
import unittest
from auto_control_tunnel.blending import PathRules, metric_path, mix_paths, stamp_ns
from lane_fixture import lane, ns


class BlendingTests(unittest.TestCase):
    def setUp(self):
        self.rules = PathRules()

    def mix(self, old, new, weight=.5):
        return mix_paths(old, new, metric_path(old, self.rules), metric_path(new, self.rules),
                         weight, self.rules, .2, math.radians(20))

    def test_matches_forward_distance_with_different_padding_density_and_extent(self):
        a = lane(1, slope=.05, start=.1, count=50, step=.02, padding=0)
        b = lane(1.02, lateral=.1, slope=.05, start=.15, count=40, step=.025, padding=30)
        mixed, reason = self.mix(a, b)
        self.assertEqual(reason, 'blending')
        for x, y, _ in metric_path(mixed, self.rules):
            self.assertAlmostEqual(y, .05 + .05 * x)
        self.assertEqual(stamp_ns(mixed), ns(1))
        self.assertNotEqual(a.centerline_points, mixed.centerline_points)
        self.assertTrue(a.stop_line_present)
        self.assertFalse(mixed.stop_line_present)
        self.assertTrue(math.isnan(mixed.stop_line_distance_m))

    def test_no_curve_extrapolation_and_minimum_overlap(self):
        result, reason = self.mix(lane(1, start=.1, count=20), lane(1.01, start=.6, count=20))
        self.assertIsNone(result)
        self.assertEqual(reason, 'insufficient_overlap')

    def test_rejects_heading_disagreement_and_nonmonotonic_forward_curve(self):
        a, b = lane(1, start=.1, count=15), lane(1.01, lateral=-.12, slope=.5, start=.1, count=15)
        self.assertEqual(self.mix(a, b)[1], 'heading_disagreement')
        a = lane(1)
        a.centerline_points[10].y = a.centerline_points[9].y + 1.
        a.centerline_points[10].x += 1.
        self.assertEqual(self.mix(a, lane(1.01))[1], 'nonmonotonic_forward_path')

    def test_invalid_controller_inputs_never_become_blend_candidates(self):
        for mutate in (
            lambda m: setattr(m, 'state', 0),
            lambda m: setattr(m, 'centerline_valid', False),
            lambda m: setattr(m, 'centerline_sample_limit_reached', True),
            lambda m: setattr(m.header, 'frame_id', 'wrong'),
            lambda m: setattr(m, 'centerline_bev_width_m', 2.),
            lambda m: setattr(m, 'source_width', 0),
            lambda m: m.centerline_support.pop(),
            lambda m: m.centerline_support.__setitem__(0, 99),
            lambda m: setattr(m.centerline_points[0], 'z', 1.),
            lambda m: setattr(m.centerline_points[0], 'x', float('nan')),
        ):
            m = deepcopy(lane(1))
            mutate(m)
            self.assertIsNone(metric_path(m, self.rules))
        self.assertIsNone(metric_path(lane(1, count=5), self.rules))
        self.assertIsNone(metric_path(lane(1, count=8, step=.01), self.rules))
        self.assertIsNone(metric_path(lane(1, count=10, step=.2), self.rules))

    def test_configured_controller_limits_are_used(self):
        self.assertIsNotNone(metric_path(lane(1), self.rules))
        self.assertIsNone(metric_path(lane(1), PathRules(minimum_points=60)))


if __name__ == '__main__':
    unittest.main()
