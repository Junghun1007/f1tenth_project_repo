from copy import deepcopy
import unittest
from auto_control_tunnel.selection import LaneSelector
from auto_control_tunnel.path_validation import PathRules, metric_path, stamp_ns
from lane_fixture import lane, ns


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.selector = LaneSelector(validation_mode=True)

    def send(self, source, t, **kwargs):
        return self.selector.lane(source, lane(t, **kwargs), ns(t + .005))

    def entry(self, t=1.):
        self.selector.event('entry', ns(t), ns(t))

    def test_outside_routes_original_only(self):
        message = lane(.95)
        self.assertIs(self.selector.lane('RGB', message, ns(.96)), message)
        self.assertEqual(message.lane_source, 'RGB')
        self.assertIsNone(self.send('IR', .98))
        self.assertEqual(self.selector.compute, 'RGB')

    def test_entry_selects_ir_and_cuts_rgb_before_any_ir_result(self):
        self.send('RGB', .98)
        self.entry()
        self.assertEqual((self.selector.state, self.selector.selected, self.selector.camera,
                          self.selector.compute), ('INSIDE', 'IR', 'IR', 'IR'))
        for t in (1.01, 1.04, 1.08, 1.12, 1.16, 1.20):
            self.assertIsNone(self.send('RGB', t))
        self.assertEqual(self.selector.last_handover,
                         dict(target='IR', reason='tcn_entry', total_ms=0.))

    def test_ir_result_is_forwarded_without_blending(self):
        self.entry()
        target = lane(1.04, lateral=.1)
        before = deepcopy(target)
        result = self.selector.lane('IR', target, ns(1.045))
        self.assertIs(result, target)
        self.assertEqual(result.centerline_points, before.centerline_points)
        self.assertEqual(result.centerline_support, before.centerline_support)
        self.assertEqual(result.header, before.header)
        self.assertEqual(result.stop_line_present, before.stop_line_present)
        self.assertEqual(result.stop_line_distance_m, before.stop_line_distance_m)
        self.assertAlmostEqual(metric_path(result, PathRules())[0][1], .1)
        self.assertEqual((self.selector.state, self.selector.selected, self.selector.compute),
                         ('INSIDE', 'IR', 'IR'))
        self.assertIsNone(self.send('RGB', 1.05))

    def test_exit_selects_rgb_and_cuts_ir_before_any_rgb_result(self):
        self.entry()
        self.send('IR', 1.01)
        self.selector.event('exit', ns(2), ns(2))
        self.assertEqual((self.selector.state, self.selector.selected, self.selector.compute,
                          self.selector.camera), ('OUTSIDE', 'RGB', 'RGB', 'RGB'))
        self.assertIsNone(self.send('IR', 2.01))
        self.assertEqual(self.selector.last_handover,
                         dict(target='RGB', reason='tcn_exit', total_ms=0.))

    def test_first_ir_result_is_forwarded_even_if_centerline_invalid(self):
        self.entry()
        result = self.send('IR', 1.03, valid=False)
        self.assertFalse(result.centerline_valid)
        self.assertIsNone(self.send('RGB', 1.04))

    def test_first_rgb_result_is_forwarded_even_if_centerline_invalid(self):
        self.entry()
        self.assertIsNotNone(self.send('IR', 1.03, count=5))
        self.selector.event('exit', ns(2), ns(2))
        result = self.send('RGB', 2.01, valid=False, count=5)
        self.assertFalse(result.centerline_valid)
        self.assertEqual(self.selector.selected, 'RGB')

    def test_duplicate_entry_does_not_repeat_switch(self):
        self.entry()
        self.assertFalse(self.selector.event('entry', ns(1.1), ns(1.1)))
        self.assertEqual(self.selector.selected, 'IR')

    def test_reentry_selects_ir_without_any_intervening_rgb_results(self):
        self.entry()
        self.selector.event('exit', ns(2), ns(2))
        self.entry(2.1)
        self.assertEqual((self.selector.state, self.selector.compute, self.selector.camera),
                         ('INSIDE', 'IR', 'IR'))
        self.assertIsNotNone(self.send('IR', 2.12))

    def test_duplicate_ir_results_are_dropped(self):
        self.send('IR', .9)
        self.entry()
        self.assertIsNone(self.send('IR', .9))
        self.assertEqual(self.selector.state, 'INSIDE')
        self.assertIsNotNone(self.send('IR', 1.02))
        self.assertIsNone(self.send('IR', 1.02))

    def test_new_valid_target_can_be_older_than_last_other_camera_capture(self):
        self.send('RGB', .95)
        self.entry()
        self.send('IR', 1.05)
        self.selector.event('exit', ns(2), ns(2))
        self.selector.lane('IR', lane(2.01), ns(2.02))
        target = lane(1.95, lateral=.1)
        result = self.selector.lane('RGB', target, ns(2.03))
        self.assertIs(result, target)
        self.assertEqual(stamp_ns(result), ns(1.95))
        self.assertEqual(result.lane_source, 'RGB')
        self.assertEqual(self.selector.state, 'OUTSIDE')
        self.assertIsNone(self.selector.lane('RGB', lane(1.94), ns(2.04)))

    def test_stale_and_future_rgb_do_not_undo_immediate_exit(self):
        self.entry()
        self.selector.event('exit', ns(2), ns(2))
        for stamp, now in ((1.8, 2.1), (2.2, 2.1)):
            self.assertIsNone(self.selector.lane('RGB', lane(stamp), ns(now)))
            self.assertEqual(self.selector.state, 'OUTSIDE')
        self.assertIsNotNone(self.send('RGB', 2.15))

    def test_disagreement_does_not_create_a_third_path_or_add_confirmation(self):
        self.entry()
        self.send('RGB', 1.01)
        target = lane(1.03, lateral=.3)
        self.assertIs(self.selector.lane('IR', target, ns(1.04)), target)
        self.assertEqual(self.selector.selected, 'IR')

    def test_guarded_event_age_checks_remain(self):
        selector = LaneSelector()
        self.assertFalse(selector.event('entry', ns(1), ns(2)))
        self.assertFalse(selector.event('entry', ns(3), ns(2)))
        self.assertTrue(selector.event('entry', ns(2), ns(2)))
        self.assertFalse(selector.event('exit', ns(2), ns(2)))


if __name__ == '__main__':
    unittest.main()
