import unittest
from auto_control_tunnel.selection import LaneSelector
from auto_control_tunnel.blending import metric_path, stamp_ns
from lane_fixture import lane, ns


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.selector = LaneSelector(validation_mode=True)

    def send(self, source, t, **kwargs):
        message = lane(t, **kwargs)
        return self.selector.lane(source, message, ns(t + .005))

    def entry(self, t=1.):
        self.selector.event('entry', ns(t), ns(t))

    def test_outside_routes_original_only(self):
        m = lane(.95)
        self.assertIs(self.selector.lane('RGB', m, ns(.96)), m)
        self.assertIsNone(self.send('IR', .98))
        self.assertEqual(self.selector.compute, 'RGB')

    def test_entry_prepares_ir_without_cutting_live_rgb(self):
        self.send('RGB', .98)
        self.entry()
        self.assertEqual((self.selector.state, self.selector.selected, self.selector.camera,
                          self.selector.compute), ('ENTERING', 'RGB', 'IR', 'BOTH'))
        for t in (1.01, 1.04, 1.08, 1.12, 1.16, 1.20):
            self.assertIsNotNone(self.send('RGB', t))
        self.assertEqual(self.selector.selected, 'RGB')

    def test_first_valid_target_blends_then_commits_without_confirming_frames(self):
        self.entry()
        self.send('RGB', 1.02)
        first = self.send('IR', 1.04, lateral=.1)
        # Its conservative stamp equals the already delivered RGB stamp.
        self.assertIsNone(first)
        self.assertEqual(self.selector.weight, 0.)
        self.send('RGB', 1.10)
        middle = self.send('IR', 1.12, lateral=.1)
        self.assertTrue(0.04 < metric_path(middle, self.selector.rules)[0][1] < .07)
        self.assertFalse(middle.stop_line_present)
        self.assertLessEqual(stamp_ns(middle), ns(1.10))
        self.send('RGB', 1.18)
        self.assertIsNotNone(self.send('IR', 1.21, lateral=.1))
        self.assertEqual((self.selector.state, self.selector.selected, self.selector.compute),
                         ('INSIDE', 'IR', 'IR'))
        self.assertEqual(self.selector.last_handover['reason'], 'blend_complete')

    def test_exit_keeps_ir_profile_and_control_until_rgb_blend_finishes(self):
        self.entry()
        self.send('IR', 1.01)  # No usable RGB: target alone takes over immediately.
        self.selector.event('exit', ns(2), ns(2))
        self.assertEqual((self.selector.state, self.selector.selected, self.selector.compute,
                          self.selector.camera), ('EXITING', 'IR', 'BOTH', 'IR'))
        self.send('IR', 2.01)
        self.send('RGB', 2.02, lateral=.05)
        self.send('IR', 2.16)
        self.send('RGB', 2.19, lateral=.05)
        self.assertEqual((self.selector.state, self.selector.camera, self.selector.compute),
                         ('OUTSIDE', 'RGB', 'RGB'))

    def test_invalid_target_does_not_clear_a_good_origin(self):
        self.entry()
        self.send('RGB', 1.02)
        self.assertIsNone(self.send('IR', 1.03, valid=False))
        self.assertIsNotNone(self.send('RGB', 1.04))
        self.assertIsNone(self.selector.first_target_ns)

    def test_recent_origin_before_event_can_contribute_without_new_origin_frame(self):
        self.send('RGB', .99)
        self.entry()
        self.send('IR', 1.02, lateral=.08)
        self.assertEqual(self.selector.reason, 'blending')
        self.assertTrue(self.selector.transitioning)

    def test_pair_skew_prevents_mixing_even_when_both_results_are_fresh(self):
        self.entry()
        self.send('RGB', 1.01)
        result = self.send('IR', 1.12, lateral=.08)
        self.assertAlmostEqual(metric_path(result, self.selector.rules)[0][1], .08)
        self.assertEqual(self.selector.reason, 'timestamp_skew')

    def test_target_during_handover_must_be_fresh_even_in_validation_mode(self):
        self.entry()
        self.send('RGB', 1.10)
        stale = lane(.98, lateral=.08)
        self.assertIsNone(self.selector.lane('IR', stale, ns(1.11)))
        self.assertEqual(self.selector.selected, 'RGB')
        self.assertIsNone(self.selector.first_target_ns)

    def test_target_must_pass_controller_geometry_not_just_valid_flag(self):
        self.entry()
        self.send('RGB', 1.02)
        self.assertIsNone(self.send('IR', 1.03, count=5))
        self.assertIsNone(self.selector.first_target_ns)
        self.assertEqual(self.selector.selected, 'RGB')

    def test_invalid_origin_allows_new_valid_target_without_replaying_cache(self):
        self.entry()
        self.send('RGB', 1.02)
        self.send('IR', 1.03, lateral=.05)
        self.assertIsNone(self.send('RGB', 1.04, valid=False))
        self.assertIsNotNone(self.send('IR', 1.05, lateral=.05))
        self.assertEqual(self.selector.selected, 'IR')
        self.assertEqual(self.selector.reason, 'target_only')

    def test_both_invalid_reach_controller_and_do_not_keep_old_path_alive(self):
        self.entry()
        self.send('RGB', 1.02, valid=False)
        result = self.send('IR', 1.03, valid=False)
        self.assertIsNotNone(result)
        self.assertFalse(result.centerline_valid)

    def test_expired_contributor_is_not_blended_or_republished(self):
        self.entry()
        self.send('RGB', 1.01)
        result = self.send('IR', 1.20, lateral=.1)
        self.assertAlmostEqual(metric_path(result, self.selector.rules)[0][1], .1)
        self.assertEqual(self.selector.reason, 'target_only')

    def test_large_disagreement_selects_new_target_without_averaging(self):
        self.entry()
        self.send('RGB', 1.01)
        result = self.send('IR', 1.03, lateral=.3)
        self.assertAlmostEqual(metric_path(result, self.selector.rules)[0][1], .3)
        self.assertEqual(self.selector.reason, 'lateral_disagreement')

    def test_profile_deadline_does_not_replay_any_lane(self):
        self.entry()
        self.selector.tick(ns(1.751))
        self.assertEqual(self.selector.selected, 'IR')
        self.assertEqual(self.selector.compute, 'IR')
        self.assertEqual(self.selector.reason, 'target_wait_timeout')
        self.assertEqual(self.selector.last_output_ns, 0)
        self.assertIsNone(self.send('RGB', 1.76))

    def test_duplicate_tcn_state_does_not_restart_deadline(self):
        self.entry()
        self.assertFalse(self.selector.event('entry', ns(1.1), ns(1.1)))
        self.assertEqual(self.selector.started_ns, ns(1))

    def test_reverse_during_blend_cancels_without_waiting(self):
        self.entry()
        self.send('RGB', 1.01)
        self.send('IR', 1.02)
        self.selector.event('exit', ns(1.1), ns(1.1))
        self.assertEqual((self.selector.state, self.selector.compute, self.selector.camera),
                         ('OUTSIDE', 'RGB', 'RGB'))
        self.assertIsNotNone(self.send('RGB', 1.12))

    def test_previous_visit_and_duplicate_results_cannot_trigger_handover(self):
        self.send('IR', .9)
        self.entry()
        self.assertIsNone(self.selector.first_target_ns)
        self.assertIsNone(self.send('IR', .9))
        self.send('RGB', 1.01)
        self.assertIsNone(self.send('RGB', 1.01))

    def test_stale_future_and_cross_source_reverse_stamps_are_never_restamped(self):
        s = LaneSelector()
        self.assertIsNone(s.lane('RGB', lane(1), ns(1.3)))
        self.assertIsNone(s.lane('RGB', lane(1), ns(.9)))
        self.assertIsNotNone(s.lane('RGB', lane(2), ns(2.01)))
        s.event('entry', ns(2.02), ns(2.02))
        self.assertIsNone(s.lane('IR', lane(1.99), ns(2.03)))
        self.assertEqual(s.last_output_ns, ns(2))

    def test_guarded_event_age_checks_remain(self):
        s = LaneSelector()
        self.assertFalse(s.event('entry', ns(1), ns(2)))
        self.assertFalse(s.event('entry', ns(3), ns(2)))
        self.assertTrue(s.event('entry', ns(2), ns(2)))
        self.assertFalse(s.event('exit', ns(2), ns(2)))

    def test_transition_continues_without_tcn_reports_in_validation_mode(self):
        self.entry()
        self.assertIsNotNone(self.send('RGB', 1.04))
        self.send('IR', 1.06)
        self.send('RGB', 1.20)
        self.send('IR', 1.23)
        self.assertEqual(self.selector.state, 'INSIDE')


if __name__ == '__main__':
    unittest.main()
