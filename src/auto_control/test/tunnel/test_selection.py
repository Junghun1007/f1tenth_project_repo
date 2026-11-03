import unittest
from auto_control_tunnel.selection import LaneSelector


def ns(seconds):
    return int(seconds * 1e9)


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.selector = LaneSelector()

    def lane(self, source, seconds, delay=.01):
        return self.selector.lane(source, ns(seconds), ns(seconds + delay))

    def test_outside_starts_with_rgb_only(self):
        self.assertEqual(self.selector.compute, 'RGB')
        self.assertTrue(self.lane('RGB', 1.))
        self.assertFalse(self.lane('IR', 1.1))

    def test_entry_switches_before_any_ir_result(self):
        self.assertTrue(self.selector.event('entry', ns(1), ns(1.05)))
        self.assertEqual((self.selector.state, self.selector.selected,
                          self.selector.compute, self.selector.camera),
                         ('INSIDE', 'IR', 'IR', 'IR'))
        self.assertFalse(self.lane('RGB', 1.06))
        self.assertTrue(self.lane('IR', 1.08))

    def test_exit_restores_rgb_immediately(self):
        self.selector.event('entry', ns(1), ns(1))
        self.assertTrue(self.selector.event('exit', ns(2), ns(2.01)))
        self.assertEqual((self.selector.state, self.selector.selected,
                          self.selector.compute, self.selector.camera),
                         ('OUTSIDE', 'RGB', 'RGB', 'RGB'))
        self.assertFalse(self.lane('IR', 2.02))
        self.assertTrue(self.lane('RGB', 2.04))

    def test_stale_future_duplicate_results_never_refresh_controller(self):
        self.assertFalse(self.lane('RGB', 1., delay=.3))
        self.assertFalse(self.lane('RGB', 1., delay=-.1))
        self.assertTrue(self.lane('RGB', 2.))
        self.assertFalse(self.lane('RGB', 2.))
        self.assertFalse(self.lane('RGB', 1.9))

    def test_cross_camera_timestamps_never_go_backwards(self):
        self.assertTrue(self.lane('RGB', 1.1))
        self.selector.event('entry', ns(1.1), ns(1.11))
        self.assertFalse(self.lane('IR', 1.09))
        self.assertTrue(self.lane('IR', 1.12))

    def test_opposite_event_switches_without_waiting_for_frames(self):
        self.selector.event('entry', ns(1), ns(1))
        self.selector.event('exit', ns(2), ns(2))
        self.assertEqual(self.selector.state, 'OUTSIDE')
        self.assertFalse(self.lane('IR', 2.04))

    def test_validation_uses_standalone_ir_timing_and_does_not_cross_compare_camera_clocks(self):
        self.selector = LaneSelector(validation_mode=True)
        self.assertTrue(self.lane('RGB', 10.))
        self.assertTrue(self.selector.event('entry', ns(1.), ns(10.)))
        self.assertTrue(self.lane('IR', 9.5, delay=.6))
        self.assertFalse(self.lane('IR', 9.5, delay=.6))
        self.assertTrue(self.selector.event('exit', ns(2.), ns(10.)))
        self.assertTrue(self.lane('RGB', 10.1))
        self.assertFalse(self.lane('RGB', 10.2, delay=.6)) # Outside keeps capture-age checks.

    def test_replayed_and_stale_events_do_not_change_mode(self):
        self.assertFalse(self.selector.event('entry', ns(1), ns(2)))
        self.assertFalse(self.selector.event('entry', ns(3), ns(2)))
        self.assertTrue(self.selector.event('entry', ns(2), ns(2)))
        self.assertFalse(self.selector.event('exit', ns(2), ns(2)))
        self.assertEqual(self.selector.state, 'INSIDE')


if __name__ == '__main__':
    unittest.main()
