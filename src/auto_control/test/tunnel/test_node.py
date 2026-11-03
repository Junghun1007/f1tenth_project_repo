"""Transport/clock doubles exercise the real selector callbacks without ROS."""
import importlib.util
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch
from auto_control_tunnel.selection import LaneSelector


def load_node():
    modules = {name: ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'std_msgs', 'std_msgs.msg',
        'line_detactor', 'line_detactor.msg')}
    modules['rclpy.node'].Node = object
    for name in ('QoSProfile', 'ReliabilityPolicy', 'DurabilityPolicy'):
        setattr(modules['rclpy.qos'], name, object)
    modules['std_msgs.msg'].String = NS
    modules['line_detactor.msg'].LaneResult = object
    path = Path(__file__).resolve().parents[2] / 'auto_control_tunnel/node.py'
    spec = importlib.util.spec_from_file_location('auto_control_tunnel._node_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


class CallbackTests(unittest.TestCase):
    def setUp(self):
        self.module = load_node()
        self.node = self.module.TunnelLaneSelector.__new__(self.module.TunnelLaneSelector)
        n = self.node
        n.preview = self.module.TunnelPreviewTelemetry()
        self.message = NS(header=NS(stamp=NS(sec=10, nanosec=0)), centerline_valid=False)
        n.policy = LaneSelector()
        n.last_tcn_received, n.tcn_live = None, False
        n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_010_000_000))
        n.get_logger = lambda: NS(warning=lambda _: None)
        n.publish_status = lambda: None
        self.outputs = []
        n.output = NS(publish=self.outputs.append)

    def report(self, **overrides):
        report = dict(source_stamp_ns=10_000_000_000, status='LIVE', state='outside', event=None)
        report.update(overrides)
        self.node.on_event(NS(data=json.dumps(report)))

    def test_live_tcn_routes_original_capture_without_rewriting_stamp(self):
        self.report()
        self.node.on_lane('RGB', self.message)
        self.assertEqual(self.outputs, [self.message])
        self.assertFalse(self.outputs[0].centerline_valid)
        self.assertEqual(self.outputs[0].header.stamp.sec, 10)

    def test_warmup_and_tcn_silence_withhold_lane_results(self):
        self.node.on_lane('RGB', self.message)
        self.report(status='WARMUP 2.5s')
        self.node.on_lane('RGB', self.message)
        self.report()
        with patch.object(self.module.time, 'monotonic', return_value=self.node.last_tcn_received + .6):
            self.node.on_lane('RGB', self.message)
        self.assertEqual(self.outputs, [])

    def test_real_tcn_event_schema_requests_entry(self):
        self.report(state='inside', event=dict(type='entry', score=.9, time_s=3.))
        self.assertEqual(self.node.policy.state, 'INSIDE')
        self.assertEqual(self.node.policy.selected, 'IR')

    def test_retained_tcn_state_recovers_lost_event_without_repeating_switch(self):
        self.report(state='inside')
        self.assertEqual(self.node.policy.state, 'INSIDE')
        event_stamp = self.node.policy.last_event_ns
        self.report(state='inside', source_stamp_ns=10_005_000_000)
        self.assertEqual(self.node.policy.last_event_ns, event_stamp)

    def test_stale_or_malformed_tcn_does_not_start_motion_or_transition(self):
        for data in ('[]', '{', '{"source_stamp_ns": true}',
                     json.dumps(dict(source_stamp_ns=9_000_000_000, status='LIVE',
                                     event=dict(type='entry')))):
            self.node.on_event(NS(data=data))
        self.assertFalse(self.node.tcn_live)
        self.assertEqual(self.node.policy.state, 'OUTSIDE')


if __name__ == '__main__':
    unittest.main()
