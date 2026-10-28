"""Exercise real node callbacks with transport/clock/GPU doubles, without ROS."""
import importlib.util
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
import unittest
from collections import deque
import numpy as np
from tunnel_tcn_test.features import FeatureStream, EventDecoder


def load_node():
    modules = {}
    for name in ['rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg',
                 'std_msgs', 'std_msgs.msg', 'ament_index_python', 'ament_index_python.packages']:
        modules[name] = ModuleType(name)
    modules['rclpy.node'].Node = object
    for name in ['QoSProfile', 'ReliabilityPolicy', 'DurabilityPolicy', 'HistoryPolicy']:
        setattr(modules['rclpy.qos'], name, object)
    modules['sensor_msgs.msg'].Image = object
    modules['std_msgs.msg'].String = SimpleNamespace
    modules['ament_index_python.packages'].get_package_share_directory = lambda _: ''
    path = Path(__file__).resolve().parents[1] / 'tunnel_tcn_test/node.py'
    spec = importlib.util.spec_from_file_location('tunnel_tcn_test._node_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


class NodeTests(unittest.TestCase):
    def setUp(self):
        module = load_node()
        self.node = n = module.TunnelNode.__new__(module.TunnelNode)
        n.fps, n.max_age = 30., .25
        n.stream, n.decoder = FeatureStream(), EventDecoder()
        n.origin_ns = n.last_stamp = n.last_received = None
        n.next_inference = 0.
        n.model_id = "20261023_140042_191723_aug"
        n.last_event = None
        n.times, n.dropped = deque(maxlen=60), 0
        n.probabilities, n.status = np.zeros(2), 'WAITING'
        n.get_logger = lambda: SimpleNamespace(info=lambda _: None, warning=lambda _: None)
        self.clock_ns = 0
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.clock_ns))
        self.outputs = []
        n.result = SimpleNamespace(publish=lambda x: self.outputs.append(json.loads(x.data)))
        self.calls = []
        def infer(x):
            self.calls.append(x)
            return np.array([[10., -10.]], np.float32)
        n.backend = SimpleNamespace(infer=infer)

    def frame(self, frame, age=0.):
        stamp = 1_000_000_000 + round(frame * 1e9 / 30)
        self.clock_ns = stamp + round(age * 1e9)
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp // 1_000_000_000,
                                  nanosec=stamp % 1_000_000_000)),
                                  encoding='mono8', width=64, height=40, step=64, data=bytes(64 * 40))
        self.node.on_image(message)
        self.node.process()

    def test_warmup_holds_events_then_emits_one_entry(self):
        for frame in range(80):
            self.frame(frame)
        events = [(i, r['event']) for i, r in enumerate(self.outputs) if r['event']]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][0], 25)
        self.assertEqual(events[0][1]['type'], 'entry')
        self.assertEqual(self.node.decoder.state, 'inside')
        self.assertEqual(self.calls[-1].shape, (1, 17, 32))

    def test_history_reset_preserves_state_unless_new_drive(self):
        n = self.node
        for frame in range(80):
            self.frame(frame)
        n.clear_history()
        self.assertEqual(n.decoder.state, 'inside')
        self.assertIsNone(n.stream.started)
        self.frame(80)
        self.assertIsNone(self.outputs[-1]['event'])
        self.assertIn('WARMUP', n.status)
        n.clear_history(new_drive=True)
        self.assertEqual(n.decoder.state, 'unknown')
        self.assertIsNone(n.last_event)

    def test_stale_and_duplicate_images_do_not_infer(self):
        self.frame(0)
        self.frame(0)
        self.assertEqual(len(self.calls), 1)
        self.frame(1, age=.3)
        self.assertEqual(len(self.calls), 1)
        self.assertIn('STALE', self.node.status)
        self.frame(3)
        self.assertEqual(len(self.calls), 2)
        self.assertFalse(self.node.stream.ready)

    def test_missing_frames_are_counted_and_long_gap_holds_events(self):
        for frame in range(80):
            self.frame(frame)
        self.frame(112)
        self.assertEqual(self.node.dropped, 32)
        self.assertIn('WARMUP', self.node.status)
        self.assertEqual(self.node.decoder.state, 'inside')
        self.assertIsNone(self.outputs[-1]['event'])


if __name__ == '__main__':
    unittest.main()
