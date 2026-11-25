"""Exercise real node callbacks with transport/clock/GPU doubles, without ROS."""
import ast
import importlib.util
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
import unittest
from collections import deque
import numpy as np
from tunnel_tcn_test.loading_status import LoadingStatus
from tunnel_tcn_test.features import FeatureStream, EventDecoder


def load_node():
    modules = {}
    for name in ['rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg',
                 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv',
                 'ament_index_python', 'ament_index_python.packages']:
        modules[name] = ModuleType(name)
    modules['rclpy.node'].Node = object
    for name in ['QoSProfile', 'ReliabilityPolicy', 'DurabilityPolicy', 'HistoryPolicy']:
        setattr(modules['rclpy.qos'], name, object)
    modules['sensor_msgs.msg'].Image = object
    modules['std_msgs.msg'].String = SimpleNamespace
    modules['std_srvs.srv'].Trigger = object
    modules['ament_index_python.packages'].get_package_share_directory = lambda _: ''
    path = Path(__file__).resolve().parents[1] / 'tunnel_tcn_test/node.py'
    spec = importlib.util.spec_from_file_location('tunnel_tcn_test._node_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


class NodeTests(unittest.TestCase):
    def test_startup_trial_reports_failure_instead_of_ready(self):
        # Execute the real constructor's model-check block with a GPU double.
        path = Path(__file__).resolve().parents[1] / 'tunnel_tcn_test/node.py'
        tree = ast.parse(path.read_text())
        node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TunnelNode')
        constructor = next(n for n in node_class.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        start = next(i for i, n in enumerate(constructor.body)
                     if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Attribute)
                     and n.targets[0].attr == 'model_status')
        end = next(i for i, n in enumerate(constructor.body[start:], start)
                   if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                   and n.targets[0].id == 'qos')
        code = compile(ast.Module(body=constructor.body[start:end], type_ignores=[]), str(path), 'exec')
        for output, error in ((np.zeros((1, 2), np.float32), None),
                              (np.full((1, 2), np.nan), RuntimeError),
                              (np.zeros((2,), np.float32), RuntimeError),
                              (None, RuntimeError)):
            reports, errors = [], []
            def infer(features):
                self.assertEqual(features.shape, (1, 17, 32))
                if output is None:
                    raise RuntimeError('engine load/inference failure')
                return output
            node = SimpleNamespace(
                create_publisher=lambda *a: SimpleNamespace(publish=lambda m: reports.append(m.data)),
                get_logger=lambda: SimpleNamespace(info=lambda m: None, error=errors.append))
            namespace = dict(LoadingStatus=LoadingStatus, self=node, model_path='/model.onnx', cache_dir='/cache',
                provenance={'onnx_sha256': 'test'}, np=np, String=SimpleNamespace,
                QoSProfile=lambda **kw: kw,
                ReliabilityPolicy=SimpleNamespace(RELIABLE='reliable'),
                DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL='latched'),
                TensorRTBackend=lambda *a, **kw: SimpleNamespace(infer=infer, engine_path='/engine'))
            if error:
                with self.assertRaises(error):
                    exec(code, namespace)
                self.assertTrue(reports[-1].startswith('FAILED | '))
                self.assertFalse(any(item.startswith('READY | ') for item in reports))
            else:
                exec(code, namespace)
                self.assertTrue(reports[-1].startswith('READY | '))

    def test_headless_main_runs_inference_loop_without_gui_calls(self):
        module = load_node()
        running = [True]
        calls = []
        fake_node = SimpleNamespace(
            declare_parameter=lambda *_: SimpleNamespace(value=False),
            process=lambda: calls.append('process'),
            backend=SimpleNamespace(close=lambda: calls.append('close')),
            destroy_node=lambda: calls.append('destroy'))
        def spin_once(*_, **__):
            running[0] = False
        def unexpected_gui(*_, **__):
            self.fail('Headless TCN must not access a GUI')
        with patch.object(module, 'TunnelNode', return_value=fake_node), \
             patch.object(module, 'rclpy', SimpleNamespace(init=lambda **_: None,
                          ok=lambda: running[0], spin_once=spin_once)), \
             patch.object(module, 'cv2', SimpleNamespace(namedWindow=unexpected_gui,
                          imshow=unexpected_gui, waitKey=unexpected_gui,
                          destroyAllWindows=unexpected_gui)):
            module.main()
        self.assertEqual(calls, ['process', 'close', 'destroy'])

    def setUp(self):
        module = load_node()
        self.node = n = module.TunnelNode.__new__(module.TunnelNode)
        n.fps, n.max_age = 30., .25
        n.stream, n.decoder = FeatureStream(), EventDecoder()
        n.origin_ns = n.last_stamp = n.last_received = None
        n.accept_frames_after_ns = None
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

    def test_stats_input_matches_image_input(self):
        """camera_driver가 보낸 프레임 통계로도 영상 입력과 같은 특징·결과가 나온다."""
        from tunnel_tcn_test.features import frame_stats, FRAME_STATS
        rng = np.random.default_rng(5)
        images = [rng.integers(0, 256, (40, 64), np.uint8) for _ in range(40)]
        for frame, image in enumerate(images):
            stamp = 1_000_000_000 + round(frame * 1e9 / 30)
            self.clock_ns = stamp
            self.node.on_image(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
                sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000)),
                encoding='mono8', width=64, height=40, step=64, data=image.tobytes()))
            self.node.process()
        image_inputs = [c.copy() for c in self.calls]
        self.setUp()
        self.node.stats_input = True
        from tunnel_tcn_test.features import preprocess
        for frame, image in enumerate(images):
            stamp = 1_000_000_000 + round(frame * 1e9 / 30)
            self.clock_ns = stamp
            stats = frame_stats(preprocess(image))
            message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
                sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000)), **{k: np.float32(stats[k]) for k in FRAME_STATS})
            self.node.on_image(message)
            self.node.process()
        self.assertEqual(len(self.calls), len(image_inputs))
        for a, b in zip(self.calls, image_inputs):
            np.testing.assert_allclose(a, b, atol=1e-4)

    def test_warmup_holds_events_then_emits_one_entry(self):
        for frame in range(80):
            self.frame(frame)
        events = [(i, r['event']) for i, r in enumerate(self.outputs) if r['event']]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][0], 25)
        self.assertEqual(events[0][1]['type'], 'entry')
        self.assertEqual(self.node.decoder.state, 'inside')
        self.assertEqual(self.calls[-1].shape, (1, 17, 32))

    def test_live_tuning_keeps_tunnel_state_and_updates_reported_threshold(self):
        for frame in range(80):
            self.frame(frame)
        n = self.node
        before = n.decoder.last.copy()
        tuning = dict(threshold=.6, cooldown_seconds=.7, warmup_seconds=1.,
                      max_gap_seconds=.8, max_image_age_seconds=.3)
        n.on_tuning(SimpleNamespace(data=json.dumps(tuning)))
        self.assertEqual(n.decoder.state, 'inside')
        np.testing.assert_array_equal(n.decoder.last, before)
        self.assertEqual(n.decoder.cooldown, .7)
        self.assertEqual(n.stream.warmup_seconds, 1.)
        self.assertEqual(n.stream.max_gap_seconds, .8)
        self.assertEqual(n.max_age, .3)
        for frame in range(80, 86):
            self.frame(frame)
        self.assertEqual(self.outputs[-1]['threshold'], .6)
        self.assertIsNone(self.outputs[-1]['event'])

        n.on_tuning(SimpleNamespace(data=json.dumps(dict(tuning, threshold=.9, max_gap_seconds=0))))
        self.assertEqual(n.decoder.threshold, .6)
        self.assertEqual(n.stream.max_gap_seconds, .8)

    def test_manual_inside_and_outside_hold_then_require_a_new_entry_crossing(self):
        n = self.node
        n.stream.warmup_seconds = 0
        n.decoder.state = 'outside'
        n.on_state_override(SimpleNamespace(data='inside'))
        n.backend.infer = lambda _: np.array([[-10., 10.]], np.float32)
        for frame in range(6):
            self.frame(frame)
        self.assertEqual(self.outputs[0]['event']['type'], 'entry')
        self.assertTrue(self.outputs[0]['event']['forced'])
        self.assertEqual(self.outputs[-1]['state_override'], 'inside')
        self.assertIsNone(self.outputs[-1]['event'])

        n.on_state_override(SimpleNamespace(data='outside'))
        n.backend.infer = lambda _: np.array([[10., -10.]], np.float32)
        for frame in range(6, 12):
            self.frame(frame)
        self.assertEqual(self.outputs[-2]['event']['type'], 'exit')
        self.assertTrue(self.outputs[-2]['event']['forced'])
        self.assertEqual(self.outputs[-1]['state_override'], 'outside')
        self.assertIsNone(self.outputs[-1]['event'])

        n.on_state_override(SimpleNamespace(data='resume_outside'))
        for frame in range(12, 18):
            self.frame(frame)
        self.assertIsNone(self.outputs[-1]['state_override'])
        self.assertEqual(self.outputs[-1]['state'], 'outside')
        n.backend.infer = lambda _: np.array([[-10., -10.]], np.float32)
        for frame in range(18, 21):
            self.frame(frame)
        n.backend.infer = lambda _: np.array([[10., -10.]], np.float32)
        self.frame(21)
        self.assertEqual(self.outputs[-1]['event']['type'], 'entry')
        self.assertNotIn('forced', self.outputs[-1]['event'])

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

    def test_override_reset_starts_outside_and_discards_pre_reset_frames(self):
        n = self.node
        n.decoder = EventDecoder(initial_state='outside')
        for frame in range(80):
            self.frame(frame)
        self.assertEqual(n.decoder.state, 'inside')
        n.pending = object()
        response = n.reset_outside(None, SimpleNamespace())
        self.assertTrue(response.success)
        self.assertEqual(n.decoder.state, 'outside')
        self.assertIsNone(n.pending)
        self.assertIsNone(n.stream.started)
        self.assertIsNone(n.last_event)
        outputs_before_reset = len(self.outputs)
        self.frame(79)
        self.assertEqual(len(self.outputs), outputs_before_reset)
        self.frame(80)
        self.assertEqual(self.outputs[-1]['state'], 'outside')
        self.assertTrue(self.outputs[-1]['status'].startswith('WARMUP'))

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

    def test_selector_reset_releases_manual_hold_and_restarts_warmup(self):
        n = self.node
        n.decoder = EventDecoder(initial_state='outside')
        n.on_state_override(SimpleNamespace(data='inside'))
        n.pending = object()
        n.on_state_override(SimpleNamespace(data='reset_outside'))
        self.assertIsNone(n.manual_state)
        self.assertIsNone(n.pending_manual_event)
        self.assertIsNone(n.pending)
        self.assertEqual(n.decoder.state, 'outside')
        self.assertTrue(n.status.startswith('WARMUP'))

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
