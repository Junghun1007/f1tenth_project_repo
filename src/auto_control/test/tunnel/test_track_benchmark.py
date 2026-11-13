from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from auto_control_tunnel.track_benchmark_data import TrackCapture, benchmark_overrides, write_capture


class TrackCaptureTests(unittest.TestCase):
    def test_start_once_and_exact_sixty_second_window(self):
        capture = TrackCapture(1)
        capture.add(9, 'before', {})
        self.assertFalse(capture.due(100))
        self.assertTrue(capture.start(100))
        self.assertFalse(capture.start(105))
        capture.add(100, 'lane_rgb', {'compute_ms': 5})
        capture.add(159.999, 'lane_rgb', {'compute_ms': 15})
        capture.add(160, 'after', {})
        self.assertFalse(capture.due(159.999))
        self.assertTrue(capture.due(160))
        snapshot = capture.finish(160.1, 'duration_complete')
        self.assertEqual(snapshot['recorded_duration_sec'], 60)
        self.assertEqual(snapshot['event_counts'], {'lane_rgb': 2})
        self.assertIsNone(capture.finish(161, 'interrupted'))
        capture.add(159, 'late_callback', {})
        self.assertEqual(len(snapshot['events']), 2)

    def test_interruption_and_buffer_loss_are_explicit(self):
        capture = TrackCapture(4, max_events=2)
        capture.start(10)
        for index in range(5):
            capture.add(11 + index, 'runtime', {'mean_ms': index})
        snapshot = capture.finish(20, 'interrupted')
        self.assertEqual(snapshot['recorded_duration_sec'], 10)
        self.assertEqual(snapshot['dropped_events'], {'runtime': 3})
        self.assertEqual(snapshot['finish_reason'], 'interrupted')

    def test_saved_artifacts_preserve_sources_parameters_and_missing_values(self):
        capture = TrackCapture(4)
        capture.start(0)
        capture.add(1, 'lane_rgb', {'compute_ms': 5, 'reciprocal_fps': 200}, 100)
        capture.add(2, 'lane_ir', {'compute_ms': 0, 'reciprocal_fps': None}, 200)
        capture.add(3, 'system', {'cpu_core_pct': {'cpu0': 90}, 'gpu_pct': 50})
        with tempfile.TemporaryDirectory() as directory:
            write_capture(directory, capture.finish(60, 'duration_complete'),
                          {'/camera_driver': {'sensor_fps': 80}}, {'/other': 'unavailable'})
            root = Path(directory)
            manifest = json.loads((root / 'manifest.json').read_text())
            self.assertEqual(manifest['event_counts']['lane_ir'], 1)
            self.assertNotIn('events', manifest)
            self.assertIn('200', (root / 'lane_rgb.csv').read_text())
            self.assertTrue((root / 'lane_ir.csv').exists())
            rows = [json.loads(line) for line in (root / 'events.jsonl').read_text().splitlines()]
            self.assertIsNone(rows[1]['data']['reciprocal_fps'])
            self.assertEqual(json.loads((root / 'parameters.json').read_text())['errors'],
                             {'/other': 'unavailable'})

    def test_four_scenarios_include_requested_control_and_disable_previews(self):
        for run in range(1, 5):
            flags = benchmark_overrides(run)
            self.assertEqual(flags['obstacles_enabled'], str(run >= 2).lower())
            self.assertEqual(flags['avoidance_control_enabled'], str(run >= 2).lower())
            self.assertEqual(flags['traffic_light_enabled'], str(run >= 3).lower())
            self.assertEqual(flags['traffic_stop_enabled'], str(run >= 3).lower())
            self.assertEqual(flags['tunnel_enabled'], str(run == 4).lower())
            for name in ('preview_enabled', 'remote_preview_enabled', 'camera_preview_enabled',
                         'dataset_collection_enabled', 'dataset_collection_manual_capture_mode'):
                self.assertEqual(flags[name], 'false')


class RecorderLifecycleTests(unittest.TestCase):
    def test_real_recorder_callbacks_start_on_approval_and_finish_without_drive_commands(self):
        import ast
        from contextlib import nullcontext
        from types import SimpleNamespace
        import copy
        import math
        import threading
        from datetime import datetime
        from auto_control_tunnel.track_benchmark_data import SCENARIOS

        class FakeNode:
            def __init__(self, name):
                self.name = name
                self.publishers = {}
                self.topics = []
                self.messages = []

            def declare_parameter(self, name, default):
                return SimpleNamespace(value=directory if name == 'directory' else default)

            def create_publisher(self, kind, topic, qos):
                messages = []
                self.publishers[topic] = messages
                return SimpleNamespace(publish=messages.append)

            def create_subscription(self, kind, topic, callback, qos):
                self.topics.append(topic)
                return topic

            def destroy_subscription(self, subscription):
                self.topics.remove(subscription)

            def create_timer(self, period, callback):
                return callback

            def get_logger(self):
                return SimpleNamespace(info=self.messages.append, error=self.messages.append)

            def get_clock(self):
                return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0] * 1e9)))

            def get_name(self):
                return self.name

        class FakeCompute:
            lock = nullcontext()
            sampled_at = 100
            values = {'cpu_pct': 12}

            def close(self):
                pass

        source = Path(__file__).resolve().parents[2] / 'auto_control_tunnel/track_benchmark.py'
        tree = ast.parse(source.read_text())
        tree.body = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]
        clock = [100.0]
        namespace = dict(Node=FakeNode, ComputeMonitor=FakeCompute, TrackCapture=TrackCapture,
                         SCENARIOS=SCENARIOS, write_capture=write_capture, Path=Path, datetime=datetime,
                         time=SimpleNamespace(monotonic=lambda: clock[0]), copy=copy, json=json,
                         math=math, threading=threading, QoSProfile=lambda **kw: kw,
                         DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1), qos_profile_sensor_data=1,
                         RemoteParameters=lambda *args: SimpleNamespace(services_are_ready=lambda: False),
                         **{kind: SimpleNamespace for kind in
                            ('Bool', 'String', 'Float32', 'LaneResult', 'TrafficLightState', 'Log', 'ParameterEvent')})
        exec(compile(tree, str(source), 'exec'), namespace)
        with tempfile.TemporaryDirectory() as directory:
            node = namespace['TrackBenchmark']()
            self.assertIn('/perf/lane_result_to_control_ms', node.topics)
            node.on_context('/tunnel/state', SimpleNamespace(data='OUTSIDE'))
            node.on_startup(SimpleNamespace(data='READY_TO_START | ready'))
            node.tick()
            self.assertIsNone(node.capture.started)
            node.on_startup(SimpleNamespace(data='STARTED | automatic duty permitted'))
            node.tick()
            node.on_json('vehicle', SimpleNamespace(data='{"speed": 1.2}'))
            clock[0] = 110
            node.on_startup(SimpleNamespace(data='STARTED | automatic duty permitted'))
            self.assertEqual(node.capture.started, 100)
            clock[0] = 160
            node.tick()
            node.close()
            self.assertEqual(list(node.publishers), ['/auto/runtime_metrics/enabled'])
            self.assertFalse(node.publishers['/auto/runtime_metrics/enabled'][-1].data)
            self.assertEqual(node.topics, [])
            manifest = json.loads((node.directory / 'manifest.json').read_text())
            self.assertEqual(manifest['finish_reason'], 'duration_complete')
            self.assertEqual(manifest['initial_states'], {'/tunnel/state': 'OUTSIDE'})
            self.assertEqual(manifest['event_counts']['vehicle'], 1)


if __name__ == '__main__':
    unittest.main()
