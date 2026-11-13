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
    def test_real_recorder_starts_on_approval_and_requests_stop_at_deadline(self):
        import ast
        from concurrent.futures import Future
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
                self.stop_requests = []

            def declare_parameter(self, name, default):
                return SimpleNamespace(value=directory if name == 'directory' else default)

            def create_client(self, kind, topic):
                assert topic == '/auto_control/stop'
                def call(request):
                    self.stop_requests.append(request)
                    future = Future()
                    future.set_result(SimpleNamespace(success=True))
                    return future
                return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

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
        namespace = dict(Trigger=SimpleNamespace(Request=lambda: object()), Node=FakeNode, ComputeMonitor=FakeCompute, TrackCapture=TrackCapture,
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
            self.assertEqual(node.stop_requests, [])
            clock[0] = 160
            node.tick()
            self.assertEqual(len(node.stop_requests), 1)
            self.assertTrue(node.stop_accepted)
            clock[0] = 161
            node.tick()
            self.assertEqual(len(node.stop_requests), 1)
            node.close()
            self.assertEqual(list(node.publishers), ['/auto/runtime_metrics/enabled'])
            self.assertFalse(node.publishers['/auto/runtime_metrics/enabled'][-1].data)
            self.assertEqual(node.topics, [])
            manifest = json.loads((node.directory / 'manifest.json').read_text())
            self.assertEqual(manifest['finish_reason'], 'duration_complete')
            self.assertEqual(manifest['initial_states'], {'/tunnel/state': 'OUTSIDE'})
            self.assertEqual(manifest['event_counts']['vehicle'], 1)
            def failed_save(*args):
                raise OSError('disk full')
            namespace['write_capture'] = failed_save
            clock[0] = 200
            failed_node = namespace['TrackBenchmark']()
            failed_node.on_startup(SimpleNamespace(data='STARTED | permitted'))
            clock[0] = 260
            failed_node.tick()
            failed_node.writer.join()
            self.assertTrue(failed_node.stop_accepted)
            self.assertEqual(len(failed_node.stop_requests), 1)
            self.assertEqual(failed_node.write_error, 'disk full')
            failed_node.tick()
            self.assertTrue(any('Capture save failed' in message for message in failed_node.messages))



class StopRetryTests(unittest.TestCase):
    def test_unavailable_timeout_rejection_and_success(self):
        import ast
        from concurrent.futures import Future
        from types import SimpleNamespace, MethodType
        path = Path(__file__).resolve().parents[2] / 'auto_control_tunnel/track_benchmark.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'request_stop')
        namespace = {'Trigger': SimpleNamespace(Request=lambda: object())}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
        requests, messages = [], []
        ready = [False]
        def call(request):
            future = Future()
            requests.append(future)
            return future
        node = SimpleNamespace(stop_required=True, stop_accepted=False, stop_future=None,
            last_stop_attempt=float('-inf'),
            stop_client=SimpleNamespace(service_is_ready=lambda: ready[0], call_async=call),
            get_logger=lambda: SimpleNamespace(info=messages.append, error=messages.append))
        tick = MethodType(namespace['request_stop'], node)
        tick(0)
        self.assertEqual(requests, [])
        ready[0] = True
        tick(1)
        tick(2)
        self.assertEqual(len(requests), 1)
        tick(3)
        self.assertTrue(requests[0].cancelled())
        tick(4)
        requests[1].set_result(SimpleNamespace(success=False, message='temporary failure'))
        tick(4.1)
        self.assertFalse(node.stop_accepted)
        tick(5.2)
        requests[2].set_result(SimpleNamespace(success=True))
        tick(5.3)
        self.assertTrue(node.stop_accepted)
        tick(10)
        self.assertEqual(len(requests), 3)
        self.assertTrue(any('unavailable' in message for message in messages))
        self.assertTrue(any('timed out' in message for message in messages))


if __name__ == '__main__':
    unittest.main()
