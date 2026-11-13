"""Headless 60-second track recorder. START remains owned by auto_control."""
import copy
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import Log, ParameterEvent
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
from line_detactor.msg import LaneResult
from traffic_detection_test.msg import TrafficLightState

from .compute_status import ComputeMonitor
from .parameter_preset import parameter_values
from .remote_parameters import RemoteParameters
from .track_benchmark_data import SCENARIOS, TrackCapture, write_capture


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def finite(value):
    return value if math.isfinite(value) else None


class TrackBenchmark(Node):
    def __init__(self):
        super().__init__('track_benchmark')
        run = self.declare_parameter('run', 1).value
        self.capture = TrackCapture(run)
        root = Path(self.declare_parameter('directory', '~/.ros/auto_control/benchmarks').value).expanduser()
        self.directory = root / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + f'_run{run}')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.compute = ComputeMonitor()
        self.parameters, self.parameter_errors, self.pending = {}, {}, {}
        self.startup_messages = []
        self.context_states = {}
        self.initial_states = {}
        self.writer = None
        self.write_error = None
        self.saved = False
        self.stop_required = False
        self.stop_accepted = False
        self.stop_future = None
        self.last_stop_attempt = float('-inf')
        self.stop_client = self.create_client(Trigger, '/auto_control/stop')
        self.last_state = None
        self.last_system = None
        self.last_heartbeat = 0.0
        self.last_parameters = 0.0
        self.latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable_pub = self.create_publisher(Bool, '/auto/runtime_metrics/enabled', self.latched)
        self.subs = []
        self.subscribe(String, '/auto_control/startup_state', self.on_startup, self.latched)
        for topic, kind, qos in (
                ('/auto/runtime_metrics', 'runtime', 100),
                ('/perf/camera_stage', 'camera', 10),
                ('/perf/bev_stage', 'bev', 10),
                ('/auto/preview_snapshot', 'vehicle', qos_profile_sensor_data),
                ('/tunnel_tcn/result', 'tcn', qos_profile_sensor_data)):
            self.subscribe(String, topic, lambda m, k=kind: self.on_json(k, m), qos)
        for topic in ('/tunnel/state', '/tunnel/compute_source', '/tunnel/selected_source',
                      '/bev_processor/startup_status'):
            self.subscribe(String, topic, lambda m, t=topic: self.on_context(t, m), self.latched)
        for param, default in (
                ('detector_latency_topic', '/auto/detector_input_to_control_decision_ms'),
                ('result_latency_topic', '/perf/lane_result_to_control_ms')):
            topic = self.declare_parameter(param, default).value
            self.subscribe(Float32, topic, lambda m, t=topic: self.record(t, {'ms': finite(m.data)}),
                           qos_profile_sensor_data)
        rgb_topic = self.declare_parameter('rgb_result_topic', '/line_detactor/lane_result').value
        self.subscribe(LaneResult, rgb_topic, lambda m: self.on_lane('rgb', m), qos_profile_sensor_data)
        if SCENARIOS[run]['tunnel']:
            ir_topic = self.declare_parameter('ir_result_topic', '/tunnel/ir/lane_result').value
            self.subscribe(LaneResult, ir_topic, lambda m: self.on_lane('ir', m), qos_profile_sensor_data)
        if SCENARIOS[run]['traffic']:
            topic = self.declare_parameter('traffic_topic', '/traffic_light/state').value
            self.subscribe(TrafficLightState, topic, self.on_traffic, qos_profile_sensor_data)
        self.subscribe(Log, '/rosout', self.on_log, 100)
        self.subscribe(ParameterEvent, '/parameter_events', self.on_parameters_changed, 100)
        names = ['/auto_control', '/camera_driver', '/bev_processor', '/line_detactor']
        if SCENARIOS[run]['obstacles']:
            names.append('/auto_obstacles')
        if SCENARIOS[run]['traffic']:
            names.append('/traffic_light_detector')
        if SCENARIOS[run]['tunnel']:
            names += ['/ir_line_detactor', '/shared_ir_bev', '/tunnel_lane_selector', '/tunnel_tcn_test']
        self.clients = {name: RemoteParameters(self, name) for name in names}
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f'Run {run} armed. Output: {self.directory}. Waiting for accepted START.')

    def subscribe(self, kind, topic, callback, qos):
        self.subs.append(self.create_subscription(kind, topic, callback, qos))

    def record(self, kind, data):
        self.capture.add(time.monotonic(), kind, data, self.get_clock().now().nanoseconds)

    def on_startup(self, message):
        state = message.data.split(' | ', 1)[0]
        if state != self.last_state:
            self.get_logger().info(message.data)
            self.last_state = state
        if state == 'STARTED' and self.capture.start(time.monotonic()):
            self.initial_states = dict(self.context_states)
            self.enable_pub.publish(Bool(data=True))
            self.get_logger().info('Recording 60 seconds. Automatic STOP is requested when the capture ends.')
        self.record('startup_state', {'value': message.data})

    def on_context(self, topic, message):
        self.context_states[topic] = message.data
        self.record(topic, {'value': message.data})

    def on_json(self, kind, message):
        try:
            data = json.loads(message.data)
            # Reject invalid JSON numbers before they reach the asynchronous writer.
            json.dumps(data, allow_nan=False)
            if isinstance(data, dict):
                self.record(kind, data)
        except (ValueError, TypeError):
            self.record('invalid_message', {'kind': kind})

    def on_lane(self, source, message):
        data = {name: getattr(message, name) for name in (
            'detector_sequence', 'engine_precision', 'state', 'centerline_valid',
            'detector_queue_nanoseconds', 'h2d_preprocess_nanoseconds',
            'pure_inference_nanoseconds', 'label_export_nanoseconds',
            'backend_postprocess_nanoseconds', 'lane_geometry_nanoseconds',
            'result_message_build_nanoseconds', 'detector_total_compute_nanoseconds')}
        data.update(capture_ns=stamp_ns(message.header.stamp),
                    detector_input_ns=stamp_ns(message.detector_input_received_stamp),
                    result_ready_ns=stamp_ns(message.detector_result_ready_stamp),
                    centerline_points=len(message.centerline_points),
                    stop_line_present=message.stop_line_present,
                    stop_line_distance_m=finite(message.stop_line_distance_m))
        ms = message.detector_total_compute_nanoseconds / 1e6
        data['compute_ms'] = ms
        data['reciprocal_fps'] = 1000 / ms if ms > 0 else None
        self.record('lane_' + source, data)

    def on_traffic(self, message):
        self.record('traffic', dict(capture_ns=stamp_ns(message.header.stamp),
            generation=message.source_generation, state=message.state,
            inference_ms=finite(message.inference_ms), processing_ms=finite(message.processing_ms),
            capture_age_ms=finite(message.capture_age_ms)))

    def on_log(self, message):
        if message.name == self.get_name():
            return
        data = dict(node=message.name, level=message.level, message=message.msg,
                    stamp_ns=stamp_ns(message.stamp))
        if self.capture.started is None:
            self.startup_messages.append(data)
            self.startup_messages = self.startup_messages[-300:]
        self.record('rosout', data)

    def on_parameters_changed(self, message):
        if message.node not in self.clients:
            return
        values = message.new_parameters + message.changed_parameters
        try:
            data = parameter_values([p.name for p in values], [p.value for p in values])
            self.record('parameter_change', dict(node=message.node, values=data,
                        deleted=[p.name for p in message.deleted_parameters]))
        except ValueError as error:
            self.record('parameter_error', dict(node=message.node, error=str(error)))

    def read_parameters(self, now):
        for name, client in self.clients.items():
            if name in self.parameters:
                continue
            if name in self.pending:
                if now - self.pending[name] <= 10:
                    continue
                self.parameter_errors[name] = 'Parameter service timed out'
                # Do not accumulate service requests on an unresponsive node.
                continue
            if not client.services_are_ready():
                self.parameter_errors[name] = 'Parameter services unavailable'
                continue
            self.pending[name] = now
            client.list_parameters(depth=0).add_done_callback(
                lambda future, n=name: self.parameters_listed(n, future))

    def parameters_listed(self, name, future):
        try:
            names = list(future.result().result.names)
            self.clients[name].get_parameters(names).add_done_callback(
                lambda result: self.parameters_received(name, names, result))
        except Exception as error:
            self.parameter_errors[name] = str(error)
            self.pending.pop(name, None)

    def parameters_received(self, name, names, future):
        try:
            self.parameters[name] = parameter_values(names, future.result().values)
            self.parameter_errors.pop(name, None)
        except Exception as error:
            self.parameter_errors[name] = str(error)
        self.pending.pop(name, None)

    def request_stop(self, now):
        if not self.stop_required or self.stop_accepted:
            return
        if self.stop_future is not None:
            if self.stop_future.done():
                try:
                    response = self.stop_future.result()
                    if response.success:
                        self.stop_accepted = True
                        self.get_logger().info('Automatic STOP accepted by auto_control.')
                    else:
                        self.get_logger().error(f'Automatic STOP rejected: {response.message}; retrying.')
                except Exception as error:
                    self.get_logger().error(f'Automatic STOP failed: {error}; retrying.')
                self.stop_future = None
            elif now - self.last_stop_attempt >= 2.0:
                self.stop_future.cancel()
                self.stop_future = None
                self.get_logger().error('Automatic STOP response timed out; retrying.')
            else:
                return
            self.last_stop_attempt = now
        if self.stop_accepted or now - self.last_stop_attempt < 1.0:
            return
        self.last_stop_attempt = now
        if not self.stop_client.service_is_ready():
            self.get_logger().error('Automatic STOP service unavailable; retrying. Use the controller STOP if needed.')
            return
        try:
            self.stop_future = self.stop_client.call_async(Trigger.Request())
        except Exception as error:
            self.get_logger().error(f'Automatic STOP request failed: {error}; retrying.')

    def tick(self):
        now = time.monotonic()
        if self.capture.due(now):
            self.stop_required = True
            self.request_stop(now)
            self.finish('duration_complete')
        if self.capture.finished:
            self.request_stop(now)
            if self.writer and not self.writer.is_alive() and not self.saved:
                self.saved = True
                if self.write_error:
                    self.get_logger().error(f'Capture save failed: {self.write_error}')
                else:
                    self.get_logger().info(f'Capture saved: {self.directory}')
            return
        if now - self.last_parameters >= 1:
            self.last_parameters = now
            self.read_parameters(now)
        if self.capture.started is None:
            return
        if now - self.last_heartbeat >= 0.5:
            self.last_heartbeat = now
            self.enable_pub.publish(Bool(data=True))
        with self.compute.lock:
            sampled_at = self.compute.sampled_at
            values = dict(self.compute.values)
        if sampled_at is not None and sampled_at != self.last_system:
            self.last_system = sampled_at
            values['sample_age_sec'] = now - sampled_at
            self.record('system', values)

    def finish(self, reason):
        snapshot = self.capture.finish(time.monotonic(), reason)
        if snapshot is None:
            return
        snapshot['startup_messages'] = self.startup_messages
        snapshot['initial_states'] = self.initial_states
        for name in self.clients:
            if name not in self.parameters:
                self.parameter_errors.setdefault(name, 'No parameter response before capture ended')
        parameters = copy.deepcopy(self.parameters)
        errors = dict(self.parameter_errors)
        try:
            self.enable_pub.publish(Bool(data=False))
        except Exception:
            pass  # A shutdown may already have invalidated the ROS context.
        for subscription in self.subs:
            self.destroy_subscription(subscription)
        self.subs.clear()
        self.compute.close()

        def save():
            try:
                write_capture(self.directory, snapshot, parameters, errors)
            except Exception as error:
                self.write_error = str(error)
        self.writer = threading.Thread(target=save, name='track-capture-writer')
        self.writer.start()

    def close(self):
        self.finish('interrupted')
        self.compute.close()
        if self.writer:
            self.writer.join()
        if self.write_error:
            print(f'Capture save failed: {self.write_error}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrackBenchmark()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
