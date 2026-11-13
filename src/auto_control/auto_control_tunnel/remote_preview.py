"""remote_preview: numeric telemetry over HTTP/SSE for an iPad/MacBook browser preview.

Read-only toward driving: it only subscribes, and forwards START/STOP to the existing
/auto_control/start and /auto_control/stop Trigger services. It never publishes duty/enable.
"""
import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, String, UInt8
from std_srvs.srv import Trigger

from .compute_status import ComputeMonitor
from .inspection_state import InspectionState
from .remote_frame import RateWindow, build_frame, centerline_metric, tunnel_state
from .remote_http import RemoteServer
from .remote_link import LinkWatchdog

VEHICLE_TOPICS = {
    'speed_mps': ('/auto/current_speed', Float32), 'target_speed_mps': ('/auto/target_speed', Float32),
    'duty': ('/auto/current_duty', Float32), 'brake_current_a': ('/auto/current_brake_current', Float32),
    'servo': ('/auto/current_servo_position', Float32), 'steering_rad': ('/auto/steering_angle_rad', Float32),
    'raw_steering_rad': ('/auto/raw_steering_angle_rad', Float32),
    'cte_m': ('/auto/cross_track_error_m', Float32), 'heading_rad': ('/auto/heading_error_rad', Float32),
    'curvature': ('/auto/path_curvature', Float32), 'corner_state': ('/auto/corner_state', UInt8),
}
TRAFFIC_NAMES = {0: 'UNKNOWN', 1: 'RED', 2: 'GREEN'}
PIPELINE_STALE_SEC = 12.0  # bev_processor reports every status_log_interval_sec (5s default)


def html_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('auto_control'), 'web', 'remote_preview.html')
        if os.path.exists(path):
            return path
    except Exception:
        pass
    return os.path.join(os.path.dirname(__file__), 'web', 'remote_preview.html')


class RemotePreview(Node):
    def __init__(self):
        super().__init__('remote_preview')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.port = int(p('port', 8765))
        self.bind = str(p('bind', '0.0.0.0'))
        self.lane_width_m = float(p('lane_width_m', 0.65))
        self.frame_rate_hz = float(p('frame_rate_hz', 30.0))
        loss_stop_sec = float(p('link_loss_stop_sec', 1.0))
        lane_topics = list(p('lane_topics', ['/line_detactor/result']))
        expected = list(p('expected_nodes', ['line_detactor']))
        connection_topic = str(p('connection_status_topic', '/vesc/connected'))
        diagnostics_topic = str(p('connection_diagnostics_topic', '/vesc/connected/diagnostics'))
        traffic_topic = str(p('traffic_state_topic', '/traffic_light/state'))
        if not 1.0 <= self.frame_rate_hz <= 60.0 or self.lane_width_m <= 0:
            raise ValueError('frame_rate_hz must be 1..60 and lane_width_m positive')

        self.lock = threading.Lock()
        self.vehicle, self.lane = {}, {}
        self.traffic = self.tunnel_state = self.tunnel_lines = self.pipeline = None
        self.rates = {key: RateWindow() for key in ('lane', 'inference', 'postprocess', 'control', 'traffic')}
        self.last_traffic_generation = None
        self.tcn_fps = None
        self.message = ''
        self.state = InspectionState(expected)
        self.watchdog = LinkWatchdog(loss_stop_sec)
        self.compute = ComputeMonitor()
        self.seq = 0

        sensor = qos_profile_sensor_data
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        from line_detactor.msg import LaneResult
        subs = [self.create_subscription(kind, topic, lambda m, k=key: self.on_vehicle(k, m.data), sensor)
                for key, (topic, kind) in VEHICLE_TOPICS.items()]
        subs.append(self.create_subscription(
            Bool, connection_topic, lambda m: self.on_vehicle('vesc_connected', bool(m.data)), sensor))
        subs += [self.create_subscription(LaneResult, topic, self.on_lane, sensor) for topic in lane_topics]
        subs.append(self.create_subscription(
            Float32, '/auto/detector_input_to_control_decision_ms', self.on_control_ms, sensor))
        subs.append(self.create_subscription(String, '/bev_processor/pipeline_fps', self.on_pipeline, latched))
        subs.append(self.create_subscription(String, '/tunnel/preview_status', self.on_tunnel, latched))
        try:
            from traffic_detection_test.msg import TrafficLightState
            subs.append(self.create_subscription(TrafficLightState, traffic_topic, self.on_traffic, sensor))
        except ImportError:
            self.get_logger().warn('traffic_detection_test messages unavailable; traffic shown as --')
        subs += [
            self.create_subscription(String, '/bev_processor/startup_status',
                                     lambda m: setattr(self.state, 'camera', m.data), latched),
            self.create_subscription(String, '/auto_control/startup_state',
                                     lambda m: self.state.update_controller(m.data), latched),
            self.create_subscription(String, diagnostics_topic, lambda m: self.state.update_link(m.data), latched),
        ]
        subs += [self.create_subscription(String, f'/{name}/model_status',
                                          lambda m, n=name: self.state.update_model(n, m.data), latched)
                 for name in expected]
        self.subscriptions_owned = subs
        self.start_client = self.create_client(Trigger, '/auto_control/start')
        self.stop_client = self.create_client(Trigger, '/auto_control/stop')

        self.server = RemoteServer(self.bind, self.port, html_path(), self)
        self.server.start()
        self.timer = self.create_timer(1.0 / self.frame_rate_hz, self.tick)
        self.get_logger().info(
            f'Remote preview on http://<jetson-ip>:{self.server.port} ({self.frame_rate_hz:.0f} Hz, '
            f'lane width {self.lane_width_m:.2f} m, link-loss STOP '
            f'{"off" if loss_stop_sec <= 0 else f"{loss_stop_sec:.1f}s"}, lanes {lane_topics})')

    # ---- ROS callbacks: store numbers only ----
    def on_vehicle(self, key, value):
        with self.lock:
            self.vehicle[key] = (value, time.monotonic())

    def on_lane(self, message):
        now = time.monotonic()
        points = []
        if message.centerline_valid:
            points = centerline_metric([(q.x, q.y) for q in message.centerline_points],
                                       message.source_width, message.source_height, message.padding_left,
                                       message.centerline_bev_width_m, message.centerline_bev_height_m)
        stop = message.stop_line_distance_m
        with self.lock:
            self.lane = {'points': (points, now), 'valid': (bool(points), now), 'state': (int(message.state), now),
                         'source': (message.lane_source or 'RGB', now),
                         'stop_line_m': (stop if stop == stop else None, now)}
            self.rates['lane'].add(now)
            self.rates['inference'].add(now, message.pure_inference_nanoseconds / 1e6)
            self.rates['postprocess'].add(now, (message.label_export_nanoseconds +
                                                message.backend_postprocess_nanoseconds +
                                                message.lane_geometry_nanoseconds) / 1e6)

    def on_control_ms(self, message):
        with self.lock:
            self.rates['control'].add(time.monotonic(), float(message.data))

    def on_pipeline(self, message):
        try:
            values = json.loads(message.data)
        except ValueError:
            return
        with self.lock:
            self.pipeline = (values, time.monotonic())

    def on_tunnel(self, message):
        now = time.monotonic()
        lines = message.data.split('\n')[:3]
        tcn = None
        if len(lines) > 1 and 'avg ' in lines[1]:
            try:
                tcn = float(lines[1].split('avg ', 1)[1].split('FPS', 1)[0])
            except ValueError:
                tcn = None
        with self.lock:
            self.tunnel_state = (tunnel_state(message.data), now)
            self.tunnel_lines = (lines, now)
            self.tcn_fps = (tcn, now)

    def on_traffic(self, message):
        now = time.monotonic()
        with self.lock:
            self.traffic = (TRAFFIC_NAMES.get(int(message.state), 'UNKNOWN'), now)
            if message.source_generation != self.last_traffic_generation:
                self.last_traffic_generation = message.source_generation
                self.rates['traffic'].add(now)

    # ---- RemoteBackend (called from HTTP threads) ----
    def heartbeat(self):
        self.watchdog.heartbeat(time.monotonic())

    def _call(self, client, name):
        if not client.service_is_ready():
            return False, f'{name} service unavailable'
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(2.0):
            return False, f'{name} timed out (no controller response in 2s)'
        response = future.result()
        return bool(response.success), response.message

    def request_start(self):
        if not self.state.can_start():
            return False, 'Not ready: ' + self.state.controller
        return self._call(self.start_client, 'START')

    def request_stop(self):
        return self._call(self.stop_client, 'STOP')

    # ---- 30 Hz frame ----
    def tick(self):
        now = time.monotonic()
        controller_fresh = self.state.controller_fresh(now)
        started = controller_fresh and self.state.controller.startswith('STARTED')
        if self.watchdog.should_stop(now, started):
            if self.stop_client.service_is_ready():
                self.stop_client.call_async(Trigger.Request())
            self.message = 'LINK LOST: STOP sent — START를 다시 눌러야 출발'
            self.get_logger().warn('Remote preview heartbeat lost while STARTED; STOP requested.')
        elif started:
            self.message = ''
        with self.compute.lock:
            system = dict(self.compute.values) if self.compute.sampled_at and now - self.compute.sampled_at < 3 else {}
        with self.lock:
            pipeline = self.pipeline[0] if self.pipeline and now - self.pipeline[1] < PIPELINE_STALE_SEC else {}
            tcn = self.tcn_fps[0] if self.tcn_fps and now - self.tcn_fps[1] < 1.0 else None
            snap = {
                'vehicle': dict(self.vehicle), 'lane': dict(self.lane), 'lane_width_m': self.lane_width_m,
                'fps': {'camera': pipeline.get('stabilized_fps'), 'bev': pipeline.get('bev_ready_fps'),
                        'lane': self.rates['lane'].fps(now), 'inference_ms': self.rates['inference'].mean(now),
                        'postprocess_ms': self.rates['postprocess'].mean(now),
                        'control_ms': self.rates['control'].mean(now), 'traffic': self.rates['traffic'].fps(now),
                        'tcn': tcn},
                'traffic': self.traffic,
                'tunnel': {'state': self.tunnel_state, 'lines': self.tunnel_lines},
            }
        snap['system'] = system
        snap['startup'] = {'camera': self.state.camera, 'models': dict(self.state.models),
                           'controller': self.state.controller if controller_fresh else 'WAITING | controller silent',
                           'can_start': self.state.can_start(now), 'started': started, 'message': self.message}
        snap['link'] = {'clients': self.server.client_count(),
                        'heartbeat_age_ms': self.watchdog.heartbeat_age_ms(now),
                        'loss_stop_sec': self.watchdog.timeout_sec}
        self.seq += 1
        self.server.broadcast(build_frame(self.seq, now, snap))

    def destroy_node(self):
        self.server.close()
        self.compute.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RemotePreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
