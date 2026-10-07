"""ROS stereo input, TensorRT inference and a main-thread OpenCV preview."""
from collections import deque
import json
import os
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from .features import FeatureStream, EventDecoder, softmax
from .backend import TensorRTBackend
from .display import footer, mono_image

WINDOW = 'Tunnel TCN - partial INT8'


class TunnelNode(Node):
    def __init__(self):
        super().__init__('tunnel_tcn_test')
        cv2.setNumThreads(1)
        share = Path(get_package_share_directory('tunnel_tcn_test'))
        def parameter(name, default):
            return self.declare_parameter(name, default).value
        model_path = parameter('model_path', str(share / 'models/brightness31_partial_int8.onnx'))
        cache_dir = parameter('engine_cache_dir', '~/.cache/tunnel_tcn_test')
        self.fps = float(parameter('camera_fps', 30.0))
        self.max_age = float(parameter('max_image_age_seconds', .25))
        self.stream = FeatureStream(float(parameter('warmup_seconds', 2.5)), float(parameter('max_gap_seconds', .5)))
        self.decoder = EventDecoder(float(parameter('threshold', .8)), float(parameter('cooldown_seconds', .1)))
        if self.fps != 30 or not 0 <= self.decoder.threshold <= 1 or self.decoder.cooldown < 0:
            raise ValueError('This launch uses 30 FPS; invalid detector settings')
        self.get_logger().info('Loading partial INT8 TensorRT model; first engine build can take several minutes.')
        provenance = json.loads((share / 'models/provenance.json').read_text())
        self.backend = TensorRTBackend(model_path, cache_dir, provenance['onnx_sha256'])
        self.get_logger().info(f'TensorRT ready: {self.backend.engine_path}')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Image, parameter('image_topic', '/camera/image_stereo_ir'), self.on_image, qos)
        settings_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.controls = self.create_subscription(String, '/camera/ir_settings', self.on_settings, settings_qos)
        self.result = self.create_publisher(String, '/tunnel_tcn/result', 10)
        self.pending = None
        self.settings_revision = None
        self.settings = 'waiting for settings'
        self.origin_ns = None
        self.last_stamp = None
        self.last_received = None
        self.times = deque(maxlen=60)
        self.dropped = 0
        self.frame = np.zeros((400, 640), np.uint8)
        self.probabilities = np.zeros(3)
        self.last_event = None
        self.status = 'WAITING FOR CAMERA'
        self.infer_ms = self.total_ms = 0.
        self.measured_fps = 0.

    def on_image(self, message):
        self.pending = message

    def clear_history(self, new_drive=False):
        self.stream.reset()
        self.decoder.reset() if new_drive else self.decoder.pause()
        self.origin_ns = self.last_stamp = None
        self.times.clear()
        self.probabilities = np.zeros(3)
        self.status = 'WARMUP 2.5s'
        if new_drive:
            self.last_event = None

    def on_settings(self, message):
        try:
            data = json.loads(message.data)
            revision = data['revision']
            self.settings = f"{data['exposure_us']}us ISO{data['iso']} dot{data['projector']:.2f} flood{data['flood']:.2f}" + (' MANUAL' if data.get('manual') else ' AUTO')
            if self.settings_revision is not None and revision != self.settings_revision:
                self.clear_history()
                self.get_logger().info('Camera settings changed: rebuilding brightness history; tunnel state preserved.')
            self.settings_revision = revision
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warning(f'Invalid camera settings message: {error}')

    def process(self):
        if self.pending is None:
            if self.last_received is not None and time.monotonic() - self.last_received > .5:
                self.status = 'STALE / CAMERA STOPPED'
                self.probabilities = np.zeros(3)
            return
        message, self.pending = self.pending, None
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if age > self.max_age or age < -.1:
            self.status = 'STALE IMAGE / CHECK CAMERA CLOCK'
            self.stream.reset()
            self.decoder.pause()
            self.probabilities = np.zeros(3)
            return
        if self.last_stamp is not None and stamp == self.last_stamp:
            return
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.clear_history()
        started = time.perf_counter()
        self.frame = mono_image(message)
        self.last_received = time.monotonic()
        if self.last_stamp is not None:
            self.dropped += max(0, round((stamp - self.last_stamp) * self.fps / 1e9) - 1)
        self.last_stamp = stamp
        if self.origin_ns is None:
            self.origin_ns = stamp
        timestamp = (stamp - self.origin_ns) / 1e9
        self.times.append(timestamp)
        self.measured_fps = (len(self.times) - 1) / (self.times[-1] - self.times[0]) if len(self.times) > 1 else 0.
        inputs = self.stream.push(self.frame, timestamp)
        if self.stream.discontinuity:
            self.decoder.pause()
        start_infer = time.perf_counter()
        self.probabilities = softmax(self.backend.infer(inputs))
        self.infer_ms = (time.perf_counter() - start_infer) * 1000
        event = None
        if self.stream.ready:
            self.status = 'LIVE' if self.measured_fps >= 27 else 'LIVE / LOW SOURCE FPS'
            event = self.decoder.step(self.probabilities, timestamp)
        else:
            remaining = max(0, self.stream.warmup_seconds - (timestamp - self.stream.started))
            self.status = f'WARMUP {remaining:.1f}s (events held)'
        if event:
            self.last_event = (event, time.monotonic())
            self.get_logger().info(f"{event['type'].upper()} score={event['score']:.3f}")
        self.total_ms = (time.perf_counter() - started) * 1000
        report = {'source_stamp_ns': stamp, 'probabilities': self.probabilities.tolist(),
                  'state': self.decoder.state, 'status': self.status, 'event': event,
                  'source_fps': self.measured_fps, 'estimated_missed_frames': self.dropped,
                  'inference_wall_ms': self.infer_ms, 'processing_ms': self.total_ms,
                  'source_age_at_receive_ms': age * 1000}
        output = String()
        output.data = json.dumps(report, allow_nan=False)
        self.result.publish(output)

    def image(self):
        last = '--'
        if self.last_event:
            event, emitted = self.last_event
            last = f"{event['type'].upper()} {event['score']:.3f} ({time.monotonic() - emitted:.1f}s ago)"
        return footer(self.frame, self.probabilities, self.decoder.state, last, self.status,
                      self.measured_fps, self.infer_ms, self.total_ms, self.dropped, self.settings)


def main(args=None):
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError('A Jetson desktop display is required for the preview and camera controls')
    rclpy.init(args=args)
    node = None
    try:
        node = TunnelNode()
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 960, 760)
        last_draw = 0.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.001)
            node.process()
            if time.monotonic() - last_draw >= 1 / 30.:
                cv2.imshow(WINDOW, node.image())
                last_draw = time.monotonic()
            key = cv2.waitKey(1) & 0xff
            if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord('r'), ord('R')):
                node.clear_history(new_drive=True)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.backend.close()
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
