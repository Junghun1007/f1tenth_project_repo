import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from line_detactor.msg import LaneResult
from .selection import LaneSelector
from .blending import PathRules
from .telemetry import TunnelPreviewTelemetry


class TunnelLaneSelector(Node):
    def __init__(self):
        super().__init__('tunnel_lane_selector')
        param = lambda key, default: self.declare_parameter(key, default).value
        rules = PathRules(
            frame_id=param('lane_result_frame_id', 'front_axle_bev'),
            minimum_points=param('path_minimum_points', 8), minimum_span=param('path_minimum_span_m', .12),
            minimum_x=param('path_minimum_x_m', .05), maximum_x=param('path_maximum_x_m', 2.20),
            maximum_gap=param('path_maximum_gap_m', .15), x_max=param('bev_x_max_m', 3.),
            y_max=param('bev_y_max_m', .6), mpp=param('bev_meter_per_pixel', .01))
        self.policy = LaneSelector(param('max_age_sec', .2), param('validation_mode', False),
            blend_sec=param('blend_duration_sec', .15),
            transition_timeout_sec=param('transition_timeout_sec', .75),
            contributor_max_age_sec=param('contributor_max_age_sec', .12),
            pair_max_skew_sec=param('pair_max_skew_sec', .10),
            maximum_offset_m=param('maximum_blend_offset_m', .20),
            maximum_heading_rad=math.radians(param('maximum_blend_heading_deg', 20.)), rules=rules)
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        status = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(LaneResult, '/tunnel/lane_result', sensor)
        self.status = {key: self.create_publisher(String, '/tunnel/' + key, status)
                       for key in ('state', 'target_source', 'selected_source', 'compute_source')}
        self.preview = TunnelPreviewTelemetry()
        self.preview_pub = self.create_publisher(String, '/tunnel/preview_status', status)
        self.preview_timer = self.create_timer(.2, self.publish_preview)
        self.transition_timer = self.create_timer(.02, self.on_transition_tick)
        self.subscriptions_owned = [
            self.create_subscription(String, '/tunnel_tcn/result', self.on_event, sensor),
            self.create_subscription(LaneResult, param('rgb_result_topic', '/line_detactor/result'),
                                     lambda msg: self.on_lane('RGB', msg), sensor),
            self.create_subscription(LaneResult, '/ir_bev_line_detection/result',
                                     lambda msg: self.on_lane('IR', msg), sensor)]
        self.published = None
        self.last_tcn_received = None
        self.tcn_live = False
        self.publish_status()

    def publish_status(self):
        values = (self.policy.state, self.policy.camera, self.policy.selected, self.policy.compute)
        if values == self.published:
            return
        for key, value in zip(self.status, values):
            self.status[key].publish(String(data=value))
        self.published = values
        self.get_logger().info(f'Tunnel {values[0]} | selected={values[2]} compute={values[3]} '
                              f'camera={values[1]} reason={self.policy.reason}')
        if self.policy.last_handover and not self.policy.transitioning:
            self.get_logger().info('Tunnel handover ' + json.dumps(self.policy.last_handover))
        self.publish_preview()

    def on_transition_tick(self):
        self.policy.tick(self.get_clock().now().nanoseconds)
        self.publish_status()

    def publish_preview(self):
        self.preview_pub.publish(String(data='\n'.join(
            self.preview.lines(self.policy.state, time.monotonic()))))

    def on_event(self, message):
        try:
            report = json.loads(message.data)
            stamp = report['source_stamp_ns']
            if type(stamp) is not int or (not self.policy.validation_mode and
                    not 0 <= (self.get_clock().now().nanoseconds - stamp) / 1e9 <= .25):
                return
            self.preview.update(report, time.monotonic())
            self.last_tcn_received = time.monotonic()
            state = report.get('state')
            self.tcn_live = str(report.get('status', '')).startswith('LIVE') and state in ('inside', 'outside')
            event = report.get('event')
            if not self.tcn_live:
                return
            requested = 'IR' if state == 'inside' else 'RGB'
            # Event packets may be lost with sensor QoS. The TCN retains its
            # accepted state in subsequent reports, so reconcile that state too.
            # Matching reports do not repeat the same source change.
            if requested == self.policy.requested:
                return
            kind = 'entry' if requested == 'IR' else 'exit'
            if isinstance(event, dict) and event.get('type') != kind:
                return
            if self.policy.event(kind, stamp, self.get_clock().now().nanoseconds):
                self.publish_status()
        except (ValueError, TypeError, KeyError, AttributeError):
            self.get_logger().warning('Ignoring malformed TCN result')

    def on_lane(self, source, message):
        # Validation follows the last selected source independently of TCN
        # health, like running that detector/controller alone. TCN only switches.
        if not self.policy.validation_mode and (not self.tcn_live or self.last_tcn_received is None or
                time.monotonic() - self.last_tcn_received > .5):
            return
        first_before = self.policy.first_target_ns
        result = self.policy.lane(source, message, self.get_clock().now().nanoseconds)
        if first_before is None and self.policy.first_target_ns is not None:
            elapsed = (self.policy.first_target_ns - self.policy.started_ns) / 1e6
            self.get_logger().info(f'Tunnel first valid {self.policy.requested} path after {elapsed:.1f}ms; blending')
        self.publish_status()
        if result is not None:
            self.output.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = TunnelLaneSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
