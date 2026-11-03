import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from line_detactor.msg import LaneResult
from .selection import LaneSelector, lane_ready


class TunnelLaneSelector(Node):
    def __init__(self):
        super().__init__('tunnel_lane_selector')
        param = lambda key, default: self.declare_parameter(key, default).value
        self.policy = LaneSelector(param('max_age_sec', .2), param('confirmations', 3))
        self.rules = {key: param(key, default) for key, default in dict(
            minimum_x=.01, maximum_x=2.98, minimum_points=8,
            maximum_gap=.15, minimum_span=.12).items()}
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        status = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output = self.create_publisher(LaneResult, '/tunnel/lane_result', sensor)
        self.status = {key: self.create_publisher(String, '/tunnel/' + key, status)
                       for key in ('state', 'target_source', 'selected_source', 'compute_source')}
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
        self.get_logger().info(f'Tunnel {values[0]} | selected={values[2]} compute={values[3]}')

    def on_event(self, message):
        try:
            report = json.loads(message.data)
            stamp = report['source_stamp_ns']
            if type(stamp) is not int or not 0 <= (self.get_clock().now().nanoseconds - stamp) / 1e9 <= .25:
                return
            self.last_tcn_received = time.monotonic()
            state = report.get('state')
            self.tcn_live = str(report.get('status', '')).startswith('LIVE') and state in ('inside', 'outside')
            event = report.get('event')
            if not self.tcn_live:
                return
            requested = 'IR' if state == 'inside' else 'RGB'
            # Event packets may be lost with sensor QoS. The TCN retains its
            # accepted state in subsequent reports, so reconcile that state too.
            # Matching reports must not restart a pending three-frame handoff.
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
        # Without a live boundary detector there is no reliable way to choose
        # the next source. Withhold results so existing controller guards stop.
        if not self.tcn_live or self.last_tcn_received is None or time.monotonic() - self.last_tcn_received > .5:
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        accepted = self.policy.lane(source, stamp, self.get_clock().now().nanoseconds,
                                    lane_ready(message, self.rules))
        self.publish_status()
        if accepted:
            self.output.publish(message)


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
