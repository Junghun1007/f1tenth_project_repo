#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


class ActuatorCommanderNode(Node):
    def __init__(self) -> None:
        super().__init__("actuator_commander_node")

        self.declare_parameter("throttle_topic", "/manual/throttle")
        self.declare_parameter("steering_topic", "/manual/steering")
        self.declare_parameter("erpm_topic", "/vesc/erpm")
        self.declare_parameter("servo_position_topic", "/vesc/servo_position")

        self.declare_parameter("min_erpm", -3000)
        self.declare_parameter("max_erpm", 3000)
        self.declare_parameter("servo_left", 0.25)
        self.declare_parameter("servo_center", 0.50)
        self.declare_parameter("servo_right", 0.75)
        self.declare_parameter("throttle_deadzone", 0.03)
        self.declare_parameter("steering_deadzone", 0.05)

        throttle_topic = str(self.get_parameter("throttle_topic").value)
        steering_topic = str(self.get_parameter("steering_topic").value)
        erpm_topic = str(self.get_parameter("erpm_topic").value)
        servo_topic = str(self.get_parameter("servo_position_topic").value)

        self.min_erpm = int(self.get_parameter("min_erpm").value)
        self.max_erpm = int(self.get_parameter("max_erpm").value)
        self.servo_left = float(self.get_parameter("servo_left").value)
        self.servo_center = float(self.get_parameter("servo_center").value)
        self.servo_right = float(self.get_parameter("servo_right").value)
        self.throttle_deadzone = float(self.get_parameter("throttle_deadzone").value)
        self.steering_deadzone = float(self.get_parameter("steering_deadzone").value)

        self._last_throttle = 0.0
        self._last_steering = 0.0

        self.erpm_pub = self.create_publisher(Int32, erpm_topic, 10)
        self.servo_pub = self.create_publisher(Float32, servo_topic, 10)
        self.throttle_sub = self.create_subscription(
            Float32,
            throttle_topic,
            self._on_throttle,
            10,
        )
        self.steering_sub = self.create_subscription(
            Float32,
            steering_topic,
            self._on_steering,
            10,
        )

        self.get_logger().info(
            f"Actuator commander ready. throttle_topic={throttle_topic}, "
            f"steering_topic={steering_topic}, erpm_topic={erpm_topic}, "
            f"servo_position_topic={servo_topic}"
        )

    def _on_throttle(self, msg: Float32) -> None:
        throttle = self._clamp(float(msg.data), -1.0, 1.0)
        self._last_throttle = self._apply_deadzone(throttle, self.throttle_deadzone)
        self._publish_commands()

    def _on_steering(self, msg: Float32) -> None:
        steering = self._clamp(float(msg.data), -1.0, 1.0)
        self._last_steering = self._apply_deadzone(steering, self.steering_deadzone)
        self._publish_commands()

    def _publish_commands(self) -> None:
        target_erpm = self._throttle_to_erpm(self._last_throttle)
        servo_position = self._steering_to_servo(self._last_steering)

        self.erpm_pub.publish(Int32(data=target_erpm))
        self.servo_pub.publish(Float32(data=servo_position))

    def _throttle_to_erpm(self, throttle: float) -> int:
        # VESC set_rpm expects ERPM, not wheel RPM. ERPM is electrical RPM.
        throttle = self._clamp(throttle, -1.0, 1.0)
        target_erpm = int(throttle * self.max_erpm)
        return self._clamp_int(target_erpm, self.min_erpm, self.max_erpm)

    def _steering_to_servo(self, steering: float) -> float:
        steering = self._clamp(steering, -1.0, 1.0)
        if steering < 0.0:
            servo_position = self._lerp(self.servo_center, self.servo_left, -steering)
        else:
            servo_position = self._lerp(self.servo_center, self.servo_right, steering)

        servo_min = min(self.servo_left, self.servo_center, self.servo_right)
        servo_max = max(self.servo_left, self.servo_center, self.servo_right)
        return self._clamp(servo_position, servo_min, servo_max)

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        return value

    @staticmethod
    def _lerp(start: float, end: float, amount: float) -> float:
        return start + (end - start) * amount

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _clamp_int(value: int, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, value))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActuatorCommanderNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
