#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, Int32

from vesc_interface.vesc_driver import (
    VescCommandIds,
    VescDriver,
    VescDriverError,
    VescScales,
)


class VescInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("vesc_interface_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.1)
        self.declare_parameter("write_timeout", 2.0)
        self.declare_parameter("startup_delay", 0.2)
        self.declare_parameter("connect_on_startup", True)

        self.declare_parameter("erpm_topic", "/vesc/erpm")
        self.declare_parameter("servo_position_topic", "/vesc/servo_position")
        self.declare_parameter("min_erpm", -100000)
        self.declare_parameter("max_erpm", 100000)
        self.declare_parameter("servo_min", 0.0)
        self.declare_parameter("servo_max", 1.0)
        self.declare_parameter("command_timeout", 0.3)

        self.declare_parameter("packet.comm_set_erpm", 8)
        self.declare_parameter("packet.comm_set_servo_pos", 12)
        self.declare_parameter("packet.servo_scale", 1000)

        erpm_topic = str(self.get_parameter("erpm_topic").value)
        servo_topic = str(self.get_parameter("servo_position_topic").value)

        self.min_erpm = int(self.get_parameter("min_erpm").value)
        self.max_erpm = int(self.get_parameter("max_erpm").value)
        self.servo_min = float(self.get_parameter("servo_min").value)
        self.servo_max = float(self.get_parameter("servo_max").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)

        self._last_erpm_time: Time | None = None
        self._last_erpm_command = 0

        self.driver = VescDriver(
            port=str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            timeout=float(self.get_parameter("serial_timeout").value),
            write_timeout=float(self.get_parameter("write_timeout").value),
            startup_delay=float(self.get_parameter("startup_delay").value),
            command_ids=VescCommandIds(
                set_erpm=int(self.get_parameter("packet.comm_set_erpm").value),
                set_servo_pos=int(
                    self.get_parameter("packet.comm_set_servo_pos").value
                ),
            ),
            scales=VescScales(
                servo=int(self.get_parameter("packet.servo_scale").value),
            ),
        )

        if bool(self.get_parameter("connect_on_startup").value):
            self._try_open_driver()

        self.erpm_sub = self.create_subscription(Int32, erpm_topic, self._on_erpm, 10)
        self.servo_sub = self.create_subscription(
            Float32,
            servo_topic,
            self._on_servo_position,
            10,
        )

        if self.command_timeout > 0.0:
            self.timeout_timer = self.create_timer(0.05, self._check_command_timeout)
        else:
            self.timeout_timer = None

        self.get_logger().info(
            f"VESC interface ready. erpm_topic={erpm_topic}, "
            f"servo_position_topic={servo_topic}"
        )

    def _try_open_driver(self) -> None:
        try:
            self.driver.open()
            self.get_logger().info(
                "\n"
                "==================== VESC 연결 여부 ====================\n"
                f"VESC 연결완료: {self.driver.port}\n"
                "========================================================"
            )
        except VescDriverError as exc:
            self.get_logger().error(
                "\n"
                "==================== VESC 연결 여부 ====================\n"
                f"VESC 연결실패: {self.driver.port}\n"
                f"원인: {exc}\n"
                "========================================================"
            )

    def _on_erpm(self, msg: Int32) -> None:
        # VESC set_rpm expects ERPM, not wheel RPM. ERPM is electrical RPM.
        target_erpm = self._clamp_int(int(msg.data), self.min_erpm, self.max_erpm)
        self._last_erpm_time = self.get_clock().now()
        self._last_erpm_command = target_erpm
        self._send_erpm(target_erpm)

    def _on_servo_position(self, msg: Float32) -> None:
        position = self._clamp_float(float(msg.data), self.servo_min, self.servo_max)
        try:
            self.driver.set_servo_position(position)
        except VescDriverError as exc:
            self.get_logger().error(
                f"Servo position command failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def _check_command_timeout(self) -> None:
        if self._last_erpm_time is None or self._last_erpm_command == 0:
            return

        elapsed_sec = (
            self.get_clock().now() - self._last_erpm_time
        ).nanoseconds / 1_000_000_000
        if elapsed_sec < self.command_timeout:
            return

        self.get_logger().warn(
            f"ERPM command timeout ({elapsed_sec:.2f}s). Sending ERPM 0.",
            throttle_duration_sec=1.0,
        )
        self._last_erpm_time = self.get_clock().now()
        self._last_erpm_command = 0
        self._send_erpm(0)

    def _send_erpm(self, target_erpm: int) -> None:
        try:
            self.driver.set_erpm(target_erpm)
        except VescDriverError as exc:
            self.get_logger().error(
                f"ERPM command failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def destroy_node(self) -> bool:
        try:
            if self.driver.is_open:
                self.driver.set_erpm(0)
        except VescDriverError as exc:
            self.get_logger().warn(f"Failed to send ERPM 0 before shutdown: {exc}")
        finally:
            self.driver.close()

        return super().destroy_node()

    @staticmethod
    def _clamp_int(value: int, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _clamp_float(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VescInterfaceNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
