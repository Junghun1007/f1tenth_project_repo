#!/usr/bin/env python3

from __future__ import annotations

from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32


class TestPhase(Enum):
    WAITING_FOR_VESC = auto()
    SERVO_SWEEP = auto()
    FORWARD_PAUSE = auto()
    FORWARD_RAMP = auto()
    FORWARD_HOLD = auto()
    REVERSE_PAUSE = auto()
    REVERSE_RAMP = auto()
    REVERSE_HOLD = auto()
    FINISHED = auto()
    ABORTED = auto()


class VehicleTestDriveNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_test_drive")

        self.declare_parameter("servo_position_topic", "/vesc/servo_position")
        self.declare_parameter("erpm_topic", "/vesc/erpm")
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("connection_timeout_sec", 5.0)
        self.declare_parameter("servo_step_interval_sec", 0.5)
        self.declare_parameter("erpm_ramp_duration_sec", 3.0)
        self.declare_parameter("target_hold_sec", 0.25)
        self.declare_parameter("neutral_pause_sec", 0.5)
        self.declare_parameter("erpm_update_rate_hz", 20.0)
        self.declare_parameter("forward_erpm", 5000)
        self.declare_parameter("reverse_erpm", -5000)
        self.declare_parameter("final_servo_position", 0.5)

        servo_topic = str(self.get_parameter("servo_position_topic").value)
        erpm_topic = str(self.get_parameter("erpm_topic").value)
        connection_status_topic = str(
            self.get_parameter("connection_status_topic").value
        )
        self.connection_timeout_sec = max(
            0.1,
            float(self.get_parameter("connection_timeout_sec").value),
        )
        self.servo_step_interval_sec = max(
            0.05,
            float(self.get_parameter("servo_step_interval_sec").value),
        )
        self.erpm_ramp_duration_sec = max(
            0.1,
            float(self.get_parameter("erpm_ramp_duration_sec").value),
        )
        self.target_hold_sec = max(
            0.05,
            float(self.get_parameter("target_hold_sec").value),
        )
        self.neutral_pause_sec = max(
            0.0,
            float(self.get_parameter("neutral_pause_sec").value),
        )
        update_rate_hz = max(
            1.0,
            float(self.get_parameter("erpm_update_rate_hz").value),
        )
        self.forward_erpm = int(self.get_parameter("forward_erpm").value)
        self.reverse_erpm = int(self.get_parameter("reverse_erpm").value)
        self.final_servo_position = float(
            self.get_parameter("final_servo_position").value
        )

        self.servo_pub = self.create_publisher(Float32, servo_topic, 10)
        self.erpm_pub = self.create_publisher(Int32, erpm_topic, 10)
        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.connection_sub = self.create_subscription(
            Bool,
            connection_status_topic,
            self._on_connection_status,
            connection_qos,
        )

        self._servo_positions = [
            *(value / 10.0 for value in range(5, 0, -1)),
            *(value / 10.0 for value in range(1, 10)),
        ]
        self._servo_index = 0
        self._phase = TestPhase.WAITING_FOR_VESC
        self._phase_started_at = self._now_sec()
        self._next_servo_command_at = self._phase_started_at
        self._next_wait_log_at = self._phase_started_at
        self._connection_status_received = False
        self._vesc_connected = False
        self._test_started = False
        self.done = False
        self.failed = False

        self._timer = self.create_timer(1.0 / update_rate_hz, self._on_timer)

        self.get_logger().info(
            "Waiting for VESC connection before starting the actuator test. "
            f"connection_status_topic={connection_status_topic}, "
            f"servo_topic={servo_topic}, erpm_topic={erpm_topic}"
        )
        self.get_logger().warn(
            "Keep the vehicle lifted and clear of people while the ERPM test runs."
        )

    def _on_timer(self) -> None:
        now = self._now_sec()

        if self._phase is TestPhase.WAITING_FOR_VESC:
            self._wait_for_vesc(now)
        elif self._phase is TestPhase.SERVO_SWEEP:
            self._run_servo_sweep(now)
        elif self._phase is TestPhase.FORWARD_PAUSE:
            self._publish_erpm(0)
            if self._phase_elapsed(now) >= self.neutral_pause_sec:
                self._start_phase(TestPhase.FORWARD_RAMP, now)
                self.get_logger().info(
                    f"Starting forward ERPM ramp: 0 -> {self.forward_erpm}"
                )
        elif self._phase is TestPhase.FORWARD_RAMP:
            if self._run_erpm_ramp(0, self.forward_erpm, now):
                self._start_phase(TestPhase.FORWARD_HOLD, now)
                self.get_logger().info(
                    f"Forward target reached. Holding {self.forward_erpm} ERPM."
                )
        elif self._phase is TestPhase.FORWARD_HOLD:
            self._publish_erpm(self.forward_erpm)
            if self._phase_elapsed(now) >= self.target_hold_sec:
                self._publish_erpm(0)
                self._start_phase(TestPhase.REVERSE_PAUSE, now)
                self.get_logger().info("Holding ERPM 0 before reverse ramp.")
        elif self._phase is TestPhase.REVERSE_PAUSE:
            self._publish_erpm(0)
            if self._phase_elapsed(now) >= self.neutral_pause_sec:
                self._start_phase(TestPhase.REVERSE_RAMP, now)
                self.get_logger().info(
                    f"Starting reverse ERPM ramp: 0 -> {self.reverse_erpm}"
                )
        elif self._phase is TestPhase.REVERSE_RAMP:
            if self._run_erpm_ramp(0, self.reverse_erpm, now):
                self._start_phase(TestPhase.REVERSE_HOLD, now)
                self.get_logger().info(
                    f"Reverse target reached. Holding {self.reverse_erpm} ERPM."
                )
        elif self._phase is TestPhase.REVERSE_HOLD:
            self._publish_erpm(self.reverse_erpm)
            if self._phase_elapsed(now) >= self.target_hold_sec:
                self._finish_test()

    def _on_connection_status(self, msg: Bool) -> None:
        self._connection_status_received = True
        self._vesc_connected = bool(msg.data)

        if self._test_started and not self._vesc_connected and not self.done:
            self._abort_test("VESC serial connection was lost during the test.")

    def _wait_for_vesc(self, now: float) -> None:
        erpm_subscribers = self.erpm_pub.get_subscription_count()
        servo_subscribers = self.servo_pub.get_subscription_count()

        if (
            self._vesc_connected
            and erpm_subscribers > 0
            and servo_subscribers > 0
        ):
            self._test_started = True
            self._servo_index = 0
            self._next_servo_command_at = now
            self._start_phase(TestPhase.SERVO_SWEEP, now)
            self.get_logger().info(
                "VESC connected and command subscribers detected. Starting test."
            )
            return

        if self._phase_elapsed(now) >= self.connection_timeout_sec:
            status = (
                "disconnected"
                if self._connection_status_received
                else "no connection status received"
            )
            self._abort_test(
                "VESC connection check timed out: "
                f"status={status}, erpm_subscribers={erpm_subscribers}, "
                f"servo_subscribers={servo_subscribers}"
            )
            return

        if now >= self._next_wait_log_at:
            self.get_logger().info(
                "Waiting for VESC: "
                f"connected={self._vesc_connected}, "
                f"erpm_subscribers={erpm_subscribers}, "
                f"servo_subscribers={servo_subscribers}"
            )
            self._next_wait_log_at = now + 1.0

    def _run_servo_sweep(self, now: float) -> None:
        self._publish_erpm(0)

        if now < self._next_servo_command_at:
            return

        if self._servo_index >= len(self._servo_positions):
            self._publish_erpm(0)
            self._start_phase(TestPhase.FORWARD_PAUSE, now)
            self.get_logger().info("Servo sweep complete. Holding ERPM 0.")
            return

        position = self._servo_positions[self._servo_index]
        self._publish_servo(position)
        self.get_logger().info(f"Servo position: {position:.1f}")
        self._servo_index += 1
        self._next_servo_command_at = now + self.servo_step_interval_sec

    def _run_erpm_ramp(self, start: int, end: int, now: float) -> bool:
        progress = min(
            1.0,
            self._phase_elapsed(now) / self.erpm_ramp_duration_sec,
        )
        target_erpm = int(round(start + (end - start) * progress))
        self._publish_erpm(target_erpm)
        return progress >= 1.0

    def _finish_test(self) -> None:
        self._publish_erpm(0)
        self._publish_servo(self.final_servo_position)
        self._phase = TestPhase.FINISHED
        self.done = True
        self._timer.cancel()
        self.get_logger().info(
            "Vehicle actuator test complete. ERPM=0, servo position="
            f"{self.final_servo_position:.1f}"
        )

    def _abort_test(self, reason: str) -> None:
        self.stop_actuators()
        self._phase = TestPhase.ABORTED
        self.failed = True
        self.done = True
        self._timer.cancel()
        self.get_logger().error(f"Vehicle actuator test aborted: {reason}")

    def stop_actuators(self) -> None:
        self._publish_erpm(0)
        self._publish_servo(self.final_servo_position)

    def _start_phase(self, phase: TestPhase, now: float) -> None:
        self._phase = phase
        self._phase_started_at = now

    def _phase_elapsed(self, now: float) -> float:
        return now - self._phase_started_at

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _publish_servo(self, position: float) -> None:
        self.servo_pub.publish(Float32(data=float(position)))

    def _publish_erpm(self, erpm: int) -> None:
        self.erpm_pub.publish(Int32(data=int(erpm)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VehicleTestDriveNode()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user.")
    finally:
        failed = node.failed
        if rclpy.ok():
            node.stop_actuators()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
