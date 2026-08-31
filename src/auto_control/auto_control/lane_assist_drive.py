#!/usr/bin/env python3
"""Guarded low-speed lane-assist controller for the front-camera detector."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray


class LaneAssistDrive(Node):
    def __init__(self) -> None:
        super().__init__("lane_assist_drive")
        for name, value in {
            "model_topic": "/front_lane/model",
            "emergency_stop_topic": "/auto/emergency_stop",
            "connection_status_topic": "/vesc/connected",
            "duty_topic": "/vesc/duty",
            "servo_position_topic": "/vesc/servo_position",
            # Intentionally conservative first autonomous-drive values.
            "cruise_duty": 0.050,
            # Virtual centres remain driveable; speed is reduced from the
            # predicted close/far curve rather than merely from this flag.
            "virtual_cruise_duty": 0.050,
            "max_duty": 0.065,
            "minimum_drive_duty": 0.040,
            # Overcome static friction only when beginning a newly armed run.
            "startup_duty": 0.060,
            "startup_boost_sec": 0.80,
            "servo_left": 0.98,
            "servo_center": 0.46,
            "servo_right": 0.02,
            "heading_gain": 0.80,
            # +1: smaller VESC servo value means right turn.  Change to -1
            # after a wheels-raised test if the actual steering is reversed.
            "steering_sign": 1.0,
            # Tight corners need more authority than the earlier 0.50 cap.
            # Still below 1.0 to avoid commanding the servo endpoint.
            "max_steering": 0.65,
            "steering_duty_reduction": 0.0,
            "curve_slowdown_start_deg": 18.0,
            "curve_slowdown_full_deg": 55.0,
            "curve_duty_scale_at_full": 0.82,
            "minimum_center_points": 5,
            # A new run may start only from a measured two-boundary centre,
            # never from a virtual one-boundary centreline.
            "minimum_start_real_pairs": 5,
            "required_valid_frames": 10,
            "model_timeout_sec": 0.20,
            # Brief visual dropouts should not instantly straighten the car
            # in the middle of a corner.
            "short_loss_hold_sec": 0.80,
            # During a short visual dropout, preserve both last steering
            # command and speed instead of slowing in a corner.
            "short_loss_duty_scale": 1.0,
            "control_rate_hz": 40.0,
            "command_log_rate_hz": 2.0,
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        max_duty = abs(float(value("max_duty")))
        self.cruise_duty = min(abs(float(value("cruise_duty"))), max_duty)
        self.virtual_cruise_duty = min(abs(float(value("virtual_cruise_duty"))), max_duty)
        self.minimum_drive_duty = min(
            max_duty, max(0.0, abs(float(value("minimum_drive_duty"))))
        )
        self.startup_duty = min(max_duty, abs(float(value("startup_duty"))))
        self.startup_boost_sec = max(0.0, float(value("startup_boost_sec")))
        self.servo_left = float(value("servo_left"))
        self.servo_center = float(value("servo_center"))
        self.servo_right = float(value("servo_right"))
        self.heading_gain = float(value("heading_gain"))
        self.steering_sign = 1.0 if float(value("steering_sign")) >= 0.0 else -1.0
        self.max_steering = abs(float(value("max_steering")))
        self.steering_duty_reduction = min(
            0.95, max(0.0, float(value("steering_duty_reduction")))
        )
        self.curve_slowdown_start_rad = np.deg2rad(
            max(0.0, float(value("curve_slowdown_start_deg")))
        )
        self.curve_slowdown_full_rad = max(
            self.curve_slowdown_start_rad + np.deg2rad(1.0),
            np.deg2rad(float(value("curve_slowdown_full_deg"))),
        )
        self.curve_duty_scale_at_full = min(
            1.0, max(0.0, float(value("curve_duty_scale_at_full")))
        )
        self.minimum_center_points = max(2, int(value("minimum_center_points")))
        self.minimum_start_real_pairs = max(
            self.minimum_center_points, int(value("minimum_start_real_pairs"))
        )
        self.required_valid_frames = max(1, int(value("required_valid_frames")))
        self.model_timeout_sec = max(0.05, float(value("model_timeout_sec")))
        self.short_loss_hold_sec = max(0.0, float(value("short_loss_hold_sec")))
        self.short_loss_duty_scale = min(
            1.0, max(0.0, float(value("short_loss_duty_scale")))
        )
        self.command_log_period = 1.0 / max(0.2, float(value("command_log_rate_hz")))

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.duty_pub = self.create_publisher(Float32, str(value("duty_topic")), command_qos)
        self.servo_pub = self.create_publisher(Float32, str(value("servo_position_topic")), command_qos)
        self.model_sub = self.create_subscription(
            Float32MultiArray, str(value("model_topic")), self._on_model, command_qos
        )
        self.stop_sub = self.create_subscription(
            Bool, str(value("emergency_stop_topic")), self._on_emergency_stop, command_qos
        )
        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.connection_sub = self.create_subscription(
            Bool, str(value("connection_status_topic")), self._on_connection, connection_qos
        )

        self.connected = False
        self.emergency_stopped = False
        self.valid_frames = 0
        self.last_model_time = -1.0
        self.last_valid_model_time = -1.0
        self.close_heading = 0.0
        self.curve_ahead = 0.0
        self.virtual_only = False
        self.model_valid = False
        self.driving = False
        self.drive_started_at = -1.0
        self.last_servo_command = self.servo_center
        self.last_duty_command = 0.0
        self.holding_short_loss = False
        self.last_command_log_time = -1.0
        self.timer = self.create_timer(1.0 / max(5.0, float(value("control_rate_hz"))), self._on_timer)
        self.get_logger().warn(
            "Lane assist armed. It starts only after a stable real two-lane centre; focus preview and press SPACE or E to stop."
        )

    def _on_connection(self, msg: Bool) -> None:
        self.connected = bool(msg.data)
        if not self.connected:
            self._stop("VESC disconnected")

    def _on_emergency_stop(self, msg: Bool) -> None:
        if msg.data:
            self.emergency_stopped = True
            self._stop("emergency stop")

    def _on_model(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 6:
            return
        self.last_model_time = self._now()
        valid, close_heading, _, virtual_only, center_count, real_pair_count = msg.data[:6]
        base_valid = valid >= 0.5 and center_count >= self.minimum_center_points
        # At a fresh start require actual left/right pairs.  Once moving,
        # the one-sided virtual centreline is allowed to carry the vehicle
        # through a curve without stopping.
        startup_valid = (
            base_valid
            and virtual_only < 0.5
            and real_pair_count >= self.minimum_start_real_pairs
        )
        self.model_valid = base_valid if self.driving else startup_valid
        if self.model_valid:
            self.valid_frames += 1
            self.last_valid_model_time = self.last_model_time
            self.close_heading = float(close_heading)
            self.curve_ahead = float(msg.data[6]) if len(msg.data) >= 7 else 0.0
            self.virtual_only = virtual_only >= 0.5
        else:
            self.valid_frames = 0

    def _on_timer(self) -> None:
        now = self._now()
        fresh = self.last_model_time >= 0.0 and now - self.last_model_time <= self.model_timeout_sec
        can_drive = (
            self.connected and not self.emergency_stopped and fresh and self.model_valid
            and self.valid_frames >= self.required_valid_frames
        )
        if not can_drive:
            within_short_loss = (
                self.driving
                and self.connected
                and not self.emergency_stopped
                and self.last_valid_model_time >= 0.0
                and now - self.last_valid_model_time <= self.short_loss_hold_sec
            )
            if within_short_loss:
                held_duty = self.last_duty_command * self.short_loss_duty_scale
                self.servo_pub.publish(Float32(data=self.last_servo_command))
                self.duty_pub.publish(Float32(data=held_duty))
                if not self.holding_short_loss:
                    self.get_logger().warn(
                        "Lane target briefly lost: holding last steering for %.2f s at reduced duty."
                        % self.short_loss_hold_sec
                    )
                    self.holding_short_loss = True
                return
            self._stop(None)
            return
        steering = self.steering_sign * max(-self.max_steering, min(
            self.max_steering,
            self.heading_gain * self.close_heading,
        ))
        if steering < 0.0:
            servo = self.servo_center + (self.servo_left - self.servo_center) * (-steering)
        else:
            servo = self.servo_center + (self.servo_right - self.servo_center) * steering
        base_duty = self.virtual_cruise_duty if self.virtual_only else self.cruise_duty
        steering_ratio = abs(steering) / max(self.max_steering, 1e-6)
        curve_ratio = min(1.0, max(
            0.0,
            (abs(self.curve_ahead) - self.curve_slowdown_start_rad)
            / (self.curve_slowdown_full_rad - self.curve_slowdown_start_rad),
        ))
        curve_scale = 1.0 - curve_ratio * (1.0 - self.curve_duty_scale_at_full)
        # Do not let steering-based speed reduction fall below the torque
        # needed to keep moving.  Invalid perception still follows the
        # separate short-loss / stop safety path below.
        normal_duty = max(
            self.minimum_drive_duty,
            base_duty * curve_scale * (1.0 - self.steering_duty_reduction * steering_ratio),
        )
        starting_now = not self.driving
        if starting_now:
            self.drive_started_at = now
        boost_active = now - self.drive_started_at <= self.startup_boost_sec
        duty = self.startup_duty if boost_active else normal_duty
        self.servo_pub.publish(Float32(data=float(servo)))
        self.duty_pub.publish(Float32(data=float(max(0.0, duty))))
        self.last_servo_command = float(servo)
        self.last_duty_command = float(max(0.0, duty))
        self.holding_short_loss = False
        now = self._now()
        if now - self.last_command_log_time >= self.command_log_period:
            self.get_logger().info(
                "lane command | heading=%+.1f deg | curve=%+.1f deg | steering=%+.3f | servo=%.3f | duty=%.3f"
                % (np.degrees(self.close_heading), np.degrees(self.curve_ahead), steering, servo, duty)
            )
            self.last_command_log_time = now
        if starting_now:
            self.driving = True
            self.get_logger().warn(
                "Real lane centre verified. Starting with duty %.3f for %.1f s."
                % (self.startup_duty, self.startup_boost_sec)
            )

    def _stop(self, reason: str | None) -> None:
        self.duty_pub.publish(Float32(data=0.0))
        self.servo_pub.publish(Float32(data=self.servo_center))
        self.last_duty_command = 0.0
        self.last_servo_command = self.servo_center
        self.holding_short_loss = False
        self.drive_started_at = -1.0
        if self.driving or reason:
            if reason:
                self.get_logger().warn(f"STOP: {reason}; duty=0.")
            self.driving = False

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def stop_actuators(self) -> None:
        self._stop("node shutdown")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LaneAssistDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_actuators()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
