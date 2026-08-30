#!/usr/bin/env python3

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32

from auto_control.automatic_brake_profile import (
    AutomaticBrakeProfile,
    AutomaticBrakeProfileConfig,
)
from auto_control.control_core import (
    PathModel,
    SpeedPid,
    build_path_model,
    centerline_points_from_mono8,
    clamp,
    curvature_target_speed,
    erpm_to_speed_mps,
    move_toward,
    representative_curvature,
    speed_feedforward_duty,
    stanley_control,
    steering_angle_to_servo,
)
from auto_control.local_path_planner import (
    LocalPathPlannerConfig,
    OrderedPathModel,
    SpeedProfileConfig,
    build_ordered_path_model,
    plan_local_racing_path,
    spatial_speed_plan,
    stanley_control_ordered,
)


class AutoControlNode(Node):
    def __init__(self) -> None:
        super().__init__("auto_control")
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        command_qos.durability = DurabilityPolicy.VOLATILE
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._duty_pub = self.create_publisher(
            Float32, self.duty_topic, command_qos
        )
        self._brake_current_pub = self.create_publisher(
            Float32, self.brake_current_topic, command_qos
        )
        self._servo_pub = self.create_publisher(
            Float32, self.servo_position_topic, command_qos
        )
        self._command_duty_pub = self.create_publisher(
            Float32, self.command_duty_topic, command_qos
        )
        self._command_brake_current_pub = self.create_publisher(
            Float32, self.command_brake_current_topic, command_qos
        )
        self._target_speed_pub = self.create_publisher(
            Float32, self.target_speed_topic, command_qos
        )
        self._current_speed_pub = self.create_publisher(
            Float32, self.current_speed_topic, command_qos
        )
        self._curvature_pub = self.create_publisher(
            Float32, self.curvature_topic, command_qos
        )
        self._steering_angle_pub = self.create_publisher(
            Float32, self.steering_angle_topic, command_qos
        )
        self._cross_track_error_pub = self.create_publisher(
            Float32, self.cross_track_error_topic, command_qos
        )
        self._heading_error_pub = self.create_publisher(
            Float32, self.heading_error_topic, command_qos
        )
        self._raw_steering_angle_pub = self.create_publisher(
            Float32, self.raw_steering_angle_topic, command_qos
        )
        self._command_servo_position_pub = self.create_publisher(
            Float32, self.command_servo_position_topic, command_qos
        )
        self._planned_path_pub = self.create_publisher(
            Path, self.planned_path_topic, sensor_qos
        )

        self._lane_sub = self.create_subscription(
            Image, self.lane_topic, self._on_lane_image, sensor_qos
        )
        self._lane_path_sub = self.create_subscription(
            Path, self.lane_path_topic, self._on_lane_path, sensor_qos
        )
        self._left_boundary_path_sub = self.create_subscription(
            Path,
            self.lane_left_boundary_path_topic,
            self._on_left_boundary_path,
            sensor_qos,
        )
        self._right_boundary_path_sub = self.create_subscription(
            Path,
            self.lane_right_boundary_path_topic,
            self._on_right_boundary_path,
            sensor_qos,
        )
        self._erpm_sub = self.create_subscription(
            Int32, self.measured_erpm_topic, self._on_measured_erpm, sensor_qos
        )
        self._connection_sub = self.create_subscription(
            Bool,
            self.connection_status_topic,
            self._on_connection_status,
            connection_qos,
        )
        self._enable_sub = self.create_subscription(
            Bool, self.enable_topic, self._on_enable, command_qos
        )

        self._path: PathModel | OrderedPathModel | None = None
        self._ordered_centerline: tuple[int, np.ndarray, object] | None = None
        self._ordered_left_boundary: tuple[int, np.ndarray] | None = None
        self._ordered_right_boundary: tuple[int, np.ndarray] | None = None
        self._latest_planner_corner_count = 0
        self._last_path_received_time: Time | None = None
        self._path_capture_time: Time | None = None
        self._last_erpm_time: Time | None = None
        self._current_speed_mps = 0.0
        self._vesc_connected = False
        self._command_duty = 0.0
        self._command_brake_current = 0.0
        self._brake_mode_active = False
        self._last_motor_mode = "stop"
        self._steering_angle_rad = 0.0
        self._last_control_time = self.get_clock().now()
        self._last_stop_reason = "startup"
        self._latest_target_speed_mps = 0.0
        self._latest_curvature_per_m = 0.0
        self._latest_cross_track_error_m = 0.0
        self._latest_heading_error_rad = 0.0
        self._latest_raw_steering_angle_rad = 0.0
        self._latest_servo_position = self.servo_center
        self._latest_direction_guard_used = False

        self._speed_pid = SpeedPid(
            kp=self.speed_pid_kp,
            ki=self.speed_pid_ki,
            kd=self.speed_pid_kd,
            integral_limit=self.speed_pid_integral_limit,
            minimum_duty=self.minimum_duty,
            maximum_duty=self.maximum_duty,
        )
        self._brake_profile = AutomaticBrakeProfile(
            AutomaticBrakeProfileConfig(
                entry_speed_error_mps=self.brake_entry_speed_error_mps,
                exit_speed_error_mps=self.brake_exit_speed_error_mps,
                minimum_vehicle_speed_mps=(
                    self.brake_minimum_vehicle_speed_mps
                ),
                minimum_brake_current_amps=(
                    self.brake_minimum_current_amps
                ),
                maximum_brake_current_amps=(
                    self.brake_maximum_current_amps
                ),
                current_gain_amps_per_mps=(
                    self.brake_current_gain_amps_per_mps
                ),
                rise_amps_per_sec=self.brake_current_rise_amps_per_sec,
                fall_amps_per_sec=self.brake_current_fall_amps_per_sec,
            )
        )
        kinematic_curvature_limit = math.tan(
            self.maximum_steering_angle_rad
        ) / self.local_path_vehicle_wheelbase_m
        self._local_path_planner_config = LocalPathPlannerConfig(
            enabled=self.local_path_planner_enabled,
            expected_lane_width_m=self.local_path_expected_lane_width_m,
            vehicle_width_m=self.local_path_vehicle_width_m,
            safety_margin_m=self.local_path_safety_margin_m,
            minimum_half_width_m=self.local_path_minimum_half_width_m,
            maximum_half_width_m=self.local_path_maximum_half_width_m,
            width_smoothing_window_m=(
                self.local_path_width_smoothing_window_m
            ),
            maximum_center_correction_m=(
                self.local_path_maximum_center_correction_m
            ),
            resample_interval_m=self.local_path_resample_interval_m,
            corner_curvature_threshold_per_m=(
                self.local_path_corner_curvature_threshold_per_m
            ),
            corner_curvature_smoothing_window_m=(
                self.local_path_corner_curvature_smoothing_window_m
            ),
            corner_minimum_heading_change_deg=(
                self.local_path_corner_minimum_heading_change_deg
            ),
            corner_approach_length_m=self.local_path_corner_approach_length_m,
            corner_exit_length_m=self.local_path_corner_exit_length_m,
            outside_offset_fraction=self.local_path_outside_offset_fraction,
            apex_offset_fraction=self.local_path_apex_offset_fraction,
            maximum_offset_m=self.local_path_maximum_offset_m,
            maximum_offset_slope=self.local_path_maximum_offset_slope,
            racing_line_weight=self.local_path_racing_line_weight,
            maximum_path_curvature_per_m=min(
                self.local_path_maximum_curvature_per_m,
                kinematic_curvature_limit,
            ),
        )
        self._speed_profile_config = SpeedProfileConfig(
            enabled=self.spatial_speed_profile_enabled,
            minimum_speed_mps=self.minimum_speed_mps,
            maximum_speed_mps=self.maximum_speed_mps,
            maximum_lateral_acceleration_mps2=(
                self.maximum_lateral_acceleration_mps2
            ),
            maximum_longitudinal_acceleration_mps2=(
                self.speed_profile_maximum_longitudinal_acceleration_mps2
            ),
            maximum_longitudinal_deceleration_mps2=(
                self.speed_profile_maximum_longitudinal_deceleration_mps2
            ),
            minimum_combined_braking_ratio=(
                self.speed_profile_minimum_combined_braking_ratio
            ),
            lookahead_minimum_s_m=self.speed_profile_lookahead_minimum_s_m,
            lookahead_maximum_s_m=self.speed_profile_lookahead_maximum_s_m,
            sample_interval_m=self.speed_profile_sample_interval_m,
            acceleration_control_distance_m=(
                self.speed_profile_acceleration_control_distance_m
            ),
            curvature_percentile=self.curvature_percentile,
        )
        self._control_timer = self.create_timer(
            1.0 / self.control_rate_hz, self._on_control_timer
        )
        self._status_timer = self.create_timer(
            1.0 / self.status_log_rate_hz, self._log_status
        )

        self.get_logger().warn(
            "Automatic control is armed at launch. The vehicle will move when "
            "VESC telemetry and a valid BEV centerline are both available."
        )
        self.get_logger().info(
            "Auto control ready: lane=%s, speed=%.2f..%.2fm/s, "
            "duty=%.3f..%.3f, auto_brake=%s/%.1fA, rate=%.1fHz"
            % (
                self.lane_topic,
                self.minimum_speed_mps,
                self.maximum_speed_mps,
                self.minimum_duty,
                self.maximum_duty,
                "on" if self.electrical_brake_enabled else "off",
                self.brake_maximum_current_amps,
                self.control_rate_hz,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", True)
        self.declare_parameter("enable_topic", "/auto/enabled")
        self.declare_parameter("path_input_mode", "ordered_path")
        self.declare_parameter("lane_topic", "/camera/image_bev_lane")
        self.declare_parameter("lane_path_topic", "/camera/path_bev_lane")
        self.declare_parameter(
            "lane_left_boundary_path_topic", "/camera/path_bev_lane_left"
        )
        self.declare_parameter(
            "lane_right_boundary_path_topic", "/camera/path_bev_lane_right"
        )
        self.declare_parameter("measured_erpm_topic", "/vesc/measured_erpm")
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("duty_topic", "/vesc/duty")
        self.declare_parameter("brake_current_topic", "/vesc/brake_current")
        self.declare_parameter(
            "servo_position_topic", "/vesc/servo_position"
        )
        self.declare_parameter("command_duty_topic", "/auto/current_duty")
        self.declare_parameter(
            "command_brake_current_topic", "/auto/current_brake_current"
        )
        self.declare_parameter("target_speed_topic", "/auto/target_speed")
        self.declare_parameter("current_speed_topic", "/auto/current_speed")
        self.declare_parameter("curvature_topic", "/auto/path_curvature")
        self.declare_parameter(
            "steering_angle_topic", "/auto/steering_angle_rad"
        )
        self.declare_parameter(
            "cross_track_error_topic", "/auto/cross_track_error_m"
        )
        self.declare_parameter(
            "heading_error_topic", "/auto/heading_error_rad"
        )
        self.declare_parameter(
            "raw_steering_angle_topic", "/auto/raw_steering_angle_rad"
        )
        self.declare_parameter(
            "command_servo_position_topic", "/auto/current_servo_position"
        )
        self.declare_parameter("planned_path_topic", "/auto/planned_path")

        self.declare_parameter("control_rate_hz", 80.0)
        self.declare_parameter("status_log_rate_hz", 2.0)
        self.declare_parameter("path_timeout_sec", 0.15)
        self.declare_parameter("path_capture_maximum_age_sec", 0.20)
        self.declare_parameter("erpm_timeout_sec", 0.15)

        self.declare_parameter("bev_x_max_m", 3.0)
        self.declare_parameter("bev_y_max_m", 0.60)
        self.declare_parameter("bev_meter_per_pixel", 0.01)
        self.declare_parameter("lane_pixel_threshold", 128)
        self.declare_parameter("path_minimum_x_m", 0.05)
        self.declare_parameter("path_maximum_x_m", 2.95)
        self.declare_parameter("path_minimum_points", 8)
        self.declare_parameter("path_minimum_span_m", 0.12)
        self.declare_parameter("path_local_smoothing_window_m", 0.08)
        self.declare_parameter("path_outlier_threshold_m", 0.04)
        self.declare_parameter("path_geometry_window_m", 0.14)

        # Ordered-path corridor and conservative out-in-out local planning.
        self.declare_parameter("local_path_planner_enabled", True)
        self.declare_parameter("local_path_expected_lane_width_m", 0.65)
        self.declare_parameter("local_path_vehicle_width_m", 0.30)
        self.declare_parameter("local_path_vehicle_wheelbase_m", 0.324)
        self.declare_parameter("local_path_safety_margin_m", 0.08)
        self.declare_parameter("local_path_minimum_half_width_m", 0.24)
        self.declare_parameter("local_path_maximum_half_width_m", 0.48)
        self.declare_parameter("local_path_width_smoothing_window_m", 0.20)
        self.declare_parameter("local_path_maximum_center_correction_m", 0.08)
        self.declare_parameter("local_path_resample_interval_m", 0.02)
        self.declare_parameter(
            "local_path_corner_curvature_threshold_per_m", 0.45
        )
        self.declare_parameter(
            "local_path_corner_curvature_smoothing_window_m", 0.16
        )
        self.declare_parameter(
            "local_path_corner_minimum_heading_change_deg", 20.0
        )
        self.declare_parameter("local_path_corner_approach_length_m", 0.35)
        self.declare_parameter("local_path_corner_exit_length_m", 0.40)
        self.declare_parameter("local_path_outside_offset_fraction", 0.25)
        self.declare_parameter("local_path_apex_offset_fraction", 0.35)
        self.declare_parameter("local_path_maximum_offset_m", 0.08)
        self.declare_parameter("local_path_maximum_offset_slope", 0.70)
        self.declare_parameter("local_path_racing_line_weight", 0.20)
        self.declare_parameter("local_path_maximum_curvature_per_m", 1.8)

        self.declare_parameter("stanley_gain", 1.40)
        self.declare_parameter("stanley_softening_speed_mps", 0.40)
        self.declare_parameter("stanley_heading_lookahead_m", 0.15)
        self.declare_parameter(
            "stanley_corner_heading_threshold_deg", 4.0
        )
        self.declare_parameter(
            "stanley_corner_opposing_correction_ratio", 0.45
        )
        self.declare_parameter("maximum_steering_angle_deg", 30.0)
        self.declare_parameter("steering_current_weight", 0.47)
        self.declare_parameter("steering_rate_limit_deg_per_sec", 240.0)
        self.declare_parameter("servo_left", 1.0)
        self.declare_parameter("servo_center", 0.46)
        self.declare_parameter("servo_right", 0.0)
        # The installed servo linkage reverses the mathematical steering
        # direction. Keep desired vehicle steering signs unchanged and invert
        # only the final actuator mapping.
        self.declare_parameter("steering_servo_inverted", True)

        self.declare_parameter("minimum_speed_mps", 0.80)
        self.declare_parameter("maximum_speed_mps", 1.8)
        self.declare_parameter("maximum_lateral_acceleration_mps2", 0.6)
        self.declare_parameter("curvature_lookahead_minimum_x_m", 0.50)
        self.declare_parameter("curvature_lookahead_maximum_x_m", 1.60)
        self.declare_parameter("curvature_percentile", 90.0)
        self.declare_parameter("spatial_speed_profile_enabled", True)
        self.declare_parameter(
            "speed_profile_maximum_longitudinal_acceleration_mps2", 1.2
        )
        self.declare_parameter(
            "speed_profile_maximum_longitudinal_deceleration_mps2", 1.8
        )
        self.declare_parameter(
            "speed_profile_minimum_combined_braking_ratio", 0.20
        )
        self.declare_parameter("speed_profile_lookahead_minimum_s_m", 0.05)
        self.declare_parameter("speed_profile_lookahead_maximum_s_m", 2.8)
        self.declare_parameter("speed_profile_sample_interval_m", 0.04)
        self.declare_parameter(
            "speed_profile_acceleration_control_distance_m", 0.20
        )
        self.declare_parameter("minimum_duty", 0.070)
        self.declare_parameter("maximum_duty", 0.090)
        self.declare_parameter("duty_rise_rate_per_sec", 0.04)
        self.declare_parameter("duty_fall_rate_per_sec", 0.08)
        self.declare_parameter("speed_pid_kp", 0.012)
        self.declare_parameter("speed_pid_ki", 0.004)
        self.declare_parameter("speed_pid_kd", 0.0)
        self.declare_parameter("speed_pid_integral_limit", 1.0)
        self.declare_parameter("speed_filter_time_constant_sec", 0.05)
        self.declare_parameter("electrical_brake_enabled", True)
        self.declare_parameter("brake_entry_speed_error_mps", 0.10)
        self.declare_parameter("brake_exit_speed_error_mps", 0.03)
        self.declare_parameter("brake_minimum_vehicle_speed_mps", 0.20)
        self.declare_parameter("brake_minimum_current_amps", 0.5)
        self.declare_parameter("brake_maximum_current_amps", 2.5)
        self.declare_parameter("brake_current_gain_amps_per_mps", 6.0)
        self.declare_parameter("brake_current_rise_amps_per_sec", 6.0)
        self.declare_parameter("brake_current_fall_amps_per_sec", 16.0)

        self.declare_parameter("wheel_diameter_m", 0.1095)
        self.declare_parameter("motor_pole_pairs", 2)
        self.declare_parameter("motor_pinion_teeth", 13)
        self.declare_parameter("spur_gear_teeth", 54)
        self.declare_parameter("differential_pinion_teeth", 13)
        self.declare_parameter("differential_ring_teeth", 37)
        self.declare_parameter("erpm_direction_sign", 1.0)
        self.declare_parameter("speed_scale_correction", 1.0)

    def _read_parameters(self) -> None:
        string_parameters = (
            "enable_topic",
            "path_input_mode",
            "lane_topic",
            "lane_path_topic",
            "lane_left_boundary_path_topic",
            "lane_right_boundary_path_topic",
            "measured_erpm_topic",
            "connection_status_topic",
            "duty_topic",
            "brake_current_topic",
            "servo_position_topic",
            "command_duty_topic",
            "command_brake_current_topic",
            "target_speed_topic",
            "current_speed_topic",
            "curvature_topic",
            "steering_angle_topic",
            "cross_track_error_topic",
            "heading_error_topic",
            "raw_steering_angle_topic",
            "command_servo_position_topic",
            "planned_path_topic",
        )
        float_parameters = (
            "control_rate_hz",
            "status_log_rate_hz",
            "path_timeout_sec",
            "path_capture_maximum_age_sec",
            "erpm_timeout_sec",
            "bev_x_max_m",
            "bev_y_max_m",
            "bev_meter_per_pixel",
            "path_minimum_x_m",
            "path_maximum_x_m",
            "path_minimum_span_m",
            "path_local_smoothing_window_m",
            "path_outlier_threshold_m",
            "path_geometry_window_m",
            "local_path_expected_lane_width_m",
            "local_path_vehicle_width_m",
            "local_path_vehicle_wheelbase_m",
            "local_path_safety_margin_m",
            "local_path_minimum_half_width_m",
            "local_path_maximum_half_width_m",
            "local_path_width_smoothing_window_m",
            "local_path_maximum_center_correction_m",
            "local_path_resample_interval_m",
            "local_path_corner_curvature_threshold_per_m",
            "local_path_corner_curvature_smoothing_window_m",
            "local_path_corner_minimum_heading_change_deg",
            "local_path_corner_approach_length_m",
            "local_path_corner_exit_length_m",
            "local_path_outside_offset_fraction",
            "local_path_apex_offset_fraction",
            "local_path_maximum_offset_m",
            "local_path_maximum_offset_slope",
            "local_path_racing_line_weight",
            "local_path_maximum_curvature_per_m",
            "stanley_gain",
            "stanley_softening_speed_mps",
            "stanley_heading_lookahead_m",
            "stanley_corner_heading_threshold_deg",
            "stanley_corner_opposing_correction_ratio",
            "maximum_steering_angle_deg",
            "steering_current_weight",
            "steering_rate_limit_deg_per_sec",
            "servo_left",
            "servo_center",
            "servo_right",
            "minimum_speed_mps",
            "maximum_speed_mps",
            "maximum_lateral_acceleration_mps2",
            "curvature_lookahead_minimum_x_m",
            "curvature_lookahead_maximum_x_m",
            "curvature_percentile",
            "speed_profile_maximum_longitudinal_acceleration_mps2",
            "speed_profile_maximum_longitudinal_deceleration_mps2",
            "speed_profile_minimum_combined_braking_ratio",
            "speed_profile_lookahead_minimum_s_m",
            "speed_profile_lookahead_maximum_s_m",
            "speed_profile_sample_interval_m",
            "speed_profile_acceleration_control_distance_m",
            "minimum_duty",
            "maximum_duty",
            "duty_rise_rate_per_sec",
            "duty_fall_rate_per_sec",
            "speed_pid_kp",
            "speed_pid_ki",
            "speed_pid_kd",
            "speed_pid_integral_limit",
            "speed_filter_time_constant_sec",
            "brake_entry_speed_error_mps",
            "brake_exit_speed_error_mps",
            "brake_minimum_vehicle_speed_mps",
            "brake_minimum_current_amps",
            "brake_maximum_current_amps",
            "brake_current_gain_amps_per_mps",
            "brake_current_rise_amps_per_sec",
            "brake_current_fall_amps_per_sec",
            "wheel_diameter_m",
            "erpm_direction_sign",
            "speed_scale_correction",
        )
        int_parameters = (
            "lane_pixel_threshold",
            "path_minimum_points",
            "motor_pole_pairs",
            "motor_pinion_teeth",
            "spur_gear_teeth",
            "differential_pinion_teeth",
            "differential_ring_teeth",
        )
        self.enabled = bool(self.get_parameter("enabled").value)
        self.electrical_brake_enabled = bool(
            self.get_parameter("electrical_brake_enabled").value
        )
        self.local_path_planner_enabled = bool(
            self.get_parameter("local_path_planner_enabled").value
        )
        self.spatial_speed_profile_enabled = bool(
            self.get_parameter("spatial_speed_profile_enabled").value
        )
        self.steering_servo_inverted = bool(
            self.get_parameter("steering_servo_inverted").value
        )
        for name in string_parameters:
            setattr(self, name, str(self.get_parameter(name).value))
        for name in float_parameters:
            setattr(self, name, float(self.get_parameter(name).value))
        for name in int_parameters:
            setattr(self, name, int(self.get_parameter(name).value))
        self.maximum_steering_angle_rad = math.radians(
            self.maximum_steering_angle_deg
        )
        self.steering_rate_limit_rad_per_sec = math.radians(
            self.steering_rate_limit_deg_per_sec
        )
        self.stanley_corner_heading_threshold_rad = math.radians(
            self.stanley_corner_heading_threshold_deg
        )

    def _validate_parameters(self) -> None:
        if self.control_rate_hz <= 0.0 or self.status_log_rate_hz <= 0.0:
            raise ValueError("control and status rates must be positive")
        if min(
            self.path_timeout_sec,
            self.path_capture_maximum_age_sec,
            self.erpm_timeout_sec,
            self.bev_x_max_m,
            self.bev_y_max_m,
            self.bev_meter_per_pixel,
            self.path_minimum_span_m,
            self.path_local_smoothing_window_m,
            self.path_geometry_window_m,
            self.local_path_expected_lane_width_m,
            self.local_path_vehicle_width_m,
            self.local_path_vehicle_wheelbase_m,
            self.local_path_minimum_half_width_m,
            self.local_path_maximum_half_width_m,
            self.local_path_width_smoothing_window_m,
            self.local_path_resample_interval_m,
            self.local_path_corner_curvature_threshold_per_m,
            self.local_path_corner_curvature_smoothing_window_m,
            self.local_path_corner_minimum_heading_change_deg,
            self.local_path_corner_approach_length_m,
            self.local_path_corner_exit_length_m,
            self.local_path_maximum_offset_slope,
            self.local_path_maximum_curvature_per_m,
            self.maximum_steering_angle_rad,
            self.minimum_speed_mps,
            self.maximum_lateral_acceleration_mps2,
            self.duty_rise_rate_per_sec,
            self.duty_fall_rate_per_sec,
            self.speed_filter_time_constant_sec,
            self.speed_profile_maximum_longitudinal_acceleration_mps2,
            self.speed_profile_maximum_longitudinal_deceleration_mps2,
            self.speed_profile_lookahead_maximum_s_m,
            self.speed_profile_sample_interval_m,
            self.speed_profile_acceleration_control_distance_m,
            self.wheel_diameter_m,
        ) <= 0.0:
            raise ValueError("positive auto-control parameter is not positive")
        if self.maximum_speed_mps < self.minimum_speed_mps:
            raise ValueError("maximum_speed_mps must be >= minimum_speed_mps")
        if self.path_input_mode not in ("ordered_path", "image"):
            raise ValueError("path_input_mode must be 'ordered_path' or 'image'")
        if self.local_path_safety_margin_m < 0.0:
            raise ValueError("local_path_safety_margin_m must not be negative")
        if self.local_path_maximum_center_correction_m < 0.0:
            raise ValueError(
                "local_path_maximum_center_correction_m must not be negative"
            )
        if self.local_path_maximum_half_width_m < self.local_path_minimum_half_width_m:
            raise ValueError("local path half-width limits are reversed")
        if (
            0.5 * self.local_path_vehicle_width_m + self.local_path_safety_margin_m
            >= self.local_path_maximum_half_width_m
        ):
            raise ValueError("vehicle half-width plus safety margin leaves no corridor")
        if not 0.0 <= self.local_path_outside_offset_fraction <= 1.0:
            raise ValueError("local_path_outside_offset_fraction must be in [0, 1]")
        if not 0.0 <= self.local_path_apex_offset_fraction <= 1.0:
            raise ValueError("local_path_apex_offset_fraction must be in [0, 1]")
        if not 0.0 <= self.local_path_racing_line_weight <= 1.0:
            raise ValueError("local_path_racing_line_weight must be in [0, 1]")
        if self.local_path_maximum_offset_m < 0.0:
            raise ValueError("local_path_maximum_offset_m must not be negative")
        if not 0.0 < self.speed_profile_minimum_combined_braking_ratio <= 1.0:
            raise ValueError(
                "speed_profile_minimum_combined_braking_ratio must be in (0, 1]"
            )
        if (
            self.speed_profile_lookahead_minimum_s_m < 0.0
            or self.speed_profile_lookahead_maximum_s_m
            <= self.speed_profile_lookahead_minimum_s_m
        ):
            raise ValueError("speed profile lookahead limits are invalid")
        if not 0.0 < self.minimum_duty <= self.maximum_duty <= 1.0:
            raise ValueError("duty limits must satisfy 0 < minimum <= maximum <= 1")
        if not 0.0 < self.steering_current_weight <= 1.0:
            raise ValueError("steering_current_weight must be in (0, 1]")
        if self.steering_rate_limit_rad_per_sec <= 0.0:
            raise ValueError("steering_rate_limit_deg_per_sec must be positive")
        if self.speed_pid_integral_limit < 0.0:
            raise ValueError("speed_pid_integral_limit must not be negative")
        if self.speed_scale_correction <= 0.0:
            raise ValueError("speed_scale_correction must be positive")
        if not 0.0 <= self.curvature_percentile <= 100.0:
            raise ValueError("curvature_percentile must be in [0, 100]")
        if self.stanley_corner_heading_threshold_deg < 0.0:
            raise ValueError(
                "stanley_corner_heading_threshold_deg must not be negative"
            )
        if not 0.0 <= self.stanley_corner_opposing_correction_ratio < 1.0:
            raise ValueError(
                "stanley_corner_opposing_correction_ratio must be in [0, 1)"
            )
        if self.path_minimum_x_m >= self.path_maximum_x_m:
            raise ValueError("path X limits are reversed")
        if self.path_minimum_points < 3:
            raise ValueError("path_minimum_points must be at least 3")
        if self.path_outlier_threshold_m < 0.0:
            raise ValueError("path_outlier_threshold_m must not be negative")
        if min(
            self.motor_pole_pairs,
            self.motor_pinion_teeth,
            self.spur_gear_teeth,
            self.differential_pinion_teeth,
            self.differential_ring_teeth,
        ) <= 0:
            raise ValueError("motor and gear parameters must be positive")

    @staticmethod
    def _header_stamp_ns(message: Path) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )

    @staticmethod
    def _points_from_path(message: Path) -> np.ndarray:
        if not message.poses:
            return np.empty((0, 2), dtype=float)
        return np.asarray(
            [
                (pose.pose.position.x, pose.pose.position.y)
                for pose in message.poses
            ],
            dtype=float,
        )

    def _on_left_boundary_path(self, message: Path) -> None:
        if self.path_input_mode != "ordered_path":
            return
        stamp = self._header_stamp_ns(message)
        self._ordered_left_boundary = (stamp, self._points_from_path(message))
        self._rebuild_ordered_path(stamp)

    def _on_right_boundary_path(self, message: Path) -> None:
        if self.path_input_mode != "ordered_path":
            return
        stamp = self._header_stamp_ns(message)
        self._ordered_right_boundary = (stamp, self._points_from_path(message))
        self._rebuild_ordered_path(stamp)

    def _on_lane_path(self, message: Path) -> None:
        if self.path_input_mode != "ordered_path":
            return
        stamp = self._header_stamp_ns(message)
        self._ordered_centerline = (
            stamp,
            self._points_from_path(message),
            message.header,
        )
        self._rebuild_ordered_path(stamp)

    def _matching_boundary(
        self, observation: tuple[int, np.ndarray] | None, stamp: int
    ) -> np.ndarray | None:
        if observation is None or observation[0] != stamp:
            return None
        return observation[1]

    def _rebuild_ordered_path(self, stamp: int) -> None:
        if self._ordered_centerline is None:
            return
        center_stamp, center_points, header = self._ordered_centerline
        if center_stamp != stamp:
            return
        base_path = build_ordered_path_model(
            center_points,
            minimum_points=self.path_minimum_points,
            minimum_span_m=self.path_minimum_span_m,
            minimum_x_m=self.path_minimum_x_m,
            maximum_x_m=self.path_maximum_x_m,
            local_smoothing_window_m=self.path_local_smoothing_window_m,
            outlier_threshold_m=self.path_outlier_threshold_m,
            geometry_window_m=self.path_geometry_window_m,
            resample_interval_m=self.local_path_resample_interval_m,
        )
        now = self.get_clock().now()
        self._last_path_received_time = now
        self._path_capture_time = self._stamp_or_none(header.stamp)
        if base_path is None:
            self._path = None
            self._latest_planner_corner_count = 0
            self._publish_planned_path(None, header)
            return
        left = self._matching_boundary(self._ordered_left_boundary, stamp)
        right = self._matching_boundary(self._ordered_right_boundary, stamp)
        planned = plan_local_racing_path(
            base_path, left, right, self._local_path_planner_config
        )
        self._path = planned.path
        self._latest_planner_corner_count = planned.detected_corner_count
        self._publish_planned_path(planned.path, header)

    def _publish_planned_path(self, path: OrderedPathModel | None, header) -> None:
        message = Path()
        message.header = header
        if path is not None:
            tangent = path.tangent_at_s(path.arc_length_m)
            for point, direction in zip(path.points_m, tangent):
                pose = PoseStamped()
                pose.header = header
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                yaw = math.atan2(float(direction[1]), float(direction[0]))
                pose.pose.orientation.z = math.sin(0.5 * yaw)
                pose.pose.orientation.w = math.cos(0.5 * yaw)
                message.poses.append(pose)
        self._planned_path_pub.publish(message)

    def _on_lane_image(self, message: Image) -> None:
        if self.path_input_mode != "image":
            return
        now = self.get_clock().now()
        self._last_path_received_time = now
        self._path_capture_time = self._message_time_or_none(message)
        try:
            if message.encoding.lower() not in ("mono8", "8uc1"):
                raise ValueError(
                    f"expected mono8 centerline, received {message.encoding}"
                )
            width = int(message.width)
            height = int(message.height)
            step = int(message.step)
            if width <= 0 or height <= 0 or step < width:
                raise ValueError("invalid lane image dimensions")
            expected_size = height * step
            if len(message.data) < expected_size:
                raise ValueError("lane image data is shorter than height*step")
            image = np.frombuffer(message.data, dtype=np.uint8, count=expected_size)
            image = image.reshape((height, step))[:, :width]
            x_m, y_m = centerline_points_from_mono8(
                image,
                x_max_m=self.bev_x_max_m,
                y_max_m=self.bev_y_max_m,
                meter_per_pixel=self.bev_meter_per_pixel,
                threshold=self.lane_pixel_threshold,
            )
            self._path = build_path_model(
                x_m,
                y_m,
                minimum_points=self.path_minimum_points,
                minimum_span_m=self.path_minimum_span_m,
                minimum_x_m=self.path_minimum_x_m,
                maximum_x_m=self.path_maximum_x_m,
                local_smoothing_window_m=(
                    self.path_local_smoothing_window_m
                ),
                outlier_threshold_m=self.path_outlier_threshold_m,
                geometry_window_m=self.path_geometry_window_m,
            )
        except (TypeError, ValueError) as exception:
            self._path = None
            self.get_logger().warn(
                f"Rejected BEV centerline: {exception}",
                throttle_duration_sec=1.0,
            )

    def _message_time_or_none(self, message: Image) -> Time | None:
        return self._stamp_or_none(message.header.stamp)

    def _stamp_or_none(self, stamp) -> Time | None:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        return Time.from_msg(
            stamp,
            clock_type=self.get_clock().clock_type,
        )

    def _on_measured_erpm(self, message: Int32) -> None:
        now = self.get_clock().now()
        raw_speed_mps = erpm_to_speed_mps(
            int(message.data),
            wheel_diameter_m=self.wheel_diameter_m,
            motor_pole_pairs=self.motor_pole_pairs,
            motor_pinion_teeth=self.motor_pinion_teeth,
            spur_gear_teeth=self.spur_gear_teeth,
            differential_pinion_teeth=self.differential_pinion_teeth,
            differential_ring_teeth=self.differential_ring_teeth,
            direction_sign=self.erpm_direction_sign,
            scale_correction=self.speed_scale_correction,
        )
        if self._last_erpm_time is None:
            self._current_speed_mps = raw_speed_mps
        else:
            dt_sec = self._elapsed_sec(self._last_erpm_time, now)
            if dt_sec <= 0.0 or dt_sec > self.erpm_timeout_sec:
                self._current_speed_mps = raw_speed_mps
            else:
                gain = 1.0 - math.exp(
                    -dt_sec / max(1.0e-4, self.speed_filter_time_constant_sec)
                )
                self._current_speed_mps += gain * (
                    raw_speed_mps - self._current_speed_mps
                )
        self._last_erpm_time = now

    def _on_connection_status(self, message: Bool) -> None:
        self._vesc_connected = bool(message.data)

    def _on_enable(self, message: Bool) -> None:
        self.enabled = bool(message.data)
        if not self.enabled:
            self._stop_control("disabled")
            self._publish_commands(
                duty=0.0,
                brake_current=0.0,
                servo_position=self.servo_center,
                motor_mode="stop",
            )
        self.get_logger().warn(
            f"Automatic control {'enabled' if self.enabled else 'disabled'}."
        )

    def _on_control_timer(self) -> None:
        now = self.get_clock().now()
        nominal_period_sec = 1.0 / self.control_rate_hz
        dt_sec = clamp(
            self._elapsed_sec(self._last_control_time, now),
            0.0,
            2.0 * nominal_period_sec,
        )
        self._last_control_time = now

        stop_reason = self._stop_reason(now)
        if stop_reason is not None:
            self._stop_control(stop_reason)
            self._publish_commands(
                duty=0.0,
                brake_current=0.0,
                servo_position=self.servo_center,
                motor_mode="stop",
            )
            return

        assert self._path is not None
        if isinstance(self._path, OrderedPathModel):
            speed_plan = spatial_speed_plan(
                self._path,
                current_speed_mps=self._current_speed_mps,
                config=self._speed_profile_config,
            )
            curvature_per_m = speed_plan.representative_curvature_per_m
            target_speed_mps = speed_plan.target_speed_mps
            stanley = stanley_control_ordered(
                self._path,
                speed_mps=self._current_speed_mps,
                gain=self.stanley_gain,
                softening_speed_mps=self.stanley_softening_speed_mps,
                heading_lookahead_m=self.stanley_heading_lookahead_m,
                maximum_steering_angle_rad=self.maximum_steering_angle_rad,
                corner_heading_threshold_rad=(
                    self.stanley_corner_heading_threshold_rad
                ),
                corner_opposing_correction_ratio=(
                    self.stanley_corner_opposing_correction_ratio
                ),
            )
        else:
            curvature_per_m = representative_curvature(
                self._path,
                lookahead_minimum_x_m=self.curvature_lookahead_minimum_x_m,
                lookahead_maximum_x_m=self.curvature_lookahead_maximum_x_m,
                percentile=self.curvature_percentile,
            )
            target_speed_mps = curvature_target_speed(
                curvature_per_m,
                maximum_lateral_acceleration_mps2=(
                    self.maximum_lateral_acceleration_mps2
                ),
                minimum_speed_mps=self.minimum_speed_mps,
                maximum_speed_mps=self.maximum_speed_mps,
            )
            stanley = stanley_control(
                self._path,
                speed_mps=self._current_speed_mps,
                gain=self.stanley_gain,
                softening_speed_mps=self.stanley_softening_speed_mps,
                heading_lookahead_m=self.stanley_heading_lookahead_m,
                maximum_steering_angle_rad=self.maximum_steering_angle_rad,
                corner_heading_threshold_rad=(
                    self.stanley_corner_heading_threshold_rad
                ),
                corner_opposing_correction_ratio=(
                    self.stanley_corner_opposing_correction_ratio
                ),
            )

        corner_sign_reset_used = (
            abs(stanley.heading_error_rad)
            >= self.stanley_corner_heading_threshold_rad
            and stanley.steering_angle_rad * self._steering_angle_rad < 0.0
        )
        if corner_sign_reset_used:
            # Do not let temporal smoothing retain a left command after the
            # fitted path has clearly entered a right corner, or vice versa.
            self._steering_angle_rad = 0.0
            filtered_steering_rad = stanley.steering_angle_rad
        else:
            filtered_steering_rad = (
                self.steering_current_weight * stanley.steering_angle_rad
                + (1.0 - self.steering_current_weight)
                * self._steering_angle_rad
            )
        self._steering_angle_rad = move_toward(
            self._steering_angle_rad,
            filtered_steering_rad,
            self.steering_rate_limit_rad_per_sec * dt_sec,
        )
        servo_position = steering_angle_to_servo(
            self._steering_angle_rad,
            maximum_steering_angle_rad=self.maximum_steering_angle_rad,
            servo_left=self.servo_left,
            servo_center=self.servo_center,
            servo_right=self.servo_right,
            inverted=self.steering_servo_inverted,
        )

        if self.electrical_brake_enabled:
            self._command_brake_current = self._brake_profile.update(
                target_speed_mps=target_speed_mps,
                current_speed_mps=self._current_speed_mps,
                dt_sec=max(dt_sec, nominal_period_sec),
            )
        else:
            self._brake_profile.reset()
            self._command_brake_current = 0.0

        if self._command_brake_current > 0.0:
            # Brake current owns the VESC motor mode. Discard propulsion PID
            # state so positive duty can never fight an active brake command.
            self._command_duty = 0.0
            self._speed_pid.reset()
            motor_mode = "brake"
            self._brake_mode_active = True
        elif self._brake_mode_active:
            # Explicitly release COMM_SET_CURRENT_BRAKE, then wait one control
            # cycle before returning to positive duty.
            self._command_duty = 0.0
            self._speed_pid.reset()
            motor_mode = "brake_release"
            self._brake_mode_active = False
        else:
            feedforward_duty = speed_feedforward_duty(
                target_speed_mps,
                minimum_speed_mps=self.minimum_speed_mps,
                maximum_speed_mps=self.maximum_speed_mps,
                minimum_duty=self.minimum_duty,
                maximum_duty=self.maximum_duty,
            )
            desired_duty = self._speed_pid.update(
                target_speed_mps=target_speed_mps,
                current_speed_mps=self._current_speed_mps,
                feedforward_duty=feedforward_duty,
                dt_sec=max(dt_sec, nominal_period_sec),
            )
            if self._command_duty < self.minimum_duty:
                # Match manual driving's start-duty behavior when a path appears.
                self._command_duty = self.minimum_duty
            else:
                rate = (
                    self.duty_rise_rate_per_sec
                    if desired_duty >= self._command_duty
                    else self.duty_fall_rate_per_sec
                )
                self._command_duty = move_toward(
                    self._command_duty, desired_duty, rate * dt_sec
                )
            motor_mode = "duty"

        self._last_stop_reason = "running"
        self._latest_target_speed_mps = target_speed_mps
        self._latest_curvature_per_m = curvature_per_m
        self._latest_cross_track_error_m = stanley.cross_track_error_m
        self._latest_heading_error_rad = stanley.heading_error_rad
        self._latest_raw_steering_angle_rad = stanley.steering_angle_rad
        self._latest_direction_guard_used = (
            stanley.direction_guard_used or corner_sign_reset_used
        )
        self._publish_commands(
            duty=self._command_duty,
            brake_current=self._command_brake_current,
            servo_position=servo_position,
            motor_mode=motor_mode,
        )

    def _stop_reason(self, now: Time) -> str | None:
        if not self.enabled:
            return "disabled"
        if not self._vesc_connected:
            return "vesc_disconnected"
        if self._last_erpm_time is None:
            return "waiting_for_erpm"
        if self._elapsed_sec(self._last_erpm_time, now) > self.erpm_timeout_sec:
            return "erpm_timeout"
        if self._path is None or self._last_path_received_time is None:
            return "centerline_missing"
        if (
            self._elapsed_sec(self._last_path_received_time, now)
            > self.path_timeout_sec
        ):
            return "centerline_timeout"
        if self._path_capture_time is not None and (
            self._elapsed_sec(self._path_capture_time, now)
            > self.path_capture_maximum_age_sec
        ):
            return "centerline_stale"
        return None

    def _stop_control(self, reason: str) -> None:
        self._command_duty = 0.0
        self._command_brake_current = 0.0
        self._brake_mode_active = False
        self._steering_angle_rad = 0.0
        self._latest_target_speed_mps = 0.0
        self._latest_curvature_per_m = 0.0
        self._latest_cross_track_error_m = 0.0
        self._latest_heading_error_rad = 0.0
        self._latest_raw_steering_angle_rad = 0.0
        self._latest_servo_position = self.servo_center
        self._latest_direction_guard_used = False
        self._speed_pid.reset()
        self._brake_profile.reset()
        if reason != self._last_stop_reason:
            self.get_logger().warn(
                f"Automatic drive stopped: {reason}. Releasing motor command."
            )
        self._last_stop_reason = reason

    def _publish_commands(
        self,
        *,
        duty: float,
        brake_current: float,
        servo_position: float,
        motor_mode: str,
    ) -> None:
        if motor_mode == "duty":
            self._duty_pub.publish(Float32(data=float(duty)))
        elif motor_mode == "brake":
            self._brake_current_pub.publish(
                Float32(data=float(brake_current))
            )
        elif motor_mode == "brake_release":
            self._brake_current_pub.publish(Float32(data=0.0))
        elif motor_mode == "stop":
            # Either zero command safely supersedes a previous drive mode.
            # Publish both so the bridge state and diagnostics are explicit.
            self._brake_current_pub.publish(Float32(data=0.0))
            self._duty_pub.publish(Float32(data=0.0))
        else:
            raise ValueError(f"unsupported motor mode: {motor_mode}")
        self._last_motor_mode = motor_mode
        self._latest_servo_position = float(servo_position)
        self._servo_pub.publish(Float32(data=float(servo_position)))
        self._command_duty_pub.publish(Float32(data=float(duty)))
        self._command_brake_current_pub.publish(
            Float32(data=float(brake_current))
        )
        self._target_speed_pub.publish(
            Float32(data=float(self._latest_target_speed_mps))
        )
        self._current_speed_pub.publish(
            Float32(data=float(self._current_speed_mps))
        )
        self._curvature_pub.publish(
            Float32(data=float(self._latest_curvature_per_m))
        )
        self._steering_angle_pub.publish(
            Float32(data=float(self._steering_angle_rad))
        )
        self._cross_track_error_pub.publish(
            Float32(data=float(self._latest_cross_track_error_m))
        )
        self._heading_error_pub.publish(
            Float32(data=float(self._latest_heading_error_rad))
        )
        self._raw_steering_angle_pub.publish(
            Float32(data=float(self._latest_raw_steering_angle_rad))
        )
        self._command_servo_position_pub.publish(
            Float32(data=self._latest_servo_position)
        )

    def _log_status(self) -> None:
        path_points = self._path.point_count if self._path is not None else 0
        self.get_logger().info(
            "Auto status | state=%s | path_points=%d | speed=%.2f/%.2fm/s | "
            "curvature=%.3f/m | planner_corners=%d | cte=%+.3fm | "
            "heading=%+.1fdeg | guard=%s | "
            "raw/final_steering=%+.1f/%+.1fdeg | servo=%.3f | "
            "motor=%s | duty=%.4f | brake=%.2fA"
            % (
                self._last_stop_reason,
                path_points,
                self._current_speed_mps,
                self._latest_target_speed_mps,
                self._latest_curvature_per_m,
                self._latest_planner_corner_count,
                self._latest_cross_track_error_m,
                math.degrees(self._latest_heading_error_rad),
                "on" if self._latest_direction_guard_used else "off",
                math.degrees(self._latest_raw_steering_angle_rad),
                math.degrees(self._steering_angle_rad),
                self._latest_servo_position,
                self._last_motor_mode,
                self._command_duty,
                self._command_brake_current,
            )
        )

    @staticmethod
    def _elapsed_sec(start: Time, end: Time) -> float:
        return max(0.0, (end - start).nanoseconds / 1_000_000_000.0)

    def stop_actuators(self) -> None:
        self._control_timer.cancel()
        self._status_timer.cancel()
        self._stop_control("shutdown")
        self._publish_commands(
            duty=0.0,
            brake_current=0.0,
            servo_position=self.servo_center,
            motor_mode="stop",
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutoControlNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.stop_actuators()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
