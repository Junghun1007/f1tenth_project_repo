import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_PARAMETER_FILE_DEFAULT = "__PARAMETER_FILE_DEFAULT__"


def _ros_parameters(config_path, node_name):
    with open(config_path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    return config.get(node_name, {}).get("ros__parameters", {})


def _launch_default(parameters, name, fallback):
    value = parameters.get(name, fallback)
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _merged_parameters(base_path, selected_path, node_name):
    parameters = _ros_parameters(base_path, node_name)
    selected_path = os.path.abspath(selected_path)
    if not os.path.isfile(selected_path):
        raise RuntimeError(f"parameter file does not exist: {selected_path}")
    if os.path.realpath(selected_path) != os.path.realpath(base_path):
        overrides = _ros_parameters(selected_path, node_name)
        if not overrides:
            raise RuntimeError(
                f"{selected_path} has no {node_name}.ros__parameters section"
            )
        parameters.update(overrides)
    return parameters


def _apply_parameter_file_defaults(
    context,
    *,
    bev_config,
    auto_control_config,
    bev_arguments,
    controller_arguments,
):
    bev_defaults = _merged_parameters(
        bev_config,
        LaunchConfiguration("bev_params_file").perform(context),
        "bev_processor",
    )
    controller_defaults = _merged_parameters(
        auto_control_config,
        LaunchConfiguration("auto_control_params_file").perform(context),
        "auto_control",
    )
    if (
        context.launch_configurations["preview_enabled"]
        == _PARAMETER_FILE_DEFAULT
    ):
        context.launch_configurations["preview_enabled"] = _launch_default(
            bev_defaults, "preview_enabled", "true"
        )
    for name, fallback in bev_arguments:
        if context.launch_configurations[name] == _PARAMETER_FILE_DEFAULT:
            context.launch_configurations[name] = _launch_default(
                bev_defaults, name, fallback
            )
    for argument_name, fallback, parameter_name, _ in controller_arguments:
        if (
            context.launch_configurations[argument_name]
            == _PARAMETER_FILE_DEFAULT
        ):
            context.launch_configurations[argument_name] = _launch_default(
                controller_defaults, parameter_name, fallback
            )
    return []


def generate_launch_description():
    bev_share = get_package_share_directory("bev_processor")
    auto_control_share = get_package_share_directory("auto_control")
    vehicle_bringup_share = get_package_share_directory("vehicle_bringup")
    bev_config = os.path.join(bev_share, "config", "bev_config.yaml")
    auto_control_config = os.path.join(
        auto_control_share, "config", "auto_control.yaml"
    )
    vesc_config = os.path.join(
        vehicle_bringup_share, "config", "vesc_config.yaml"
    )
    bev_defaults = _ros_parameters(bev_config, "bev_processor")
    controller_defaults = _ros_parameters(auto_control_config, "auto_control")

    vesc_port = LaunchConfiguration("vesc_port")
    bev_params_file = LaunchConfiguration("bev_params_file")
    auto_control_params_file = LaunchConfiguration("auto_control_params_file")
    preview_enabled = LaunchConfiguration("preview_enabled")
    bev_argument_fallbacks = [
        ("lane_centerline_path_topic", "/camera/path_bev_lane"),
        ("lane_left_boundary_path_topic", "/camera/path_bev_lane_left"),
        ("lane_right_boundary_path_topic", "/camera/path_bev_lane_right"),
        ("lane_preview_planned_path_enabled", "true"),
        ("lane_preview_planned_path_topic", "/auto/planned_path"),
        ("lane_preview_planned_path_maximum_age_sec", "0.20"),
        ("lane_preview_planned_path_thickness_px", "2"),
        ("lane_seed_roi_height_ratio", "0.25"),
        ("lane_seed_temporal_side_lock_reset_frames", "100"),
        ("lane_seed_temporal_side_reacquire_base_distance_px", "45.0"),
        (
            "lane_seed_temporal_side_reacquire_distance_per_missing_frame_px",
            "3.0",
        ),
        ("lane_seed_temporal_side_reacquire_maximum_distance_px", "63.0"),
        ("lane_seed_pair_minimum_distance_px", "45.0"),
        ("lane_seed_pair_maximum_distance_px", "100.0"),
        ("lane_seed_sliding_window_minimum_seed_arc_length_px", "15.0"),
        ("lane_centerline_corner_outward_bias_m", "0.00"),
    ]
    # YAML is the single source of default values. A value supplied through
    # `ros2 launch ... name:=value` still replaces the declared default.
    bev_arguments = [
        (name, _launch_default(bev_defaults, name, fallback))
        for name, fallback in bev_argument_fallbacks
    ]
    bev_overrides = {
        name: LaunchConfiguration(name) for name, _ in bev_arguments
    }
    controller_argument_fallbacks = [
        ("auto_enabled", "true", "enabled", bool),
        ("path_input_mode", "ordered_path", "path_input_mode", str),
        ("lane_path_topic", "/camera/path_bev_lane", "lane_path_topic", str),
        (
            "lane_left_boundary_path_topic",
            "/camera/path_bev_lane_left",
            "lane_left_boundary_path_topic",
            str,
        ),
        (
            "lane_right_boundary_path_topic",
            "/camera/path_bev_lane_right",
            "lane_right_boundary_path_topic",
            str,
        ),
        ("planned_path_topic", "/auto/planned_path", "planned_path_topic", str),
        ("minimum_duty", "0.070", "minimum_duty", float),
        ("maximum_duty", "0.090", "maximum_duty", float),
        (
            "duty_rise_rate_per_sec",
            "0.04",
            "duty_rise_rate_per_sec",
            float,
        ),
        (
            "duty_fall_rate_per_sec",
            "0.08",
            "duty_fall_rate_per_sec",
            float,
        ),
        ("minimum_speed_mps", "0.80", "minimum_speed_mps", float),
        ("maximum_speed_mps", "1.8", "maximum_speed_mps", float),
        ("stanley_gain", "1.40", "stanley_gain", float),
        (
            "stanley_softening_speed_mps",
            "0.40",
            "stanley_softening_speed_mps",
            float,
        ),
        (
            "stanley_heading_lookahead_m",
            "0.15",
            "stanley_heading_lookahead_m",
            float,
        ),
        (
            "stanley_corner_heading_threshold_deg",
            "4.0",
            "stanley_corner_heading_threshold_deg",
            float,
        ),
        (
            "stanley_corner_opposing_correction_ratio",
            "0.45",
            "stanley_corner_opposing_correction_ratio",
            float,
        ),
        (
            "steering_current_weight",
            "0.47",
            "steering_current_weight",
            float,
        ),
        (
            "steering_servo_inverted",
            "true",
            "steering_servo_inverted",
            bool,
        ),
        (
            "maximum_lateral_acceleration_mps2",
            "0.6",
            "maximum_lateral_acceleration_mps2",
            float,
        ),
        ("curvature_percentile", "90.0", "curvature_percentile", float),
        ("speed_pid_kp", "0.012", "speed_pid_kp", float),
        ("speed_pid_ki", "0.004", "speed_pid_ki", float),
        ("speed_pid_kd", "0.0", "speed_pid_kd", float),
        (
            "speed_pid_integral_limit",
            "1.0",
            "speed_pid_integral_limit",
            float,
        ),
        (
            "speed_filter_time_constant_sec",
            "0.05",
            "speed_filter_time_constant_sec",
            float,
        ),
        (
            "electrical_brake_enabled",
            "true",
            "electrical_brake_enabled",
            bool,
        ),
        (
            "brake_entry_speed_error_mps",
            "0.10",
            "brake_entry_speed_error_mps",
            float,
        ),
        (
            "brake_exit_speed_error_mps",
            "0.03",
            "brake_exit_speed_error_mps",
            float,
        ),
        (
            "brake_minimum_vehicle_speed_mps",
            "0.20",
            "brake_minimum_vehicle_speed_mps",
            float,
        ),
        (
            "brake_minimum_current_amps",
            "0.5",
            "brake_minimum_current_amps",
            float,
        ),
        (
            "brake_maximum_current_amps",
            "2.5",
            "brake_maximum_current_amps",
            float,
        ),
        (
            "brake_current_gain_amps_per_mps",
            "6.0",
            "brake_current_gain_amps_per_mps",
            float,
        ),
        (
            "brake_current_rise_amps_per_sec",
            "6.0",
            "brake_current_rise_amps_per_sec",
            float,
        ),
        (
            "brake_current_fall_amps_per_sec",
            "16.0",
            "brake_current_fall_amps_per_sec",
            float,
        ),
        (
            "path_local_smoothing_window_m",
            "0.08",
            "path_local_smoothing_window_m",
            float,
        ),
        (
            "path_outlier_threshold_m",
            "0.04",
            "path_outlier_threshold_m",
            float,
        ),
        (
            "path_geometry_window_m",
            "0.14",
            "path_geometry_window_m",
            float,
        ),
        (
            "local_path_planner_enabled",
            "true",
            "local_path_planner_enabled",
            bool,
        ),
        (
            "local_path_expected_lane_width_m",
            "0.65",
            "local_path_expected_lane_width_m",
            float,
        ),
        (
            "local_path_vehicle_width_m",
            "0.30",
            "local_path_vehicle_width_m",
            float,
        ),
        (
            "local_path_vehicle_wheelbase_m",
            "0.324",
            "local_path_vehicle_wheelbase_m",
            float,
        ),
        (
            "local_path_safety_margin_m",
            "0.08",
            "local_path_safety_margin_m",
            float,
        ),
        (
            "local_path_minimum_half_width_m",
            "0.24",
            "local_path_minimum_half_width_m",
            float,
        ),
        (
            "local_path_maximum_half_width_m",
            "0.48",
            "local_path_maximum_half_width_m",
            float,
        ),
        (
            "local_path_width_smoothing_window_m",
            "0.20",
            "local_path_width_smoothing_window_m",
            float,
        ),
        (
            "local_path_maximum_center_correction_m",
            "0.08",
            "local_path_maximum_center_correction_m",
            float,
        ),
        (
            "local_path_resample_interval_m",
            "0.02",
            "local_path_resample_interval_m",
            float,
        ),
        (
            "local_path_corner_curvature_threshold_per_m",
            "0.45",
            "local_path_corner_curvature_threshold_per_m",
            float,
        ),
        (
            "local_path_corner_curvature_smoothing_window_m",
            "0.16",
            "local_path_corner_curvature_smoothing_window_m",
            float,
        ),
        (
            "local_path_corner_minimum_heading_change_deg",
            "20.0",
            "local_path_corner_minimum_heading_change_deg",
            float,
        ),
        (
            "local_path_corner_approach_length_m",
            "0.35",
            "local_path_corner_approach_length_m",
            float,
        ),
        (
            "local_path_corner_setup_transition_length_m",
            "0.0",
            "local_path_corner_setup_transition_length_m",
            float,
        ),
        (
            "local_path_corner_pre_turn_outside_hold_m",
            "0.25",
            "local_path_corner_pre_turn_outside_hold_m",
            float,
        ),
        (
            "local_path_corner_exit_length_m",
            "0.40",
            "local_path_corner_exit_length_m",
            float,
        ),
        (
            "local_path_post_corner_offset_hold_distance_m",
            "0.50",
            "local_path_post_corner_offset_hold_distance_m",
            float,
        ),
        (
            "local_path_same_direction_corner_link_maximum_gap_m",
            "1.50",
            "local_path_same_direction_corner_link_maximum_gap_m",
            float,
        ),
        (
            "local_path_outside_offset_fraction",
            "0.25",
            "local_path_outside_offset_fraction",
            float,
        ),
        (
            "local_path_apex_offset_fraction",
            "0.35",
            "local_path_apex_offset_fraction",
            float,
        ),
        (
            "local_path_maximum_offset_m",
            "0.08",
            "local_path_maximum_offset_m",
            float,
        ),
        (
            "local_path_maximum_offset_slope",
            "0.70",
            "local_path_maximum_offset_slope",
            float,
        ),
        (
            "local_path_racing_line_weight",
            "0.20",
            "local_path_racing_line_weight",
            float,
        ),
        (
            "local_path_maximum_curvature_per_m",
            "1.8",
            "local_path_maximum_curvature_per_m",
            float,
        ),
        (
            "path_minimum_x_m",
            "0.05",
            "path_minimum_x_m",
            float,
        ),
        (
            "path_maximum_x_m",
            "2.95",
            "path_maximum_x_m",
            float,
        ),
        (
            "path_minimum_points",
            "8",
            "path_minimum_points",
            int,
        ),
        (
            "path_minimum_span_m",
            "0.12",
            "path_minimum_span_m",
            float,
        ),
        (
            "spatial_speed_profile_enabled",
            "true",
            "spatial_speed_profile_enabled",
            bool,
        ),
        (
            "speed_profile_maximum_longitudinal_acceleration_mps2",
            "1.2",
            "speed_profile_maximum_longitudinal_acceleration_mps2",
            float,
        ),
        (
            "speed_profile_maximum_longitudinal_deceleration_mps2",
            "1.8",
            "speed_profile_maximum_longitudinal_deceleration_mps2",
            float,
        ),
        (
            "speed_profile_minimum_combined_braking_ratio",
            "0.20",
            "speed_profile_minimum_combined_braking_ratio",
            float,
        ),
        (
            "speed_profile_lookahead_minimum_s_m",
            "0.05",
            "speed_profile_lookahead_minimum_s_m",
            float,
        ),
        (
            "speed_profile_lookahead_maximum_s_m",
            "2.8",
            "speed_profile_lookahead_maximum_s_m",
            float,
        ),
        (
            "speed_profile_sample_interval_m",
            "0.04",
            "speed_profile_sample_interval_m",
            float,
        ),
        (
            "speed_profile_acceleration_control_distance_m",
            "0.20",
            "speed_profile_acceleration_control_distance_m",
            float,
        ),
    ]
    controller_arguments = [
        (
            argument_name,
            _launch_default(controller_defaults, parameter_name, fallback),
            parameter_name,
            value_type,
        )
        for (
            argument_name,
            fallback,
            parameter_name,
            value_type,
        ) in controller_argument_fallbacks
    ]
    controller_overrides = {
        parameter_name: ParameterValue(
            LaunchConfiguration(argument_name), value_type=value_type
        )
        for argument_name, _, parameter_name, value_type in controller_arguments
    }

    bev_launch_path = os.path.join(
        bev_share, "launch", "bev_processor.launch.py"
    )

    bev_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bev_launch_path),
        launch_arguments={
            "bev_params_file": bev_params_file,
            # Autonomous mode also overlays the planned path in yellow.
            # preview_enabled:=false disables the OpenCV window completely.
            "preview_enabled": preview_enabled,
            "lane_preview_enabled": "true",
            "lane_preview_result_only_enabled": "true",
            "lane_preview_sliding_windows_enabled": "false",
            **bev_overrides,
        }.items(),
    )

    auto_control_node = Node(
        package="auto_control",
        executable="auto_control_node",
        name="auto_control",
        output="screen",
        parameters=[
            auto_control_config,
            auto_control_params_file,
            controller_overrides,
        ],
    )

    vesc_bridge_node = Node(
        package="vesc_bridge",
        executable="vesc_bridge_node",
        name="vesc_bridge_node",
        output="screen",
        parameters=[vesc_config, {"port": vesc_port}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument(
                "bev_params_file",
                default_value=bev_config,
                description="BEV parameter override YAML",
            ),
            DeclareLaunchArgument(
                "auto_control_params_file",
                default_value=auto_control_config,
                description="Auto-control parameter override YAML",
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value=_PARAMETER_FILE_DEFAULT,
                description=(
                    "Show result-only BEV lane preview. Set false for no GUI."
                ),
            ),
            *[
                DeclareLaunchArgument(
                    name, default_value=_PARAMETER_FILE_DEFAULT
                )
                for name, _ in bev_arguments
            ],
            *[
                DeclareLaunchArgument(
                    argument_name, default_value=_PARAMETER_FILE_DEFAULT
                )
                for argument_name, _, _, _ in controller_arguments
            ],
            OpaqueFunction(
                function=_apply_parameter_file_defaults,
                kwargs={
                    "bev_config": bev_config,
                    "auto_control_config": auto_control_config,
                    "bev_arguments": bev_arguments,
                    "controller_arguments": controller_arguments,
                },
            ),
            bev_launch,
            vesc_bridge_node,
            auto_control_node,
        ]
    )
