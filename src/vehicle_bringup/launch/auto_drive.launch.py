from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vesc_port = LaunchConfiguration("vesc_port")
    preview_enabled = LaunchConfiguration("preview_enabled")
    bev_arguments = [
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
        ("lane_centerline_corner_outward_bias_m", "0.05"),
    ]
    bev_overrides = {
        name: LaunchConfiguration(name) for name, _ in bev_arguments
    }
    controller_arguments = [
        ("auto_enabled", "true", "enabled", bool),
        ("minimum_duty", "0.070", "minimum_duty", float),
        ("maximum_duty", "0.090", "maximum_duty", float),
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
            "path_minimum_x_m",
            "0.05",
            "path_minimum_x_m",
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
    ]
    controller_overrides = {
        parameter_name: ParameterValue(
            LaunchConfiguration(argument_name), value_type=value_type
        )
        for argument_name, _, parameter_name, value_type in controller_arguments
    }

    bev_launch_path = PathJoinSubstitution(
        [FindPackageShare("bev_processor"), "launch", "bev_processor.launch.py"]
    )
    auto_control_config = PathJoinSubstitution(
        [FindPackageShare("auto_control"), "config", "auto_control.yaml"]
    )
    vesc_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_bringup"), "config", "vesc_config.yaml"]
    )

    bev_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bev_launch_path),
        launch_arguments={
            # Autonomous mode shows only measured lanes and the centerline.
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
                "preview_enabled",
                default_value="true",
                description=(
                    "Show result-only BEV lane preview. Set false for no GUI."
                ),
            ),
            *[
                DeclareLaunchArgument(name, default_value=default)
                for name, default in bev_arguments
            ],
            *[
                DeclareLaunchArgument(argument_name, default_value=default)
                for argument_name, default, _, _ in controller_arguments
            ],
            bev_launch,
            vesc_bridge_node,
            auto_control_node,
        ]
    )
