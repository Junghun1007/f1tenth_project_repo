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
    controller_arguments = [
        ("auto_enabled", "true", "enabled", bool),
        ("minimum_duty", "0.05", "minimum_duty", float),
        ("maximum_duty", "0.06", "maximum_duty", float),
        ("minimum_speed_mps", "0.8", "minimum_speed_mps", float),
        ("maximum_speed_mps", "1.2", "maximum_speed_mps", float),
        ("stanley_gain", "1.2", "stanley_gain", float),
        (
            "stanley_softening_speed_mps",
            "0.50",
            "stanley_softening_speed_mps",
            float,
        ),
        (
            "stanley_heading_lookahead_m",
            "0.10",
            "stanley_heading_lookahead_m",
            float,
        ),
        (
            "stanley_corner_heading_threshold_deg",
            "6.0",
            "stanley_corner_heading_threshold_deg",
            float,
        ),
        (
            "stanley_corner_opposing_correction_ratio",
            "0.80",
            "stanley_corner_opposing_correction_ratio",
            float,
        ),
        (
            "steering_current_weight",
            "0.45",
            "steering_current_weight",
            float,
        ),
        (
            "maximum_lateral_acceleration_mps2",
            "0.8",
            "maximum_lateral_acceleration_mps2",
            float,
        ),
        ("curvature_percentile", "90.0", "curvature_percentile", float),
        ("speed_pid_kp", "0.012", "speed_pid_kp", float),
        ("speed_pid_ki", "0.004", "speed_pid_ki", float),
        ("speed_pid_kd", "0.0", "speed_pid_kd", float),
        (
            "path_local_smoothing_window_m",
            "0.12",
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
            "0.16",
            "path_geometry_window_m",
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
                DeclareLaunchArgument(argument_name, default_value=default)
                for argument_name, default, _, _ in controller_arguments
            ],
            bev_launch,
            vesc_bridge_node,
            auto_control_node,
        ]
    )
