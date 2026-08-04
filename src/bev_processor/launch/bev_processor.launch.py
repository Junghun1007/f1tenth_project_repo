import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_share = get_package_share_directory("camera_driver")
    bev_share = get_package_share_directory("bev_processor")

    camera_params = os.path.join(
        camera_share, "config", "camera_config.yaml"
    )
    bev_params = os.path.join(bev_share, "config", "bev_config.yaml")
    performance_measurement_enabled = LaunchConfiguration(
        "performance_measurement_enabled"
    )
    performance_measurement_parameter = ParameterValue(
        performance_measurement_enabled,
        value_type=bool,
    )

    # Keep every lane tuning value overridable from `ros2 launch ... name:=x`.
    # Defaults below mirror the documented YAML baseline. They are explicit
    # launch-time overrides so field tuning never requires rebuilding.
    lane_parameters = [
        ("lane_reconstruction_enabled", "true", bool),
        ("lane_output_topic", "/camera/image_bev_lane", str),
        ("lane_preview_enabled", "true", bool),
        ("lane_minimum_brightness", "160", int),
        ("lane_far_minimum_brightness", "110", int),
        ("lane_maximum_saturation", "255", int),
        ("lane_brightness_blur_kernel", "1", int),
        ("lane_vertical_close_m", "0.05", float),
        ("lane_minimum_mark_width_m", "0.01", float),
        ("lane_maximum_mark_width_m", "0.18", float),
        ("lane_row_step_px", "2", int),
        ("lane_observation_minimum_x_m", "0.20", float),
        ("lane_observation_maximum_x_m", "1.80", float),
        ("lane_reconstruction_minimum_x_m", "0.20", float),
        ("lane_reconstruction_maximum_x_m", "2.70", float),
        ("lane_maximum_extrapolation_m", "0.20", float),
        ("lane_sliding_window_step_m", "0.06", float),
        ("lane_sliding_window_length_m", "0.18", float),
        ("lane_sliding_window_half_width_near_m", "0.12", float),
        ("lane_sliding_window_half_width_far_m", "0.22", float),
        ("lane_sliding_window_measurement_weight", "0.90", float),
        ("lane_sliding_window_heading_weight", "0.80", float),
        ("lane_maximum_tracking_arc_length_m", "3.20", float),
        ("lane_maximum_gap_fill_m", "0.26", float),
        ("lane_measured_point_smoothing_weight", "0.85", float),
        ("lane_minimum_window_pixel_count", "6", int),
        ("lane_expected_width_m", "0.625", float),
        ("lane_width_tolerance_m", "0.20", float),
        ("lane_initial_center_tolerance_m", "0.30", float),
        ("lane_single_initial_tolerance_m", "0.20", float),
        ("lane_maximum_tracking_gap_m", "0.20", float),
        ("lane_minimum_points", "5", int),
        ("lane_allow_single_lane", "true", bool),
        ("lane_output_line_thickness_m", "0.04", float),
    ]
    lane_launch_arguments = [
        DeclareLaunchArgument(
            name,
            default_value=default,
            description=f"Override bev_processor parameter '{name}'.",
        )
        for name, default, _ in lane_parameters
    ]
    lane_parameter_overrides = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type in lane_parameters
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=camera_params,
                description="Camera driver parameter YAML",
            ),
            DeclareLaunchArgument(
                "bev_params_file",
                default_value=bev_params,
                description=(
                    "BEV parameter YAML; its root must be bev_processor"
                ),
            ),
            DeclareLaunchArgument(
                "performance_measurement_enabled",
                default_value="false",
                description=(
                    "Disable GUI previews and print stabilized/BEV pipeline "
                    "performance measurements."
                ),
            ),
            *lane_launch_arguments,
            ComposableNodeContainer(
                name="bev_processor_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    # BEV가 먼저 OAK를 단독으로 열어 roll·pitch와 자동 모드의
                    # 높이를 측정하고 장치를 닫은 뒤 camera_driver가 시작된다.
                    ComposableNode(
                        package="bev_processor",
                        plugin="bev_processor::BevProcessorNode",
                        name="bev_processor",
                        parameters=[
                            LaunchConfiguration("bev_params_file"),
                            {
                                "performance_measurement_enabled": (
                                    performance_measurement_parameter
                                ),
                            },
                            lane_parameter_overrides,
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                    # 안정화된 전체 NV12 영상을 BEV에 전달한다. BEV는
                    # 시작 측정 LUT를 유지하므로 IMU 토픽 발행은 필요 없다.
                    ComposableNode(
                        package="camera_driver",
                        plugin="camera_driver::CameraDriverNode",
                        name="camera_driver",
                        parameters=[
                            LaunchConfiguration("camera_params_file"),
                            {
                                "preview_enabled": False,
                                "publish_enabled": True,
                                "imu_bridge_enabled": False,
                                "imu_stabilization_enabled": True,
                                "output_crop_top_px": 0,
                                "performance_measurement_enabled": (
                                    performance_measurement_parameter
                                ),
                            },
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                ],
            ),
        ]
    )
