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
    imu_stabilization_enabled = LaunchConfiguration(
        "imu_stabilization_enabled"
    )
    bev_input_bottom_fraction = LaunchConfiguration(
        "bev_input_bottom_fraction"
    )
    performance_measurement_parameter = ParameterValue(
        performance_measurement_enabled,
        value_type=bool,
    )
    imu_stabilization_parameter = ParameterValue(
        imu_stabilization_enabled,
        value_type=bool,
    )

    # Keep every lane tuning value overridable from `ros2 launch ... name:=x`.
    # Defaults below mirror the documented YAML baseline. They are explicit
    # launch-time overrides so field tuning never requires rebuilding.
    lane_parameters = [
        ("lane_reconstruction_enabled", "true", bool),
        ("lane_output_topic", "/camera/image_bev_lane", str),
        ("lane_preview_enabled", "true", bool),
        ("lane_preview_result_only_enabled", "false", bool),
        ("lane_preview_sliding_windows_enabled", "false", bool),
        ("preview_x_origin_m", "0.02", float),
        ("lane_output_lateral_margin_m", "0.70", float),
        ("lane_preview_overlay_alpha", "0.8", float),
        ("lane_minimum_brightness", "160", int),
        ("lane_far_minimum_brightness", "110", int),
        ("lane_maximum_saturation", "80", int),
        ("lane_brightness_blur_kernel", "1", int),
        ("lane_vertical_close_m", "0.05", float),
        ("lane_minimum_mark_width_m", "0.015", float),
        ("lane_maximum_mark_width_m", "0.030", float),
        ("lane_minimum_local_contrast", "55", int),
        ("lane_maximum_local_background_brightness", "140", int),
        ("lane_local_background_band_m", "0.04", float),
        ("lane_tracked_mark_width_near_m", "0.11", float),
        ("lane_tracked_mark_width_far_m", "0.22", float),
        ("lane_measurement_lateral_gate_near_m", "0.05", float),
        ("lane_measurement_lateral_gate_far_m", "0.10", float),
        ("lane_row_step_px", "2", int),
        ("lane_observation_minimum_x_m", "0.20", float),
        ("lane_observation_maximum_x_m", "1.30", float),
        ("lane_reconstruction_minimum_x_m", "0.10", float),
        ("lane_reconstruction_maximum_x_m", "3.0", float),
        ("lane_maximum_extrapolation_m", "0.0", float),
        ("lane_sliding_window_step_m", "0.04", float),
        ("lane_sliding_window_length_m", "0.18", float),
        ("lane_sliding_window_half_width_near_m", "0.08", float),
        ("lane_sliding_window_half_width_far_m", "0.14", float),
        ("lane_sliding_window_measurement_weight", "0.88", float),
        ("lane_sliding_window_heading_weight", "0.50", float),
        ("lane_maximum_tracking_arc_length_m", "3.20", float),
        ("lane_maximum_gap_fill_m", "0.30", float),
        ("lane_measured_point_smoothing_weight", "0.60", float),
        ("lane_minimum_window_pixel_count", "6", int),
        ("lane_expected_width_m", "0.65", float),
        ("lane_width_tolerance_m", "0.08", float),
        ("lane_initial_center_tolerance_m", "0.60", float),
        ("lane_single_initial_tolerance_m", "0.20", float),
        ("lane_maximum_tracking_gap_m", "0.16", float),
        ("lane_minimum_points", "6", int),
        ("lane_minimum_counterpart_points", "3", int),
        ("lane_allow_single_lane", "true", bool),
        ("lane_centerline_from_single_boundary_enabled", "true", bool),
        ("lane_centerline_preserve_reference_shape", "false", bool),
        ("lane_centerline_midpoint_smoothing_weight", "0.45", float),
        ("lane_centerline_temporal_current_weight", "0.60", float),
        ("lane_centerline_transition_maximum_correction_m", "0.15", float),
        ("lane_centerline_transition_correction_decay", "0.70", float),
        ("lane_centerline_tangent_window_m", "0.20", float),
        ("lane_centerline_maximum_curvature_per_m", "1.25", float),
        ("lane_centerline_maximum_heading_step_deg", "8.0", float),
        ("lane_temporal_tracking_enabled", "false", bool),
        ("lane_temporal_maximum_lateral_jump_near_m", "0.06", float),
        ("lane_temporal_maximum_lateral_jump_far_m", "0.12", float),
        ("lane_temporal_maximum_heading_jump_deg", "15.0", float),
        ("lane_temporal_confirmation_frames", "4", int),
        ("lane_temporal_hold_frames", "0", int),
        ("lane_output_line_thickness_m", "0.02", float),
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
            DeclareLaunchArgument(
                "imu_stabilization_enabled",
                default_value="true",
                description=(
                    "Apply OAK IMU pitch/roll stabilization. The camera's "
                    "fixed_view_zoom remains active when this is disabled."
                ),
            ),
            DeclareLaunchArgument(
                "bev_input_bottom_fraction",
                default_value="0.70",
                description=(
                    "Bottom fraction of the rectified camera frame sent to "
                    "the fused CUDA stabilization/BEV path."
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
                                "input_bottom_fraction": ParameterValue(
                                    bev_input_bottom_fraction,
                                    value_type=float,
                                ),
                            },
                            lane_parameter_overrides,
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                    # 하단 raw NV12와 보정 행렬을 전달한다. CUDA가 안정화와
                    # BEV를 한 번에 수행하므로 전체 CPU warp는 실행하지 않는다.
                    ComposableNode(
                        package="camera_driver",
                        plugin="camera_driver::CameraDriverNode",
                        name="camera_driver",
                        parameters=[
                            LaunchConfiguration("camera_params_file"),
                            {
                                "preview_enabled": False,
                                "publish_enabled": False,
                                "fused_bev_output_enabled": True,
                                "bev_input_bottom_fraction": ParameterValue(
                                    bev_input_bottom_fraction,
                                    value_type=float,
                                ),
                                "imu_bridge_enabled": False,
                                "imu_stabilization_enabled": (
                                    imu_stabilization_parameter
                                ),
                                (
                                    "imu_stabilization_external_reference_required"
                                ): True,
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
