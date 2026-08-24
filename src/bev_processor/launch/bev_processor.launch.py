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
    high_frequency_vibration_only_enabled = LaunchConfiguration(
        "imu_stabilization_high_frequency_vibration_only_enabled"
    )
    high_frequency_vibration_cutoff_hz = LaunchConfiguration(
        "imu_stabilization_high_frequency_vibration_cutoff_hz"
    )
    can_low_frequency_compensation_enabled = LaunchConfiguration(
        "imu_stabilization_can_low_frequency_compensation_enabled"
    )
    low_frequency_cutoff_hz = LaunchConfiguration(
        "imu_stabilization_low_frequency_cutoff_hz"
    )
    low_frequency_correction_gain = LaunchConfiguration(
        "imu_stabilization_low_frequency_correction_gain"
    )
    low_frequency_maximum_correction_deg = LaunchConfiguration(
        "imu_stabilization_low_frequency_maximum_correction_deg"
    )
    bev_input_bottom_fraction = LaunchConfiguration(
        "bev_input_bottom_fraction"
    )
    preview_enabled = LaunchConfiguration("preview_enabled")
    bev_interpolation = LaunchConfiguration("bev_interpolation")
    performance_measurement_parameter = ParameterValue(
        performance_measurement_enabled,
        value_type=bool,
    )
    imu_stabilization_parameter = ParameterValue(
        imu_stabilization_enabled,
        value_type=bool,
    )
    high_frequency_vibration_only_parameter = ParameterValue(
        high_frequency_vibration_only_enabled,
        value_type=bool,
    )
    high_frequency_vibration_cutoff_parameter = ParameterValue(
        high_frequency_vibration_cutoff_hz,
        value_type=float,
    )
    can_low_frequency_compensation_parameter = ParameterValue(
        can_low_frequency_compensation_enabled,
        value_type=bool,
    )
    low_frequency_cutoff_parameter = ParameterValue(
        low_frequency_cutoff_hz,
        value_type=float,
    )
    low_frequency_correction_gain_parameter = ParameterValue(
        low_frequency_correction_gain,
        value_type=float,
    )
    low_frequency_maximum_correction_parameter = ParameterValue(
        low_frequency_maximum_correction_deg,
        value_type=float,
    )

    # Keep every lane tuning value overridable from `ros2 launch ... name:=x`.
    # Defaults below mirror the documented YAML baseline. They are explicit
    # launch-time overrides so field tuning never requires rebuilding.
    lane_parameters = [
        ("capture_directory", ".", str),
        ("lane_seed_detection_enabled", "true", bool),
        ("lane_output_topic", "/camera/image_bev_lane", str),
        ("lane_preview_enabled", "true", bool),
        ("lane_gray_mode", "0", int),
        ("lane_top_hat_shape", "1", int),
        ("lane_top_hat_iterations", "1", int),
        ("lane_top_hat_border", "0", int),
        ("lane_near_ratio", "0.45", float),
        ("lane_near_gain", "1.5", float),
        ("lane_near_noise_floor", "17", int),
        ("lane_near_kernel_width", "7", int),
        ("lane_near_kernel_height", "7", int),
        ("lane_middle_ratio", "0.35", float),
        ("lane_middle_gain", "1.6", float),
        ("lane_middle_noise_floor", "13", int),
        ("lane_middle_kernel_width", "17", int),
        ("lane_middle_kernel_height", "17", int),
        ("lane_far_ratio", "0.20", float),
        ("lane_far_gain", "1.65", float),
        ("lane_far_noise_floor", "11", int),
        ("lane_far_kernel_width", "27", int),
        ("lane_far_kernel_height", "27", int),
        ("lane_seed_roi_bottom_exclusion_ratio", "0.09", float),
        ("lane_seed_roi_height_ratio", "0.40", float),
        ("lane_seed_minimum_response", "30", int),
        ("lane_seed_minimum_run_width_px", "2", int),
        ("lane_seed_maximum_run_width_px", "8", int),
        ("lane_seed_maximum_lateral_step_px", "4.0", float),
        ("lane_seed_maximum_gap_rows", "4", int),
        ("lane_seed_minimum_track_arc_length_px", "20.0", float),
        ("lane_seed_minimum_bilateral_contrast", "25.0", float),
        ("lane_seed_maximum_background_asymmetry", "50.0", float),
        ("lane_seed_background_gap_px", "1", int),
        ("lane_seed_background_band_width_px", "5", int),
        ("lane_seed_contrast_score_weight", "0.30", float),
        ("lane_seed_contrast_relaxation_enabled", "true", bool),
        ("lane_seed_contrast_relaxation_step", "5.0", float),
        ("lane_seed_contrast_relaxation_retries", "5", int),
        ("lane_seed_slope_filter_enabled", "true", bool),
        ("lane_seed_slope_median_window", "5", int),
        ("lane_seed_maximum_slope_change_px_per_row", "2.0", float),
        ("lane_seed_pair_minimum_distance_px", "50.0", float),
        ("lane_seed_pair_maximum_distance_px", "95.0", float),
        ("lane_seed_column_tracking_enabled", "true", bool),
        ("lane_seed_cross_direction_merge_enabled", "true", bool),
        (
            "lane_seed_cross_direction_merge_maximum_endpoint_distance_px",
            "3.0",
            float,
        ),
        (
            "lane_seed_cross_direction_merge_minimum_connector_support_ratio",
            "0.70",
            float,
        ),
        (
            "lane_seed_cross_direction_merge_maximum_turn_angle_deg",
            "110.0",
            float,
        ),
        ("lane_seed_temporal_side_lock_enabled", "true", bool),
        ("lane_seed_temporal_side_lock_reset_frames", "30", int),
        (
            "lane_seed_temporal_side_reacquire_maximum_distance_px",
            "20.0",
            float,
        ),
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
                "imu_stabilization_high_frequency_vibration_only_enabled",
                default_value="false",
                description=(
                    "Pass low-frequency attitude through and correct only "
                    "high-frequency camera vibration."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_high_frequency_vibration_cutoff_hz",
                default_value="3.0",
                description="High-frequency correction cutoff in Hz.",
            ),
            DeclareLaunchArgument(
                "imu_stabilization_can_low_frequency_compensation_enabled",
                default_value="false",
                description=(
                    "Use CAN vehicle acceleration for low-frequency tilt "
                    "correction."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_low_frequency_cutoff_hz",
                default_value="1.0",
                description="CAN-compensated low-frequency cutoff in Hz.",
            ),
            DeclareLaunchArgument(
                "imu_stabilization_low_frequency_correction_gain",
                default_value="0.5",
                description="Low-frequency correction strength from 0 to 1.",
            ),
            DeclareLaunchArgument(
                "imu_stabilization_low_frequency_maximum_correction_deg",
                default_value="1.0",
                description="Maximum CAN low-frequency correction angle.",
            ),
            DeclareLaunchArgument(
                "bev_input_bottom_fraction",
                default_value="0.70",
                description=(
                    "Bottom fraction of the rectified camera frame sent to "
                    "the fused CUDA stabilization/BEV path."
                ),
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value="true",
                description="Show or completely disable the BEV GUI preview.",
            ),
            DeclareLaunchArgument(
                "bev_interpolation",
                default_value="bilinear",
                choices=["bilinear", "bicubic", "adaptive"],
                description="Interpolation used by the CUDA NV12-to-BEV warp.",
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
                                "preview_enabled": ParameterValue(
                                    preview_enabled,
                                    value_type=bool,
                                ),
                                "input_bottom_fraction": ParameterValue(
                                    bev_input_bottom_fraction,
                                    value_type=float,
                                ),
                                "bev_interpolation": ParameterValue(
                                    bev_interpolation,
                                    value_type=str,
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
                                "imu_bridge_enabled": (
                                    can_low_frequency_compensation_parameter
                                ),
                                "imu_stabilization_enabled": (
                                    imu_stabilization_parameter
                                ),
                                (
                                    "imu_stabilization_high_frequency_"
                                    "vibration_only_enabled"
                                ): high_frequency_vibration_only_parameter,
                                (
                                    "imu_stabilization_high_frequency_"
                                    "vibration_cutoff_hz"
                                ): high_frequency_vibration_cutoff_parameter,
                                (
                                    "imu_stabilization_can_low_frequency_"
                                    "compensation_enabled"
                                ): can_low_frequency_compensation_parameter,
                                (
                                    "imu_stabilization_low_frequency_"
                                    "cutoff_hz"
                                ): low_frequency_cutoff_parameter,
                                (
                                    "imu_stabilization_low_frequency_"
                                    "correction_gain"
                                ): low_frequency_correction_gain_parameter,
                                (
                                    "imu_stabilization_low_frequency_"
                                    "maximum_correction_deg"
                                ): low_frequency_maximum_correction_parameter,
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
