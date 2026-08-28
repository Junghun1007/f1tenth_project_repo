import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
    bev_params,
    launch_parameter_defaults,
):
    parameters = _merged_parameters(
        bev_params,
        LaunchConfiguration("bev_params_file").perform(context),
        "bev_processor",
    )
    for launch_name, parameter_name, fallback in launch_parameter_defaults:
        if (
            context.launch_configurations[launch_name]
            == _PARAMETER_FILE_DEFAULT
        ):
            context.launch_configurations[launch_name] = _launch_default(
                parameters, parameter_name, fallback
            )
    return []


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
    high_frequency_only = LaunchConfiguration(
        "imu_stabilization_high_frequency_only"
    )
    high_frequency_cutoff_hz = LaunchConfiguration(
        "imu_stabilization_high_frequency_vibration_cutoff_hz"
    )
    gyroscope_correction_gain = LaunchConfiguration(
        "imu_stabilization_gyroscope_correction_gain"
    )
    can_longitudinal_compensation_gain = LaunchConfiguration(
        "imu_stabilization_can_longitudinal_compensation_gain"
    )
    can_lateral_compensation_gain = LaunchConfiguration(
        "imu_stabilization_can_lateral_compensation_gain"
    )
    moving_accelerometer_nudge_strength = LaunchConfiguration(
        "imu_stabilization_moving_accelerometer_nudge_strength"
    )
    moving_gravity_anchor_maximum_correction_rate_degps = LaunchConfiguration(
        "imu_stabilization_moving_gravity_anchor_maximum_correction_rate_degps"
    )
    invalid_correction_hold_frames = LaunchConfiguration(
        "imu_stabilization_invalid_correction_hold_frames"
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
    high_frequency_only_parameter = ParameterValue(
        high_frequency_only,
        value_type=bool,
    )
    high_frequency_cutoff_parameter = ParameterValue(
        high_frequency_cutoff_hz,
        value_type=float,
    )
    gyroscope_correction_gain_parameter = ParameterValue(
        gyroscope_correction_gain,
        value_type=float,
    )
    can_longitudinal_compensation_parameter = ParameterValue(
        can_longitudinal_compensation_gain,
        value_type=float,
    )
    can_lateral_compensation_parameter = ParameterValue(
        can_lateral_compensation_gain,
        value_type=float,
    )
    moving_accelerometer_nudge_strength_parameter = ParameterValue(
        moving_accelerometer_nudge_strength,
        value_type=float,
    )
    moving_gravity_anchor_rate_parameter = ParameterValue(
        moving_gravity_anchor_maximum_correction_rate_degps,
        value_type=float,
    )
    invalid_correction_hold_frames_parameter = ParameterValue(
        invalid_correction_hold_frames,
        value_type=int,
    )

    # Keep every lane tuning value overridable from `ros2 launch ... name:=x`.
    # When omitted, each value is resolved from bev_params_file at launch time.
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
        ("lane_seed_roi_height_ratio", "0.25", float),
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
        ("lane_seed_pair_minimum_distance_px", "45.0", float),
        ("lane_seed_pair_maximum_distance_px", "100.0", float),
        ("lane_seed_sliding_window_enabled", "true", bool),
        (
            "lane_seed_sliding_window_minimum_seed_arc_length_px",
            "15.0",
            float,
        ),
        ("lane_seed_sliding_window_initial_width_px", "6", int),
        ("lane_seed_sliding_window_initial_height_px", "10", int),
        ("lane_seed_sliding_window_growth_ratio", "1.04", float),
        ("lane_seed_sliding_window_maximum_width_px", "25", int),
        ("lane_seed_sliding_window_maximum_height_px", "15", int),
        ("lane_seed_sliding_window_step_ratio", "0.60", float),
        ("lane_seed_sliding_window_maximum_count", "40", int),
        (
            "lane_seed_sliding_window_minimum_bright_pixels",
            "2",
            int,
        ),
        (
            "lane_seed_sliding_window_maximum_consecutive_misses",
            "2",
            int,
        ),
        (
            "lane_seed_sliding_window_maximum_turn_deg_per_window",
            "14.0",
            float,
        ),
        (
            "lane_seed_sliding_window_maximum_turn_change_deg_per_window",
            "3.0",
            float,
        ),
        (
            "lane_seed_sliding_window_heading_update_gain",
            "0.90",
            float,
        ),
        ("lane_centerline_enabled", "true", bool),
        ("lane_centerline_expected_width_m", "0.65", float),
        ("lane_centerline_width_tolerance_m", "0.08", float),
        ("lane_centerline_minimum_points", "6", int),
        (
            "lane_centerline_minimum_counterpart_points",
            "3",
            int,
        ),
        (
            "lane_centerline_measured_point_smoothing_weight",
            "0.70",
            float,
        ),
        (
            "lane_centerline_midpoint_smoothing_weight",
            "0.45",
            float,
        ),
        (
            "lane_centerline_temporal_current_weight",
            "0.60",
            float,
        ),
        (
            "lane_centerline_transition_maximum_correction_m",
            "0.15",
            float,
        ),
        (
            "lane_centerline_transition_correction_decay",
            "0.70",
            float,
        ),
        ("lane_centerline_tangent_window_m", "0.12", float),
        ("lane_centerline_maximum_curvature_per_m", "1.8", float),
        ("lane_centerline_maximum_heading_step_deg", "14.0", float),
        ("lane_centerline_maximum_gap_fill_m", "0.30", float),
        (
            "lane_centerline_corner_longer_boundary_enabled",
            "true",
            bool,
        ),
        (
            "lane_centerline_corner_outward_bias_m",
            "0.05",
            float,
        ),
        (
            "lane_centerline_corner_enter_heading_change_deg",
            "40.0",
            float,
        ),
        (
            "lane_centerline_corner_exit_heading_change_deg",
            "20.0",
            float,
        ),
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
        ("lane_seed_temporal_side_lock_reset_frames", "100", int),
        (
            "lane_seed_temporal_side_reacquire_base_distance_px",
            "45.0",
            float,
        ),
        (
            "lane_seed_temporal_side_reacquire_distance_per_missing_frame_px",
            "3.0",
            float,
        ),
        (
            "lane_seed_temporal_side_reacquire_maximum_distance_px",
            "63.0",
            float,
        ),
    ]
    launch_parameter_defaults = [
        (
            "performance_measurement_enabled",
            "performance_measurement_enabled",
            "false",
        ),
        ("bev_input_bottom_fraction", "input_bottom_fraction", "0.70"),
        ("preview_enabled", "preview_enabled", "true"),
        ("bev_interpolation", "bev_interpolation", "bilinear"),
        *[(name, name, fallback) for name, fallback, _ in lane_parameters],
    ]
    lane_launch_arguments = [
        DeclareLaunchArgument(
            name,
            default_value=_PARAMETER_FILE_DEFAULT,
            description=f"Override bev_processor parameter '{name}'.",
        )
        for name, _, _ in lane_parameters
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
                default_value=_PARAMETER_FILE_DEFAULT,
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
                "imu_stabilization_high_frequency_only",
                default_value="false",
                description=(
                    "Disable CAN and moving accelerometer corrections, "
                    "leaving gyro high-frequency stabilization only."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_high_frequency_vibration_cutoff_hz",
                default_value="3.0",
                description="High-frequency-only gyro cutoff in Hz.",
            ),
            DeclareLaunchArgument(
                "imu_stabilization_gyroscope_correction_gain",
                default_value="1.0",
                description=(
                    "Applied gyro image correction fraction from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_can_longitudinal_compensation_gain",
                default_value="1.0",
                description=(
                    "Fraction of CAN longitudinal acceleration removed "
                    "from the camera accelerometer, from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_can_lateral_compensation_gain",
                default_value="1.0",
                description=(
                    "Fraction of CAN lateral acceleration removed from "
                    "the camera accelerometer, from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_moving_accelerometer_nudge_strength",
                default_value="0.15",
                description=(
                    "Strength of the bounded residual-accelerometer image "
                    "correction, from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                (
                    "imu_stabilization_moving_gravity_anchor_maximum_"
                    "correction_rate_degps"
                ),
                default_value="0.50",
                description=(
                    "Maximum persistent CAN-compensated gravity anchor "
                    "correction rate in degrees per second."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_invalid_correction_hold_frames",
                default_value="2",
                description=(
                    "Reuse the last valid stabilization homography for this "
                    "many consecutive RGB/IMU matching misses."
                ),
            ),
            DeclareLaunchArgument(
                "bev_input_bottom_fraction",
                default_value=_PARAMETER_FILE_DEFAULT,
                description=(
                    "Bottom fraction of the rectified camera frame sent to "
                    "the fused CUDA stabilization/BEV path."
                ),
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value=_PARAMETER_FILE_DEFAULT,
                description="Show or completely disable the BEV GUI preview.",
            ),
            DeclareLaunchArgument(
                "bev_interpolation",
                default_value=_PARAMETER_FILE_DEFAULT,
                description="Interpolation used by the CUDA NV12-to-BEV warp.",
            ),
            *lane_launch_arguments,
            OpaqueFunction(
                function=_apply_parameter_file_defaults,
                kwargs={
                    "bev_params": bev_params,
                    "launch_parameter_defaults": launch_parameter_defaults,
                },
            ),
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
                            bev_params,
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
                                # CAN dynamics uses this yaw rate for ay.
                                "imu_bridge_enabled": True,
                                "imu_stabilization_enabled": (
                                    imu_stabilization_parameter
                                ),
                                "imu_stabilization_high_frequency_only": (
                                    high_frequency_only_parameter
                                ),
                                (
                                    "imu_stabilization_high_frequency_"
                                    "vibration_cutoff_hz"
                                ): high_frequency_cutoff_parameter,
                                (
                                    "imu_stabilization_gyroscope_"
                                    "correction_gain"
                                ): gyroscope_correction_gain_parameter,
                                (
                                    "imu_stabilization_can_longitudinal_"
                                    "compensation_gain"
                                ): can_longitudinal_compensation_parameter,
                                (
                                    "imu_stabilization_can_lateral_"
                                    "compensation_gain"
                                ): can_lateral_compensation_parameter,
                                (
                                    "imu_stabilization_moving_"
                                    "accelerometer_nudge_strength"
                                ): (
                                    moving_accelerometer_nudge_strength_parameter
                                ),
                                (
                                    "imu_stabilization_moving_gravity_anchor_"
                                    "maximum_correction_rate_degps"
                                ): moving_gravity_anchor_rate_parameter,
                                (
                                    "imu_stabilization_invalid_correction_"
                                    "hold_frames"
                                ): invalid_correction_hold_frames_parameter,
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
