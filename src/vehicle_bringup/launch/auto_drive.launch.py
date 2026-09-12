import os
import math

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
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
    bev_config,
    auto_control_config,
    bev_arguments,
    controller_arguments,
    line_detactor_config,
    bev_launch,
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
    detector = _merged_parameters(
        line_detactor_config,
        LaunchConfiguration("line_detactor_params_file").perform(context),
        "line_detactor",
    )
    context.launch_configurations["ml_engine_precision"] = str(
        detector.get("engine_precision", "fp32")
    )
    context.launch_configurations["ml_model_path"] = str(
        detector.get("model_path", os.path.join(
            get_package_share_directory("line_detactor"), "models",
            "fast_scnn_stop_line_120x300_batch_1.onnx",
        ))
    )
    # One physical geometry and result topic for producer and consumer. Old BEV
    # lane parameters in external YAML cannot reactivate the removed detector.
    x_min, x_max = float(bev_defaults["x_min_m"]), float(bev_defaults["x_max_m"])
    y_min, y_max = float(bev_defaults["y_min_m"]), float(bev_defaults["y_max_m"])
    mpp = float(bev_defaults["meter_per_pixel"])
    if not all(math.isfinite(v) for v in (x_min, x_max, y_min, y_max, mpp)) or mpp <= 0:
        raise RuntimeError("BEV geometry must be finite with positive meter_per_pixel")
    if abs(x_min) > 1e-6 or not math.isclose(y_min, -y_max, abs_tol=1e-6) or min(x_max, y_max) <= 0:
        raise RuntimeError("ML auto drive requires front-axle BEV: x_min=0, symmetric +/-Y")
    width, height = int(bev_defaults["output_width"]), int(bev_defaults["output_height"])
    if width != round((y_max-y_min)/mpp) or height != round((x_max-x_min)/mpp):
        raise RuntimeError("BEV output size and metric extent disagree")
    if width != int(detector["model_input_width"]) or height != int(detector["model_input_height"]):
        raise RuntimeError("BEV output size must match the static line_detactor model input")
    for required in ("connection_enabled", "centerline_enabled", "result_publish_enabled"):
        if detector.get(required) is not True:
            raise RuntimeError(f"ML auto drive requires line_detactor {required}: true")
    detector.update({
        "input_topic": str(bev_defaults["output_topic"]),
        "direct_bev_input_enabled": True,
        "centerline_bev_width_m": y_max-y_min,
        "centerline_bev_height_m": x_max-x_min,
    })
    context.launch_configurations["ml_lane_result_topic"] = str(detector["result_topic"])
    context.launch_configurations["ml_lane_frame_id"] = str(bev_defaults["output_frame_id"])
    context.launch_configurations["ml_bev_x_max_m"] = str(x_max)
    context.launch_configurations["ml_bev_y_max_m"] = str(y_max)
    context.launch_configurations["ml_bev_meter_per_pixel"] = str(mpp)
    if (
        context.launch_configurations["preview_enabled"]
        == _PARAMETER_FILE_DEFAULT
    ):
        context.launch_configurations["preview_enabled"] = _launch_default(
            detector, "preview_enabled", "true"
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
    detector["control_latency_topic"] = context.launch_configurations[
        "input_to_control_decision_topic"
    ]
    preview = context.launch_configurations["preview_enabled"].lower()
    if preview not in ("true", "false"):
        raise RuntimeError("preview_enabled must be true or false")
    detector["preview_enabled"] = preview == "true"
    measurement = LaunchConfiguration(
        "performance_measurement_enabled"
    ).perform(context).lower()
    if measurement not in ("true", "false"):
        raise RuntimeError("performance_measurement_enabled must be true or false")
    if measurement == "true":
        # GUI work would contaminate engine comparisons and is unnecessary for
        # the fixed-duration, self-terminating measurement.
        detector["preview_enabled"] = False
        preview = "false"
    result_only = LaunchConfiguration("preview_result_only_enabled").perform(context)
    if result_only != _PARAMETER_FILE_DEFAULT:
        if result_only.lower() not in ("true", "false"):
            raise RuntimeError("preview_result_only_enabled must be true or false")
        detector["preview_result_only_enabled"] = result_only.lower() == "true"
    for parameter_name in (
        "centerline_corner_outward_offset_m",
        "centerline_corner_entry_distance_m",
    ):
        value = LaunchConfiguration(parameter_name).perform(context)
        if value != _PARAMETER_FILE_DEFAULT:
            detector[parameter_name] = float(value)
    return [LogInfo(msg=(
        "[ML auto drive] launch=" + os.path.realpath(__file__) +
        " | BEV=" + LaunchConfiguration("bev_params_file").perform(context) +
        " | detector_yaml=" + LaunchConfiguration("line_detactor_params_file").perform(context) +
        " | model=" + str(detector.get("model_path", os.path.join(
            get_package_share_directory("line_detactor"), "models",
            "fast_scnn_stop_line_120x300_batch_1.onnx"))) +
        " | pipeline=direct BEV memory -> " + str(detector["result_topic"]) +
        " -> auto_control | ML preview=" + preview + " | raw BEV preview=false"
        " | performance measurement=" + measurement
    )), bev_launch, LoadComposableNodes(
        target_container="/bev_processor_container",
        composable_node_descriptions=[ComposableNode(
            package="line_detactor", plugin="line_detactor::LineDetactorNode",
            name="line_detactor", parameters=[detector],
            extra_arguments=[{"use_intra_process_comms": True}],
        )],
    )]


def generate_launch_description():
    bev_share = get_package_share_directory("bev_processor")
    auto_control_share = get_package_share_directory("auto_control")
    vehicle_bringup_share = get_package_share_directory("vehicle_bringup")
    bev_config = os.path.join(bev_share, "config", "bev_config.yaml")
    line_detactor_config = os.path.join(get_package_share_directory("line_detactor"), "config", "line_detactor.yaml")
    camera_config = os.path.join(get_package_share_directory("camera_driver"), "config", "camera_config.yaml")
    dynamics_config = os.path.join(get_package_share_directory("vehicle_dynamics_monitor"), "config", "vehicle_dynamics.yaml")
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
    performance_measurement_enabled = LaunchConfiguration(
        "performance_measurement_enabled"
    )
    bev_argument_fallbacks = [
        ("dataset_collection_enabled", "false"),
        ("dataset_collection_root_directory", "datasets"),
        ("dataset_collection_fps", "10.0"),
        ("dataset_collection_target_count", "1000"),
        ("dataset_collection_stop_auto_on_complete", "true"),
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
        ("auto_control_mode", "drive", "control_mode", str),
        (
            "input_to_control_decision_topic",
            "/auto/detector_input_to_control_decision_ms",
            "input_to_control_decision_topic",
            str,
        ),
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
        ("path_maximum_gap_m", "0.15", "path_maximum_gap_m", float),
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
            "camera_params_file": LaunchConfiguration("camera_params_file"),
            "preview_enabled": "false",
            "publish_enabled": "false",
            "direct_output_enabled": "true",
            "performance_measurement_enabled": performance_measurement_enabled,
            **bev_overrides,
        }.items(),
    )

    controller_overrides.update({
        "lane_result_topic": LaunchConfiguration("ml_lane_result_topic"),
        "lane_result_frame_id": LaunchConfiguration("ml_lane_frame_id"),
        "bev_x_max_m": ParameterValue(LaunchConfiguration("ml_bev_x_max_m"), value_type=float),
        "bev_y_max_m": ParameterValue(LaunchConfiguration("ml_bev_y_max_m"), value_type=float),
        "bev_meter_per_pixel": ParameterValue(LaunchConfiguration("ml_bev_meter_per_pixel"), value_type=float),
        "performance_measurement_enabled": ParameterValue(
            performance_measurement_enabled, value_type=bool
        ),
        "performance_measurement_duration_sec": ParameterValue(
            LaunchConfiguration("performance_measurement_duration_sec"),
            value_type=float,
        ),
        "performance_measurement_startup_timeout_sec": ParameterValue(
            LaunchConfiguration("performance_measurement_startup_timeout_sec"),
            value_type=float,
        ),
        "performance_measurement_power_sample_interval_sec": ParameterValue(
            LaunchConfiguration("performance_measurement_power_sample_interval_sec"),
            value_type=float,
        ),
        "performance_measurement_log_directory": LaunchConfiguration(
            "performance_measurement_log_directory"
        ),
        "performance_measurement_engine_precision": LaunchConfiguration(
            "ml_engine_precision"
        ),
        "performance_measurement_model_path": LaunchConfiguration(
            "ml_model_path"
        ),
    })
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
    performance_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=auto_control_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="auto-drive performance measurement completed"
                    )
                )
            ],
        ),
        condition=IfCondition(performance_measurement_enabled),
    )

    vesc_bridge_node = Node(
        package="vesc_bridge",
        executable="vesc_bridge_node",
        name="vesc_bridge_node",
        output="screen",
        parameters=[vesc_config, {"port": vesc_port}],
    )

    dynamics_node = Node(
        package="vehicle_dynamics_monitor", executable="vehicle_dynamics_node",
        name="vehicle_dynamics_node", output="screen",
        parameters=[dynamics_config, {
            "input_mode": LaunchConfiguration("input_mode"),
            "can_interface": LaunchConfiguration("can_interface"),
            "slcan_channel": LaunchConfiguration("slcan_channel"),
            "slcan_bitrate": ParameterValue(LaunchConfiguration("slcan_bitrate"), value_type=int),
            "can_controller_id": ParameterValue(LaunchConfiguration("can_controller_id"), value_type=int),
            "commanded_duty_topic": "/auto/current_duty",
        }],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument("camera_params_file", default_value=camera_config),
            DeclareLaunchArgument("line_detactor_params_file", default_value=line_detactor_config,
                                  description="ML lane/centerline YAML; source geometry follows BEV YAML"),
            DeclareLaunchArgument("preview_result_only_enabled", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Show lanes/path on black background; omitted uses ML YAML"),
            DeclareLaunchArgument("centerline_corner_outward_offset_m", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Corner path shift toward observed outer lane in metres; 0 disables"),
            DeclareLaunchArgument("centerline_corner_entry_distance_m", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Extend outward shift before observed corners in metres; 0 uses local evidence only"),
            DeclareLaunchArgument("input_mode", default_value="ros_topic"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("slcan_channel", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("slcan_bitrate", default_value="500000"),
            DeclareLaunchArgument("can_controller_id", default_value="0"),
            DeclareLaunchArgument(
                "performance_measurement_enabled",
                default_value="false",
                description=(
                    "Measure after the first valid ML centerline, write JSON, "
                    "and shut down; forces monitor_only and disables previews"
                ),
            ),
            DeclareLaunchArgument(
                "performance_measurement_duration_sec",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "performance_measurement_startup_timeout_sec",
                default_value="300.0",
            ),
            DeclareLaunchArgument(
                "performance_measurement_power_sample_interval_sec",
                default_value="0.1",
            ),
            DeclareLaunchArgument(
                "performance_measurement_log_directory",
                default_value=os.path.join(os.getcwd(), "performance_logs"),
            ),
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
                    "Show ML lanes and yellow centerline preview. Set false for no GUI."
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
                    "line_detactor_config": line_detactor_config,
                    "bev_launch": bev_launch,
                },
            ),
            vesc_bridge_node,
            dynamics_node,
            performance_shutdown_handler,
            auto_control_node,
        ]
    )
