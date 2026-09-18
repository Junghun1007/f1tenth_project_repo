"""Manual control + existing 120x300 lane BEV + in-process depth obstacles."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def _parameters(path, node):
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or node not in document:
        raise ValueError(f"{path}: missing {node}.ros__parameters")
    section = document[node]
    values = section.get("ros__parameters") if isinstance(section, dict) else None
    if not isinstance(values, dict):
        raise ValueError(f"{path}: invalid {node}.ros__parameters")
    return values


def _typed(values):
    # Preserve strings such as 'off'; launch otherwise reinterprets YAML scalars.
    return {key: ParameterValue(value, value_type=str) if isinstance(value, str) else value
            for key, value in values.items()}


def _start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        text = value(name).lower()
        if text not in ("true", "false"):
            raise ValueError(f"{name}: true / false")
        return text == "true"

    def path(name):
        return Path(value(name)).expanduser().resolve(strict=True)

    experiment = path("params_file")
    camera = _parameters(path("camera_params_file"), "camera_driver")
    bev = _parameters(path("bev_params_file"), "bev_processor")
    lane = _parameters(path("lane_params_file"), "line_detactor")
    for node, target in (("camera_driver", camera), ("bev_processor", bev), ("line_detactor", lane)):
        target.update(_parameters(experiment, node))

    # Retain the trained lane model's geometry; fail visibly on conflicting YAML.
    expected = dict(x_min_m=0.0, x_max_m=3.0, y_min_m=-0.6, y_max_m=0.6,
                    meter_per_pixel=0.01, output_width=120, output_height=300)
    for key, wanted in expected.items():
        if bev.get(key) != wanted:
            raise ValueError(f"Manual obstacle view requires bev_processor.{key}={wanted}")
    if lane.get("model_input_width", 120) != 120 or lane.get("model_input_height", 300) != 300:
        raise ValueError("Lane model input must remain 120x300")
    if camera.get("camera_socket", "CAM_A") != "CAM_A":
        raise ValueError("Shared RGB/stereo input requires camera_socket: CAM_A")
    if not camera.get("undistort_enabled", True):
        raise ValueError("Shared BEV geometry requires undistort_enabled: true")
    if bool(camera.get("obstacles.depth.enabled", False)) != bool(bev.get("obstacles.enabled", False)):
        raise ValueError("Enable/disable camera obstacles.depth.enabled and bev obstacles.enabled together")
    if lane.get("obstacles.overlay_enabled", False) and not bev.get("obstacles.enabled", False):
        raise ValueError("Obstacle overlay requires obstacles.enabled")

    ns = value("vehicle_namespace").strip("/")
    prefix = f"/{ns}" if ns else ""
    camera.update({
        "enabled": True, "preview_enabled": False, "publish_enabled": False,
        "fused_bev_output_enabled": True, "fused_bev_topic": bev["input_topic"],
        "bev_input_bottom_fraction": bev["input_bottom_fraction"], "output_crop_top_px": 0,
        "imu_bridge_enabled": True, "imu_stabilization_external_reference_required": True,
        "imu_stabilization_external_reference_topic": bev["startup_ground_reference_topic"],
        "imu_stabilization_measured_erpm_topic": f"{prefix}/vesc/measured_erpm",
        "imu_stabilization_can_acceleration_topic": f"{prefix}/vehicle/dynamics/acceleration",
        "capture_joy_topic": f"{prefix}/joy", "performance_measurement_enabled": False,
    })
    bev.update({
        "preview_enabled": False, "publish_enabled": False,
        "direct_output_enabled": True, "direct_host_copy_enabled": False,
        "dataset_collection_enabled": False, "dataset_collection_manual_capture_mode": False,
        "dataset_collection_stop_auto_on_complete": False,
        "capture_joy_topic": f"{prefix}/joy", "performance_measurement_enabled": False,
    })
    lane.update({
        "direct_bev_input_enabled": True, "preview_enabled": enabled("gui"),
        "preview_result_only_enabled": True, "bev_theme_enable": False,
        "connection_enabled": True, "result_publish_enabled": True,
        "centerline_bev_width_m": 1.2, "centerline_bev_height_m": 3.0,
        "preview_window_name": "Manual drive | lanes + depth obstacles (120 x 300 cm)",
    })
    share = Path(get_package_share_directory("vehicle_bringup"))
    actions = [LogInfo(msg=f"Manual obstacle YAML: {experiment}; BEV=120x300cm; auto_control is not loaded")]
    if enabled("manual_enabled"):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/manual_drive.launch.py")),
            launch_arguments={name: value(name) for name in
                              ("vehicle_namespace", "vesc_port", "controller_name_contains")}.items()))
    if enabled("dynamics_enabled"):
        dynamics = Path(get_package_share_directory("vehicle_dynamics_monitor"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(dynamics / "launch/vehicle_dynamics_monitor.launch.py")),
            launch_arguments={"vehicle_namespace": value("vehicle_namespace"), "input_mode": "ros_topic"}.items()))
    # Sequential component loading: one-shot measurement closes the OAK first;
    # camera_driver then opens that same ID with RGB and stereo in one pipeline.
    actions.append(ComposableNodeContainer(
        name="manual_obstacle_container", namespace="", package="rclcpp_components",
        executable="component_container_mt", output="screen",
        composable_node_descriptions=[
            ComposableNode(package=package, plugin=plugin, name=name, parameters=[_typed(parameters)],
                           extra_arguments=[{"use_intra_process_comms": True}])
            for package, plugin, name, parameters in (
                ("bev_processor", "bev_processor::BevProcessorNode", "bev_processor", bev),
                ("camera_driver", "camera_driver::CameraDriverNode", "camera_driver", camera),
                ("line_detactor", "line_detactor::LineDetactorNode", "line_detactor", lane),
            )]))
    return actions


def generate_launch_description():
    def share(package, relative):
        return str(Path(get_package_share_directory(package)) / relative)

    defaults = {
        "params_file": share("vehicle_bringup", "config/manual_obstacle_view.yaml"),
        "camera_params_file": share("camera_driver", "config/camera_config.yaml"),
        "bev_params_file": share("bev_processor", "config/bev_config.yaml"),
        "lane_params_file": share("line_detactor", "config/line_detactor.yaml"),
        "vehicle_namespace": "autopilot03", "vesc_port": "/dev/ttyTHS1",
        "controller_name_contains": "8BitDo", "manual_enabled": "true",
        "dynamics_enabled": "true", "gui": "true",
    }
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default)
                              for name, default in defaults.items()] + [OpaqueFunction(function=_start)])
