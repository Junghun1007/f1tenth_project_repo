from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load(path, node):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)[node]["ros__parameters"]


def telemetry_topic(override, configured, namespace):
    # Match auto_control's root topics by default; manual_drive adds a vehicle namespace.
    if override.strip():
        return override.strip()
    prefix = namespace.strip("/")
    return "/" + "/".join(part for part in (prefix, configured.strip("/")) if part)


def nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    roi = load(arg("roi_params_file"), "roi_lidar")
    # Keep the user's requested 1.2x3m terrain and a separate clean 120x300 lane BEV.
    roi.update({"camera.fps": 30.0, "preview.width_m": 1.2, "preview.forward_m": 3.0,
                "preview.gui": True, "preview.publish_lane_bev": True,
                "motion.publish_imu": True})
    bev = load(arg("bev_params_file"), "bev_processor")
    for key in ("camera_x_m", "camera_y_m", "camera_yaw_deg"):
        roi["bev." + key] = bev[key]
    roi.update({k: v for k, v in bev.items()
                if k.startswith("measurement_") or k.startswith("manual_camera_height_")})
    roi["frame_id"] = "front_axle_bev"
    lane = load(arg("lane_params_file"), "line_detactor")
    lane.update({"input_topic": "/roi_lidar/lane_bev", "direct_bev_input_enabled": False,
                 "preview_enabled": False, "bev_theme_enable": False,
                 "centerline_enabled": True, "connection_enabled": True,
                 "centerline_bev_width_m": 1.2, "centerline_bev_height_m": 3.0,
                 "result_publish_enabled": True, "result_topic": "/line_detactor/result"})
    control = load(arg("vehicle_params_file"), "auto_control")
    obstacles = load(arg("obstacle_params_file"), "auto_obstacles")
    erpm_topic = telemetry_topic(arg("measured_erpm_topic"), control["measured_erpm_topic"], arg("vehicle_namespace"))
    connection_topic = telemetry_topic(arg("vesc_connection_topic"), control["connection_status_topic"], arg("vehicle_namespace"))
    test = load(arg("params_file"), "roi_vfh_test")
    test.update({"wheelbase_m": control["avoidance_wheelbase_m"],
                 "max_steering_deg": control["maximum_steering_angle_deg"],
                 "steering_rate_deg_per_sec": control["steering_rate_limit_deg_per_sec"],
                 "vehicle_width_m": 2 * obstacles["obstacles.avoidance.vehicle_half_width_m"],
                 "rear_extent_m": obstacles["obstacles.avoidance.vehicle_half_length_m"],
                 "front_overhang_m": obstacles["obstacles.avoidance.vehicle_half_length_m"],
                 "lane_width_m": lane["centerline_lane_width_m"],
                 "measured_speed_topic": arg("measured_speed_topic")})
    actions = [Node(package="roi_lidar", executable="roi_lidar_node", name="roi_lidar",
                    output="screen", parameters=[roi]),
               IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory("line_detactor")) / "launch/line_detactor.launch.py")),
                    launch_arguments={"params_file": arg("lane_params_file"),
                                      "input_topic": lane["input_topic"], "direct_bev_input_enabled": "false",
                                      "preview_enabled": "false", "bev_theme_enable": "false",
                                      "centerline_enabled": "true", "connection_enabled": "true",
                                      "centerline_bev_width_m": "1.2", "centerline_bev_height_m": "3.0",
                                      "result_publish_enabled": "true", "result_topic": "/line_detactor/result"}.items()),
               Node(package="roi_vfh_test", executable="roi_vfh_preview", name="roi_vfh_test",
                    output="screen", parameters=[test])]
    if arg("dynamics_enabled").lower() == "true":
        dynamics = load(str(Path(get_package_share_directory("vehicle_dynamics_monitor")) / "config/vehicle_dynamics.yaml"), "/**")
        dynamics.update({"input_mode": "ros_topic", "measured_erpm_topic": erpm_topic,
                         "connection_status_topic": connection_topic,
                         "speed_topic": arg("measured_speed_topic"), "imu_topic": "/roi_lidar/imu",
                         "imu_yaw_axis": "z", "imu_yaw_rate_sign": 1.0})
        # Keep ERPM-to-m/s calibration consistent with the existing controller.
        for key in ("wheel_diameter_m", "motor_pole_pairs", "motor_pinion_teeth", "spur_gear_teeth",
                    "differential_pinion_teeth", "differential_ring_teeth", "erpm_direction_sign",
                    "speed_scale_correction", "speed_filter_time_constant_sec"):
            dynamics[key] = control[key]
        actions.append(Node(package="vehicle_dynamics_monitor", executable="vehicle_dynamics_node",
                            name="roi_vfh_dynamics", output="screen", parameters=[dynamics]))
    elif arg("dynamics_enabled").lower() != "false":
        raise ValueError("dynamics_enabled must be true or false")
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("roi_vfh_test"))
    auto = Path(get_package_share_directory("auto_control"))
    defaults = {"params_file": share / "config/preview.yaml",
                "roi_params_file": Path(get_package_share_directory("roi_lidar")) / "config/roi_lidar.yaml",
                "lane_params_file": Path(get_package_share_directory("line_detactor")) / "config/line_detactor.yaml",
                "bev_params_file": Path(get_package_share_directory("bev_processor")) / "config/bev_config.yaml",
                "vehicle_params_file": auto / "config/auto_control.yaml",
                "obstacle_params_file": auto / "config/obstacles.yaml",
                "dynamics_enabled": "true", "vehicle_namespace": "", "measured_erpm_topic": "",
                "vesc_connection_topic": "",
                "measured_speed_topic": "/roi_vfh_test/measured_speed"}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=str(v)) for k, v in defaults.items()] + [OpaqueFunction(function=nodes)])
