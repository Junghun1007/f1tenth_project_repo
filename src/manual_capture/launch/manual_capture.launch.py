"""Capture GUI + manual driving in one launch.

ros2 launch manual_capture manual_capture.launch.py vehicle_namespace:=autopilot03 ...
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_MANUAL_DRIVE_ARGUMENTS = (
    ("vehicle_namespace", "autopilot03"),
    ("vesc_port", "/dev/ttyTHS1"),
    ("controller_name_contains", "8BitDo"),
    ("input_mode", "ros_topic"),
    ("can_interface", "can0"),
    ("can_controller_id", "0"),
    ("slcan_channel", "/dev/ttyACM0"),
    ("slcan_bitrate", "500000"),
)


# auto_control BEV (bev_processor) pose parameter -> manual_capture parameter.
_AUTO_CONTROL_POSE_PARAMETERS = (
    ("camera_x_m", "bev.camera_x_m"),
    ("camera_y_m", "bev.camera_y_m"),
    ("camera_yaw_deg", "bev.camera_yaw_deg"),
    ("manual_camera_height_enabled", "measurement_manual_camera_height_enabled"),
    ("manual_camera_height_m", "measurement_manual_camera_height_m"),
)


def _default_auto_control_bev_params():
    try:
        share = get_package_share_directory("bev_processor")
    except PackageNotFoundError:
        return ""
    return os.path.join(share, "config", "bev_config.yaml")


def _auto_control_pose_overrides(path):
    """Camera mount pose used by auto_control's BEV, so captures match driving."""
    if not path:
        return {}
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.isfile(path):
        raise RuntimeError(f"auto_control BEV parameter file does not exist: {path}")
    with open(path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    parameters = config.get("bev_processor", {}).get("ros__parameters", {})
    # Node declares these as double/bool; an integer in YAML (e.g. 0) must not fail.
    return {
        target: (bool(parameters[source]) if source.endswith("_enabled") else float(parameters[source]))
        for source, target in _AUTO_CONTROL_POSE_PARAMETERS
        if source in parameters
    }


def _workspace_root(package_share):
    # <ws>/install/manual_capture/share/manual_capture -> <ws>
    return Path(package_share).resolve().parents[3]


def _capture_node(context):
    package_share = get_package_share_directory("manual_capture")
    capture_root = LaunchConfiguration("capture_root").perform(context).strip()
    if not capture_root:
        capture_root = str(_workspace_root(package_share) / "captures")
    joy_topic = LaunchConfiguration("capture_joy_topic").perform(context).strip()
    if not joy_topic:
        namespace = LaunchConfiguration("vehicle_namespace").perform(context).strip("/")
        joy_topic = f"/{namespace}/joy" if namespace else "/joy"

    pose = _auto_control_pose_overrides(
        LaunchConfiguration("auto_control_bev_params_file").perform(context).strip()
    )
    pose_message = ", ".join(f"{key}={value}" for key, value in pose.items()) or "manual_capture.yaml"

    environment = {}
    library_dir = LaunchConfiguration("depthai_library_dir").perform(context).strip()
    if library_dir:
        previous = context.environment.get("LD_LIBRARY_PATH", "")
        environment["LD_LIBRARY_PATH"] = os.pathsep.join(
            item for item in (library_dir, previous) if item
        )
    return [
        LogInfo(msg=f"[manual_capture] camera pose from auto_control BEV: {pose_message}"),
        Node(
            package="manual_capture",
            executable="manual_capture_node",
            name="manual_capture",
            output="screen",
            additional_env=environment,
            parameters=[
                LaunchConfiguration("params_file"),
                pose,
                {"capture_root": os.path.abspath(capture_root), "capture_joy_topic": joy_topic},
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("manual_capture")
    runtime_file = Path(package_share) / "depthai_runtime_dir.txt"
    default_library_dir = (
        runtime_file.read_text(encoding="utf-8").strip() if runtime_file.exists() else ""
    )
    manual_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("vehicle_bringup"), "launch",
                 "manual_drive_with_dynamics.launch.py"]
            )
        ),
        launch_arguments={
            name: LaunchConfiguration(name) for name, _ in _MANUAL_DRIVE_ARGUMENTS
        }.items(),
        condition=IfCondition(LaunchConfiguration("manual_drive_enabled")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=f"{package_share}/config/manual_capture.yaml",
            ),
            DeclareLaunchArgument(
                "auto_control_bev_params_file",
                default_value=_default_auto_control_bev_params(),
                description=(
                    "bev_processor YAML whose camera_x/y/yaw and manual height override "
                    "params_file so the capture BEV matches auto_control. Empty: params_file only."
                ),
            ),
            DeclareLaunchArgument(
                "capture_root",
                default_value="",
                description="Empty: <workspace root>/captures.",
            ),
            DeclareLaunchArgument(
                "capture_joy_topic",
                default_value="",
                description="Empty: /<vehicle_namespace>/joy.",
            ),
            DeclareLaunchArgument("manual_drive_enabled", default_value="true"),
            DeclareLaunchArgument("depthai_library_dir", default_value=default_library_dir),
            *[DeclareLaunchArgument(name, default_value=value)
              for name, value in _MANUAL_DRIVE_ARGUMENTS],
            manual_drive,
            OpaqueFunction(function=_capture_node),
        ]
    )
