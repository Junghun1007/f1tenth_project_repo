from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _prepare_output_root(context):
    output_root = Path(LaunchConfiguration("output_root").perform(context)).expanduser()
    output_root.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    vehicle_namespace = LaunchConfiguration("vehicle_namespace")
    output_root = LaunchConfiguration("output_root")
    recording_fps = LaunchConfiguration("recording_fps")
    session_name = datetime.now().astimezone().strftime(
        "tunnel_%Y%m%d_%H%M%S_%f%z"
    )
    bag_path = PathJoinSubstitution([output_root, session_name])

    manual_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vehicle_bringup"),
                    "launch",
                    "manual_drive.launch.py",
                ]
            )
        ),
        launch_arguments={
            "vehicle_namespace": vehicle_namespace,
            "vesc_port": LaunchConfiguration("vesc_port"),
            "controller_name_contains": LaunchConfiguration(
                "controller_name_contains"
            ),
        }.items(),
    )

    camera_and_bev = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("bev_processor"), "launch", "bev_processor.launch.py"]
            )
        ),
        launch_arguments={
            "performance_measurement_enabled": "false",
            "preview_enabled": "true",
            "camera_preview_enabled": "true",
            "camera_publish_enabled": "true",
            "camera_publish_fps": recording_fps,
            "publish_enabled": "true",
            "publish_max_fps": recording_fps,
            "direct_output_enabled": "false",
            "direct_host_copy_enabled": "false",
        }.items(),
    )

    # sensor_msgs/Image is stored without a video codec. The bag is the pixel-exact
    # source; PNG datasets are derived later by extract_frames.
    recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--storage",
            "sqlite3",
            "--output",
            bag_path,
            "--max-bag-size",
            LaunchConfiguration("max_bag_size"),
            "/camera/image_rect",
            "/camera/image_bev",
            "/camera/imu",
            "/camera/startup_ground_normal",
            ["/", vehicle_namespace, "/joy"],
            ["/", vehicle_namespace, "/manual/current_duty"],
            ["/", vehicle_namespace, "/manual/current_brake_current"],
            ["/", vehicle_namespace, "/manual/gear"],
            ["/", vehicle_namespace, "/vesc/measured_erpm"],
            ["/", vehicle_namespace, "/vesc/connected"],
            ["/", vehicle_namespace, "/vesc/duty"],
            ["/", vehicle_namespace, "/vesc/brake_current"],
            ["/", vehicle_namespace, "/vesc/servo_position"],
        ],
        output="screen",
        sigterm_timeout="15",
        sigkill_timeout="5",
    )
    stop_if_recorder_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=recorder,
            on_exit=[
                LogInfo(
                    msg=(
                        "rosbag recorder exited; stopping camera and manual drive "
                        "so the vehicle cannot continue without recording."
                    )
                ),
                EmitEvent(
                    event=Shutdown(reason="tunnel rosbag recorder exited")
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_namespace",
                default_value="autopilot03",
                description="Namespace used by the manual drive stack.",
            ),
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument(
                "controller_name_contains", default_value="8BitDo"
            ),
            DeclareLaunchArgument(
                "output_root",
                default_value=str(Path.cwd() / "tunnel_recordings"),
                description="Parent directory for timestamped rosbag sessions.",
            ),
            DeclareLaunchArgument(
                "recording_fps",
                default_value="30.0",
                description="Maximum RGB and BEV recording rate.",
            ),
            DeclareLaunchArgument(
                "max_bag_size",
                default_value="4294967296",
                description="Maximum bytes per sqlite3 file before bag splitting.",
            ),
            OpaqueFunction(function=_prepare_output_root),
            LogInfo(msg=["Tunnel recording directory: ", bag_path]),
            manual_drive,
            camera_and_bev,
            stop_if_recorder_exits,
            recorder,
        ]
    )
