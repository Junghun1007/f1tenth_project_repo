from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _prepare_output_root(context):
    output_root = Path(LaunchConfiguration("output_root").perform(context)).expanduser()
    output_root.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    vehicle_namespace = LaunchConfiguration("vehicle_namespace")
    manual_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("vehicle_bringup"), "launch", "manual_drive.launch.py"]
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

    # This is the sole OAK owner. CAM_A supplies clean RGB while synchronized
    # CAM_B/C frames supply the monochrome metric BEV.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ir_camera_driver"), "launch", "ir_camera_driver.launch.py"]
            )
        ),
        launch_arguments={
            "reprojection_enabled": "true",
            "ir_enabled": LaunchConfiguration("ir_enabled"),
            "ir_dot_projector_intensity": LaunchConfiguration(
                "ir_dot_projector_intensity"
            ),
            "ir_flood_light_intensity": LaunchConfiguration(
                "ir_flood_light_intensity"
            ),
        }.items(),
    )

    recorder = Node(
        package="tunnel_data_collection",
        executable="recording_controller",
        name="tunnel_recorder",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "output_root": LaunchConfiguration("output_root"),
                "max_bag_size": ParameterValue(
                    LaunchConfiguration("max_bag_size"), value_type=int
                ),
                "joy_topic": ParameterValue(
                    ["/", vehicle_namespace, "/joy"], value_type=str
                ),
                "toggle_button": ParameterValue(
                    LaunchConfiguration("record_button"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_namespace",
                default_value="autopilot03",
                description="Namespace used by the existing manual drive stack.",
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
                "record_button",
                default_value="6",
                description="sensor_msgs/Joy button index used to toggle recording.",
            ),
            DeclareLaunchArgument(
                "max_bag_size",
                default_value="4294967296",
                description="Maximum bytes per sqlite3 file before bag splitting.",
            ),
            DeclareLaunchArgument("ir_enabled", default_value="true"),
            DeclareLaunchArgument(
                "ir_dot_projector_intensity", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "ir_flood_light_intensity", default_value="0.0"
            ),
            OpaqueFunction(function=_prepare_output_root),
            manual_drive,
            camera,
            recorder,
        ]
    )
