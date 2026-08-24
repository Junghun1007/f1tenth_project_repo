from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vesc_port = LaunchConfiguration("vesc_port")
    controller_name_contains = LaunchConfiguration("controller_name_contains")
    input_mode = LaunchConfiguration("input_mode")
    can_interface = LaunchConfiguration("can_interface")
    can_controller_id = LaunchConfiguration("can_controller_id")

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
            "vesc_port": vesc_port,
            "controller_name_contains": controller_name_contains,
        }.items(),
    )
    dynamics_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vehicle_dynamics_monitor"),
                    "launch",
                    "vehicle_dynamics_monitor.launch.py",
                ]
            )
        ),
        launch_arguments={
            "input_mode": input_mode,
            "can_interface": can_interface,
            "can_controller_id": can_controller_id,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument(
                "controller_name_contains", default_value="8BitDo"
            ),
            DeclareLaunchArgument(
                "input_mode",
                default_value="ros_topic",
                description="ros_topic (current UART bridge) or socketcan.",
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("can_controller_id", default_value="0"),
            manual_drive,
            dynamics_monitor,
        ]
    )
