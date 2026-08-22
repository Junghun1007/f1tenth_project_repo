from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vesc_port = LaunchConfiguration("vesc_port")
    controller_name_contains = LaunchConfiguration("controller_name_contains")
    actuator_commander_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_bringup"), "config", "manual_vesc_config.yaml"]
    )
    vesc_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_bringup"), "config", "vesc_config.yaml"]
    )
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare("joy_initializer"), "launch", "joy.launch.py"]
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch_path),
        launch_arguments={
            "device_id": "0",
            "device_name_contains": controller_name_contains,
            "deadzone": "0.05",
            "autorepeat_rate": "50.0",
            "sticky_buttons": "false",
            "coalesce_interval_ms": "1",
            "reconnect_interval_sec": "0.2",
        }.items(),
    )

    actuator_commander_node = Node(
        package="manual_control",
        executable="actuator_commander_node",
        name="actuator_commander_node",
        output="screen",
        parameters=[actuator_commander_config],
    )

    vesc_bridge_node = Node(
        package="vesc_bridge",
        executable="vesc_bridge_node",
        name="vesc_bridge_node",
        output="screen",
        parameters=[vesc_config, {"port": vesc_port}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument(
                "controller_name_contains", default_value="8BitDo"
            ),
            joy_launch,
            actuator_commander_node,
            vesc_bridge_node,
        ]
    )
