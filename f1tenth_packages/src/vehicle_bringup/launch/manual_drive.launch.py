from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keymap_path = PathJoinSubstitution(
        [FindPackageShare("vehicle_bringup"), "config", "manual_keymap.yaml"]
    )
    manual_to_vesc_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_bringup"), "config", "manual_to_vesc.yaml"]
    )
    vesc_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_bringup"), "config", "vesc.yaml"]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "device_id": 0,
                "device_name": "",
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
                "sticky_buttons": False,
                "coalesce_interval_ms": 1,
            }
        ],
    )

    manual_control_node = Node(
        package="manual_control",
        executable="manual_control_node",
        name="manual_control_node",
        output="screen",
        parameters=[
            {
                "keymap_path": keymap_path,
                "joy_topic": "/joy",
                "throttle_topic": "/manual/throttle",
                "steering_topic": "/manual/steering",
                "debug_topic": "/manual/controller_debug",
                "publish_debug": True,
            }
        ],
    )

    manual_to_vesc_node = Node(
        package="manual_control",
        executable="manual_to_vesc_node",
        name="manual_to_vesc_node",
        output="screen",
        parameters=[manual_to_vesc_config],
    )

    vesc_interface_node = Node(
        package="vesc_interface",
        executable="vesc_interface_node",
        name="vesc_interface_node",
        output="screen",
        parameters=[vesc_config],
    )

    return LaunchDescription(
        [joy_node, manual_control_node, manual_to_vesc_node, vesc_interface_node]
    )
