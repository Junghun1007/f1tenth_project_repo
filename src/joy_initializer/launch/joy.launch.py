from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device_path = LaunchConfiguration("device_path")
    device_name = LaunchConfiguration("device_name")
    feedback_device = LaunchConfiguration("feedback_device")
    deadzone = LaunchConfiguration("deadzone")
    autorepeat_rate = LaunchConfiguration("autorepeat_rate")
    sticky_buttons = LaunchConfiguration("sticky_buttons")
    coalesce_interval_sec = LaunchConfiguration("coalesce_interval_sec")

    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        output="screen",
        parameters=[
            {
                "dev": ParameterValue(device_path, value_type=str),
                "dev_name": ParameterValue(device_name, value_type=str),
                # An empty path makes joy_linux_node skip opening a
                # force-feedback endpoint entirely.
                "dev_ff": ParameterValue(feedback_device, value_type=str),
                "deadzone": ParameterValue(deadzone, value_type=float),
                "autorepeat_rate": ParameterValue(
                    autorepeat_rate, value_type=float
                ),
                "sticky_buttons": ParameterValue(sticky_buttons, value_type=bool),
                "coalesce_interval": ParameterValue(
                    coalesce_interval_sec, value_type=float
                ),
            }
        ],
        # Defense in depth: even if another node publishes JoyFeedback, it is
        # not delivered on joy_linux_node's conventional feedback topic.
        remappings=[("joy/set_feedback", "joy/feedback_disabled")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("device_path", default_value="/dev/input/js0"),
            DeclareLaunchArgument(
                "device_name",
                default_value=(
                    "8BitDo 8BitDo Ultimate 2 Wireless Controller for PC"
                ),
            ),
            DeclareLaunchArgument(
                "feedback_device",
                default_value="",
            ),
            DeclareLaunchArgument("deadzone", default_value="0.05"),
            DeclareLaunchArgument("autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("sticky_buttons", default_value="false"),
            DeclareLaunchArgument("coalesce_interval_sec", default_value="0.001"),
            joy_node,
        ]
    )
