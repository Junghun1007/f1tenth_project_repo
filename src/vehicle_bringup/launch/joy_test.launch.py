from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Test only:
    #   8BitDo Controller -> joy_linux_node -> /joy
    #
    # Run:
    #   ros2 launch vehicle_bringup joy_test.launch.py
    #
    # Check:
    #   ros2 topic list
    #   ros2 topic echo /joy
    #
    # Select by device name so reconnecting another input device cannot silently
    # move the controller to a different /dev/input/jsN number.
    joy_device_path = LaunchConfiguration("joy_device_path")
    joy_device_name = LaunchConfiguration("joy_device_name")
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare("joy_initializer"), "launch", "joy.launch.py"]
    )
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch_path),
        launch_arguments={
            "device_path": joy_device_path,
            "device_name": joy_device_name,
            "feedback_device": "",
            "deadzone": "0.05",
            "autorepeat_rate": "20.0",
            "sticky_buttons": "false",
            "coalesce_interval_sec": "0.001",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_device_path",
                default_value="/dev/input/js0",
            ),
            DeclareLaunchArgument(
                "joy_device_name",
                default_value=(
                    "8BitDo 8BitDo Ultimate 2 Wireless Controller for PC"
                ),
            ),
            joy_launch,
        ]
    )
