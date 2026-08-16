from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Test only:
    #   8BitDo D-input Controller -> haptic-free joy_input_node -> /joy
    #
    # Run:
    #   ros2 launch vehicle_bringup joy_test.launch.py
    #
    # Check:
    #   ros2 topic list
    #   ros2 topic echo /joy
    #
    # The exact SDL device name remains stable if another input device is added.
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare("joy_initializer"), "launch", "joy.launch.py"]
    )
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch_path),
        launch_arguments={
            "device_id": "0",
            "device_name": "8BitDo Ultimate 2 Wireless Controller for PC",
            "deadzone": "0.05",
            "autorepeat_rate": "50.0",
            "sticky_buttons": "false",
            "coalesce_interval_ms": "1",
        }.items(),
    )

    return LaunchDescription([joy_launch])
