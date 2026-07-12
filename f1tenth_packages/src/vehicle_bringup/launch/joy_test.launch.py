from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Test only:
    #   8BitDo Controller -> joy_node -> /joy
    #
    # Run:
    #   ros2 launch vehicle_bringup joy_test.launch.py
    #
    # Check:
    #   ros2 topic list
    #   ros2 topic echo /joy
    #
    # ROS 2 Humble joy_node selects /dev/input/jsN with device_id=N.
    # Current controller path /dev/input/js1 therefore maps to device_id=1.
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {
                "device_id": 1,
                "device_name": "",
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
                "sticky_buttons": False,
                "coalesce_interval_ms": 1,
            }
        ],
    )

    return LaunchDescription([joy_node])
