from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    camera_config = PathJoinSubstitution(
        [FindPackageShare("camera_driver"), "config", "camera_config.yaml"]
    )

    camera_driver_node = Node(
        package="camera_driver",
        executable="camera_driver_node",
        name="camera_driver_node",
        output="screen",
        parameters=[camera_config],
    )

    return LaunchDescription([camera_driver_node])
