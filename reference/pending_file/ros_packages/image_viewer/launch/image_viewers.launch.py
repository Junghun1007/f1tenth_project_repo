from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    viewer_config = PathJoinSubstitution(
        [FindPackageShare("image_viewer"), "config", "image_viewers.yaml"]
    )

    viewers = [
        Node(
            package="image_viewer",
            executable="image_viewer_node",
            name=viewer_name,
            output="screen",
            parameters=[viewer_config],
        )
        for viewer_name in ("viewer_1", "viewer_2", "viewer_3")
    ]

    return LaunchDescription(viewers)
