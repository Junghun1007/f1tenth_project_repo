from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    normal_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("image_processor"),
                    "launch",
                    "normal_image.launch.py",
                ]
            )
        )
    )
    bev_config = PathJoinSubstitution(
        [FindPackageShare("bev_processor"), "config", "bev_config.yaml"]
    )
    bev_node = Node(
        package="bev_processor",
        executable="bev_processor_node",
        name="bev_processor_node",
        output="screen",
        parameters=[bev_config],
    )

    return LaunchDescription([normal_pipeline, bev_node])
