from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("camera_height_estimator")
    default_params = f"{package_share}/config/camera_height_estimator.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Independent one-shot height estimator parameters.",
            ),
            Node(
                package="camera_height_estimator",
                executable="camera_height_estimator_node",
                name="camera_height_estimator",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
