import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_share = get_package_share_directory("auto_control")
    camera_share = get_package_share_directory("camera_driver")
    detector_params = os.path.join(
        auto_share, "config", "front_lane_rotated_detector.yaml"
    )
    camera_launch = os.path.join(
        camera_share, "launch", "camera_driver.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "detector_params_file",
                default_value=detector_params,
                description="Front lane detector parameter YAML.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                launch_arguments={
                    "preview_enabled": "false",
                    "publish_enabled": "true",
                }.items(),
            ),
            Node(
                package="auto_control",
                executable="front_lane_rotated_detector",
                name="front_lane_rotated_detector",
                output="screen",
                parameters=[LaunchConfiguration("detector_params_file")],
            ),
        ]
    )
