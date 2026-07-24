from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory("bev_processor")
    default_params = os.path.join(package_share, "config", "bev_config.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="BEV processor parameter YAML",
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value="true",
                description="Show the independent OpenCV BEV preview",
            ),
            DeclareLaunchArgument(
                "publish_enabled",
                default_value="true",
                description="Publish the BEV image topic",
            ),
            DeclareLaunchArgument(
                "preview_max_fps",
                default_value="30.0",
                description="Maximum OpenCV preview FPS; BEV processing is unaffected",
            ),
            Node(
                package="bev_processor",
                executable="bev_processor_node",
                name="bev_processor",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "preview_enabled": LaunchConfiguration(
                            "preview_enabled"
                        ),
                        "publish_enabled": LaunchConfiguration(
                            "publish_enabled"
                        ),
                        "preview_max_fps": LaunchConfiguration(
                            "preview_max_fps"
                        ),
                    },
                ],
            ),
        ]
    )
