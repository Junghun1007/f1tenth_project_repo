from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():
    camera_share = get_package_share_directory("camera_driver")
    bev_share = get_package_share_directory("bev_processor")

    camera_params = os.path.join(
        camera_share, "config", "camera_config.yaml"
    )
    bev_params = os.path.join(bev_share, "config", "bev_config.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=camera_params,
                description="Camera driver parameter YAML",
            ),
            DeclareLaunchArgument(
                "bev_params_file",
                default_value=bev_params,
                description="BEV processor parameter YAML",
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value="true",
                description="Show the BEV output preview",
            ),
            DeclareLaunchArgument(
                "publish_enabled",
                default_value="true",
                description="Publish the BGR8 BEV output topic",
            ),
            DeclareLaunchArgument(
                "preview_max_fps",
                default_value="30.0",
                description="Maximum BEV preview refresh rate",
            ),
            ComposableNodeContainer(
                name="camera_bev_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="camera_driver",
                        plugin="camera_driver::CameraDriverNode",
                        name="camera_driver",
                        parameters=[
                            LaunchConfiguration("camera_params_file"),
                            {
                                "preview_enabled": False,
                                "publish_enabled": True,
                            },
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                    ComposableNode(
                        package="bev_processor",
                        plugin="bev_processor::BevProcessorNode",
                        name="bev_processor",
                        parameters=[
                            LaunchConfiguration("bev_params_file"),
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
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                ],
            ),
        ]
    )
