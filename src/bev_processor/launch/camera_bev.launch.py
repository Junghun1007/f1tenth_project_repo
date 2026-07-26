from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
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
            DeclareLaunchArgument(
                "camera_x_m",
                default_value="0.0",
                description="Camera X position from the front axle in meters",
            ),
            DeclareLaunchArgument(
                "camera_y_m",
                default_value="0.0",
                description="Camera Y position from vehicle center in meters",
            ),
            DeclareLaunchArgument(
                "camera_z_m",
                default_value="0.20",
                description="Camera height above the ground in meters",
            ),
            DeclareLaunchArgument(
                "camera_roll_deg",
                default_value="0.0",
                description="Camera mounting roll in degrees",
            ),
            DeclareLaunchArgument(
                "camera_downward_pitch_deg",
                default_value="14.0",
                description="Positive downward camera pitch in degrees",
            ),
            DeclareLaunchArgument(
                "camera_yaw_deg",
                default_value="0.0",
                description="Camera mounting yaw in degrees",
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
                                "camera_x_m": ParameterValue(
                                    LaunchConfiguration("camera_x_m"),
                                    value_type=float,
                                ),
                                "camera_y_m": ParameterValue(
                                    LaunchConfiguration("camera_y_m"),
                                    value_type=float,
                                ),
                                "camera_z_m": ParameterValue(
                                    LaunchConfiguration("camera_z_m"),
                                    value_type=float,
                                ),
                                "camera_roll_deg": ParameterValue(
                                    LaunchConfiguration("camera_roll_deg"),
                                    value_type=float,
                                ),
                                "camera_downward_pitch_deg": ParameterValue(
                                    LaunchConfiguration(
                                        "camera_downward_pitch_deg"
                                    ),
                                    value_type=float,
                                ),
                                "camera_yaw_deg": ParameterValue(
                                    LaunchConfiguration("camera_yaw_deg"),
                                    value_type=float,
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
