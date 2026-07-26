from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
            DeclareLaunchArgument(
                "imu_attitude_enabled",
                default_value="true",
                description="Replace BEV roll/pitch once from /camera/imu",
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
                        "imu_attitude_enabled": ParameterValue(
                            LaunchConfiguration("imu_attitude_enabled"),
                            value_type=bool,
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
            ),
        ]
    )
