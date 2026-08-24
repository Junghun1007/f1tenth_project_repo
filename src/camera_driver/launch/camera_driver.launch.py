from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("camera_driver")
    default_params = f"{package_share}/config/camera_config.yaml"

    params_file = LaunchConfiguration("params_file")
    preview_enabled = LaunchConfiguration("preview_enabled")
    preview_grid_enabled = LaunchConfiguration("preview_grid_enabled")
    capture_directory = LaunchConfiguration("capture_directory")
    imu_stabilization_enabled = LaunchConfiguration(
        "imu_stabilization_enabled"
    )
    high_frequency_vibration_only_enabled = LaunchConfiguration(
        "imu_stabilization_high_frequency_vibration_only_enabled"
    )
    high_frequency_vibration_cutoff_hz = LaunchConfiguration(
        "imu_stabilization_high_frequency_vibration_cutoff_hz"
    )
    publish_enabled = LaunchConfiguration("publish_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Camera driver parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value="false",
                description="Show the independent latest-frame preview.",
            ),
            DeclareLaunchArgument(
                "preview_grid_enabled",
                default_value="false",
                description="Draw a light-gray 20-pixel grid on the preview.",
            ),
            DeclareLaunchArgument(
                "capture_directory",
                default_value=".",
                description=(
                    "Directory for B-button camera captures; default is the "
                    "launch working directory."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_enabled",
                default_value="true",
                description=(
                    "Apply OAK IMU roll/pitch correction to preview and "
                    "published NV12 output."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_high_frequency_vibration_only_enabled",
                default_value="true",
                description=(
                    "Pass slow body attitude through and stabilize only "
                    "high-frequency roll/pitch vibration."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_high_frequency_vibration_cutoff_hz",
                default_value="3.0",
                description=(
                    "First-order high-pass cutoff in Hz for roll/pitch "
                    "vibration correction."
                ),
            ),
            DeclareLaunchArgument(
                "publish_enabled",
                default_value="false",
                description="Publish sensor_msgs/Image frames.",
            ),
            ComposableNodeContainer(
                name="camera_container",
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
                            params_file,
                            {
                                "preview_enabled": preview_enabled,
                                "preview_grid_enabled": preview_grid_enabled,
                                "capture_directory": capture_directory,
                                "imu_stabilization_enabled": (
                                    imu_stabilization_enabled
                                ),
                                (
                                    "imu_stabilization_high_frequency_"
                                    "vibration_only_enabled"
                                ): ParameterValue(
                                    high_frequency_vibration_only_enabled,
                                    value_type=bool,
                                ),
                                (
                                    "imu_stabilization_high_frequency_"
                                    "vibration_cutoff_hz"
                                ): ParameterValue(
                                    high_frequency_vibration_cutoff_hz,
                                    value_type=float,
                                ),
                                "publish_enabled": publish_enabled,
                                # Standalone launch does not publish IMU;
                                # stabilization can still use it internally.
                                "imu_bridge_enabled": False,
                            },
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    )
                ],
            ),
        ]
    )
