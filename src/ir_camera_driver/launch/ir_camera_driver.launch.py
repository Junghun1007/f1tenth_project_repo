from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("ir_camera_driver")
    default_params = f"{package_share}/config/ir_camera_config.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="IR camera preview parameter file.",
            ),
            DeclareLaunchArgument(
                "reprojection_enabled",
                default_value="true",
                description=(
                    "Reproject synchronized stereo IR images to one virtual "
                    "camera view."
                ),
            ),
            DeclareLaunchArgument(
                "selected_camera",
                default_value="LEFT",
                description=(
                    "Lens used when reprojection is off and as the invalid-"
                    "disparity fallback: LEFT/CAM_B or RIGHT/CAM_C."
                ),
            ),
            DeclareLaunchArgument(
                "virtual_camera_position_ratio",
                default_value="0.5",
                description=(
                    "Virtual position from LEFT/CAM_B (0.0) to RIGHT/CAM_C "
                    "(1.0)."
                ),
            ),
            DeclareLaunchArgument(
                "ir_enabled",
                default_value="true",
                description="Enable configured OAK IR emitters at startup.",
            ),
            DeclareLaunchArgument(
                "ir_dot_projector_intensity",
                default_value="1.0",
                description="IR laser dot-projector intensity from 0.0 to 1.0.",
            ),
            DeclareLaunchArgument(
                "ir_flood_light_intensity",
                default_value="0.0",
                description="IR flood-light intensity from 0.0 to 1.0.",
            ),
            DeclareLaunchArgument(
                "capture_directory",
                default_value=".",
                description="Directory used by the B-key PNG capture.",
            ),
            Node(
                package="ir_camera_driver",
                executable="ir_camera_driver_node",
                name="ir_camera_driver",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "reprojection_enabled": ParameterValue(
                            LaunchConfiguration("reprojection_enabled"),
                            value_type=bool,
                        ),
                        "selected_camera": LaunchConfiguration(
                            "selected_camera"
                        ),
                        "virtual_camera_position_ratio": ParameterValue(
                            LaunchConfiguration(
                                "virtual_camera_position_ratio"
                            ),
                            value_type=float,
                        ),
                        "ir_enabled": ParameterValue(
                            LaunchConfiguration("ir_enabled"),
                            value_type=bool,
                        ),
                        "ir_dot_projector_intensity": ParameterValue(
                            LaunchConfiguration(
                                "ir_dot_projector_intensity"
                            ),
                            value_type=float,
                        ),
                        "ir_flood_light_intensity": ParameterValue(
                            LaunchConfiguration("ir_flood_light_intensity"),
                            value_type=float,
                        ),
                        "capture_directory": LaunchConfiguration(
                            "capture_directory"
                        ),
                    },
                ],
            ),
        ]
    )
