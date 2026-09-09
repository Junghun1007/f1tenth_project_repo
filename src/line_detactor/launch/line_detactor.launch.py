from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("line_detactor")
    default_params = f"{package_share}/config/line_detactor.yaml"
    default_model = f"{package_share}/models/best.pt"

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument("input_topic", default_value="/camera/image_bev"),
            DeclareLaunchArgument("device", default_value="cuda:0"),
            DeclareLaunchArgument("amp_enabled", default_value="true"),
            DeclareLaunchArgument("mask_threshold", default_value="0.5"),
            DeclareLaunchArgument("preview_enabled", default_value="true"),
            DeclareLaunchArgument("preview_fps", default_value="30.0"),
            Node(
                package="line_detactor",
                executable="line_detactor_node",
                name="line_detactor",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "device": LaunchConfiguration("device"),
                        "amp_enabled": ParameterValue(
                            LaunchConfiguration("amp_enabled"),
                            value_type=bool,
                        ),
                        "mask_threshold": ParameterValue(
                            LaunchConfiguration("mask_threshold"),
                            value_type=float,
                        ),
                        "preview_enabled": ParameterValue(
                            LaunchConfiguration("preview_enabled"),
                            value_type=bool,
                        ),
                        "preview_fps": ParameterValue(
                            LaunchConfiguration("preview_fps"),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
