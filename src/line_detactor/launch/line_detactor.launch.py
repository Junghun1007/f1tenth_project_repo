from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("line_detactor")
    default_params = f"{package_share}/config/line_detactor.yaml"
    default_model = (
        f"{package_share}/models/"
        "fast_scnn_highres_120x300_batch_1.onnx"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument("input_topic", default_value="/camera/image_bev"),
            DeclareLaunchArgument("engine_cache_path", default_value=""),
            DeclareLaunchArgument(
                "tensorrt_workspace_size_mb", default_value="1024"
            ),
            DeclareLaunchArgument("model_input_width", default_value="120"),
            DeclareLaunchArgument("model_input_height", default_value="300"),
            DeclareLaunchArgument("mask_threshold", default_value="0.5"),
            DeclareLaunchArgument("overlay_alpha", default_value="0.75"),
            DeclareLaunchArgument("preview_enabled", default_value="true"),
            DeclareLaunchArgument("preview_fps", default_value="30.0"),
            ComposableNodeContainer(
                name="line_detactor_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="line_detactor",
                        plugin="line_detactor::LineDetactorNode",
                        name="line_detactor",
                        parameters=[
                            LaunchConfiguration("params_file"),
                            {
                                "model_path": LaunchConfiguration("model_path"),
                                "input_topic": LaunchConfiguration("input_topic"),
                                "engine_cache_path": LaunchConfiguration(
                                    "engine_cache_path"
                                ),
                                "tensorrt_workspace_size_mb": ParameterValue(
                                    LaunchConfiguration(
                                        "tensorrt_workspace_size_mb"
                                    ),
                                    value_type=int,
                                ),
                                "model_input_width": ParameterValue(
                                    LaunchConfiguration("model_input_width"),
                                    value_type=int,
                                ),
                                "model_input_height": ParameterValue(
                                    LaunchConfiguration("model_input_height"),
                                    value_type=int,
                                ),
                                "mask_threshold": ParameterValue(
                                    LaunchConfiguration("mask_threshold"),
                                    value_type=float,
                                ),
                                "overlay_alpha": ParameterValue(
                                    LaunchConfiguration("overlay_alpha"),
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
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                ],
            ),
        ]
    )
