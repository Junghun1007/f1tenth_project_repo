from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("traffic_detection_test")
    default_params = f"{package_share}/config/traffic_detection_test.yaml"
    default_model = (
        f"{package_share}/models/"
        "traffic_light_yolox_s_640_batch_1.onnx"
    )

    params_file = LaunchConfiguration("params_file")
    model_path = LaunchConfiguration("model_path")
    score_threshold = LaunchConfiguration("score_threshold")
    nms_threshold = LaunchConfiguration("nms_threshold")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Traffic detection preview parameter YAML.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model,
                description="Decoded FP32 YOLOX ONNX model.",
            ),
            DeclareLaunchArgument(
                "score_threshold",
                default_value="0.25",
                description="Objectness times class-score threshold.",
            ),
            DeclareLaunchArgument(
                "nms_threshold",
                default_value="0.65",
                description="Intersection-over-union threshold for NMS.",
            ),
            ComposableNodeContainer(
                name="traffic_detection_test_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="traffic_detection_test",
                        plugin=(
                            "traffic_detection_test::"
                            "TrafficDetectionTestNode"
                        ),
                        name="traffic_detection_test",
                        parameters=[
                            params_file,
                            {
                                "model_path": model_path,
                                "score_threshold": ParameterValue(
                                    score_threshold, value_type=float
                                ),
                                "nms_threshold": ParameterValue(
                                    nms_threshold, value_type=float
                                ),
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
