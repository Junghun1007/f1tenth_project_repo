from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    node_name = LaunchConfiguration("node_name")
    image_topic = LaunchConfiguration("image_topic")
    preview_fps = LaunchConfiguration("preview_fps")
    window_name = LaunchConfiguration("window_name")
    fit_to_image = LaunchConfiguration("fit_to_image")
    window_width = LaunchConfiguration("window_width")
    window_height = LaunchConfiguration("window_height")
    max_window_width = LaunchConfiguration("max_window_width")
    max_window_height = LaunchConfiguration("max_window_height")
    no_frame_timeout_sec = LaunchConfiguration("no_frame_timeout_sec")

    viewer_node = Node(
        package="image_viewer",
        executable="image_viewer_node",
        name=node_name,
        output="screen",
        parameters=[
            {
                "image_topic": ParameterValue(
                    image_topic,
                    value_type=str,
                ),
                "preview_fps": ParameterValue(
                    preview_fps,
                    value_type=float,
                ),
                "window_name": ParameterValue(
                    window_name,
                    value_type=str,
                ),
                "fit_to_image": ParameterValue(
                    fit_to_image,
                    value_type=bool,
                ),
                "window_width": ParameterValue(
                    window_width,
                    value_type=int,
                ),
                "window_height": ParameterValue(
                    window_height,
                    value_type=int,
                ),
                "max_window_width": ParameterValue(
                    max_window_width,
                    value_type=int,
                ),
                "max_window_height": ParameterValue(
                    max_window_height,
                    value_type=int,
                ),
                "no_frame_timeout_sec": ParameterValue(
                    no_frame_timeout_sec,
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "node_name",
                default_value="image_viewer_node",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
            ),
            DeclareLaunchArgument("preview_fps", default_value="15.0"),
            DeclareLaunchArgument("window_name", default_value="camera_raw"),
            DeclareLaunchArgument("fit_to_image", default_value="true"),
            DeclareLaunchArgument("window_width", default_value="640"),
            DeclareLaunchArgument("window_height", default_value="480"),
            DeclareLaunchArgument("max_window_width", default_value="1280"),
            DeclareLaunchArgument("max_window_height", default_value="720"),
            DeclareLaunchArgument(
                "no_frame_timeout_sec",
                default_value="3.0",
            ),
            viewer_node,
        ]
    )
