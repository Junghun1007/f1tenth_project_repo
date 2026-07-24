from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    preview_enabled = LaunchConfiguration("preview_enabled")
    bev_config = PathJoinSubstitution(
        [FindPackageShare("bev_processor"), "config", "bev_config.yaml"]
    )
    bev_node = Node(
        package="bev_processor",
        executable="bev_processor_node",
        name="bev_processor_node",
        output="screen",
        parameters=[
            bev_config,
            {
                "preview_enabled": ParameterValue(
                    preview_enabled,
                    value_type=bool,
                )
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "preview_enabled",
                default_value="true",
                description="Show the BEV preview window.",
            ),
            bev_node,
        ]
    )
