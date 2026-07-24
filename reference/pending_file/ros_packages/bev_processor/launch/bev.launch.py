from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    normal_preview_enabled = LaunchConfiguration(
        "normal_preview_enabled"
    )
    bev_preview_enabled = LaunchConfiguration("bev_preview_enabled")
    normal_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("image_processor"),
                    "launch",
                    "normal_image.launch.py",
                ]
            )
        ),
        launch_arguments={
            "preview_enabled": normal_preview_enabled,
        }.items(),
    )
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
                    bev_preview_enabled,
                    value_type=bool,
                )
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "normal_preview_enabled",
                default_value="false",
                description="Show the rectified camera preview window.",
            ),
            DeclareLaunchArgument(
                "bev_preview_enabled",
                default_value="true",
                description="Show the BEV preview window.",
            ),
            normal_pipeline,
            bev_node,
        ]
    )
