from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    # Empty shortcuts preserve the YAML values, including custom config files.
    overrides = {}
    for argument, parameter, convert in (
        ("resolution", "camera.resolution", str),
        ("fps", "camera.fps", float),
        ("dot", "depth.ir_dot_projector_intensity", float),
        ("flood", "depth.ir_flood_light_intensity", float),
    ):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            overrides[parameter] = convert(value)
    return [
        Node(
            package="point_cloud",
            executable="point_cloud_node",
            name="point_cloud",
            output="screen",
            parameters=[LaunchConfiguration("config_file"), overrides],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="point_cloud_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("point_cloud"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=str(share / "config" / "point_cloud.yaml")),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=str(share / "rviz" / "point_cloud.rviz")),
            DeclareLaunchArgument("resolution", default_value="", description="400p / 480p / 720p / 800p; empty uses YAML"),
            DeclareLaunchArgument("fps", default_value="", description="Requested FPS; empty uses YAML"),
            DeclareLaunchArgument("dot", default_value="", description="Dot projector 0..1; empty uses YAML"),
            DeclareLaunchArgument("flood", default_value="", description="IR flood light 0..1; empty uses YAML"),
            OpaqueFunction(function=_nodes),
        ]
    )
