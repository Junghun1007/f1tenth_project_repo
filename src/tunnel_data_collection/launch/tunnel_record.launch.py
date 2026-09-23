from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _prepare_output_root(context):
    output_root = Path(LaunchConfiguration("output_root").perform(context)).expanduser()
    output_root.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    # This is the sole OAK owner. CAM_A supplies clean RGB while synchronized
    # CAM_B/C frames supply the monochrome metric BEV.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ir_camera_driver"), "launch", "ir_camera_driver.launch.py"]
            )
        ),
        launch_arguments={
            "reprojection_enabled": "true",
            "ir_enabled": LaunchConfiguration("ir_enabled"),
            "ir_dot_projector_intensity": LaunchConfiguration(
                "ir_dot_projector_intensity"
            ),
            "ir_flood_light_intensity": LaunchConfiguration(
                "ir_flood_light_intensity"
            ),
            "rgb_width": LaunchConfiguration("rgb_width"),
            "rgb_height": LaunchConfiguration("rgb_height"),
            "rgb_fps": LaunchConfiguration("camera_fps"),
            "stereo_width": LaunchConfiguration("stereo_width"),
            "stereo_height": LaunchConfiguration("stereo_height"),
            "stereo_fps": LaunchConfiguration("camera_fps"),
            "processing_fps": LaunchConfiguration("camera_fps"),
            "rgb_preview_enabled": LaunchConfiguration("rgb_preview_enabled"),
            "stereo_preview_enabled": LaunchConfiguration(
                "stereo_preview_enabled"
            ),
            "bev_preview_enabled": LaunchConfiguration("bev_preview_enabled"),
            "controls_preview_enabled": LaunchConfiguration(
                "controls_preview_enabled"
            ),
        }.items(),
    )

    recorder = Node(
        package="tunnel_data_collection",
        executable="recording_controller",
        name="tunnel_recorder",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "output_root": LaunchConfiguration("output_root"),
                "recording_fps": ParameterValue(
                    LaunchConfiguration("recording_fps"), value_type=float
                ),
                "h264_crf": ParameterValue(
                    LaunchConfiguration("h264_crf"), value_type=int
                ),
                "h264_preset": LaunchConfiguration("h264_preset"),
                "ffmpeg_binary": LaunchConfiguration("ffmpeg_binary"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_root",
                default_value=str(Path.cwd() / "tunnel_recordings"),
                description="Parent directory for timestamped MP4 sessions.",
            ),
            DeclareLaunchArgument(
                "recording_fps",
                default_value="30.0",
                description="FPS written into each MP4 and maximum saved frame rate.",
            ),
            DeclareLaunchArgument(
                "h264_crf",
                default_value="18",
                description="H.264 quality: 0 is lossless, 18 is high quality, 51 is lowest.",
            ),
            DeclareLaunchArgument("h264_preset", default_value="ultrafast"),
            DeclareLaunchArgument("ffmpeg_binary", default_value="ffmpeg"),
            DeclareLaunchArgument("camera_fps", default_value="30.0"),
            DeclareLaunchArgument("rgb_width", default_value="1280"),
            DeclareLaunchArgument("rgb_height", default_value="800"),
            DeclareLaunchArgument("stereo_width", default_value="1280"),
            DeclareLaunchArgument("stereo_height", default_value="800"),
            DeclareLaunchArgument("rgb_preview_enabled", default_value="true"),
            DeclareLaunchArgument("stereo_preview_enabled", default_value="true"),
            DeclareLaunchArgument("bev_preview_enabled", default_value="true"),
            DeclareLaunchArgument("controls_preview_enabled", default_value="true"),
            DeclareLaunchArgument("ir_enabled", default_value="true"),
            DeclareLaunchArgument(
                "ir_dot_projector_intensity", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "ir_flood_light_intensity", default_value="0.0"
            ),
            OpaqueFunction(function=_prepare_output_root),
            camera,
            recorder,
        ]
    )
