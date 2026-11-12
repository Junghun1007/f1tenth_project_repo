from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("multiclass_yolo")
    defaults = {
        "engine_cache_path": ("", str),
        "input_format": ("nv12", str),
        "inference_backend": ("TENSORRT", str),
        "engine_precision": ("fp16", str),
        "model_path": (share + "/models/multiclass_yolox_tiny_512x320_holdout10.onnx", str),
        "width": ("1280", int), "height": ("800", int),
        "sensor_fps": ("30.0", float), "undistort_enabled": ("true", bool),
        "roi_height_percent": ("65", int),
        "frame_timeout_sec": ("5.0", float),
        "tensorrt_workspace_size_mb": ("1024", int),
        "nms_threshold": ("0.65", float),
        **{name + "_threshold": ("0.25", float)
           for name in ("con", "truck", "sedan", "traffic_light")},
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, (value, _) in defaults.items()]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                  for name, (_, kind) in defaults.items()}
    return LaunchDescription(arguments + [Node(
        package="multiclass_yolo", executable="multiclass_yolo_node",
        name="multiclass_yolo", output="screen", parameters=[parameters],
    )])
