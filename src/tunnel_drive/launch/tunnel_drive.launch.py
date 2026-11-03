import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start(context):
    args = {'tunnel_enabled': 'true',
            'auto_enabled': LaunchConfiguration('auto_enabled').perform(context)}
    speed = LaunchConfiguration('speed_mps').perform(context)
    if speed != 'inherit':
        value = float(speed)
        if not math.isfinite(value) or value <= 0:
            raise ValueError('speed_mps must be a finite positive number')
        args.update(minimum_speed_mps=str(value), maximum_speed_mps=str(value))
    path = Path(get_package_share_directory('vehicle_bringup')) / 'launch/auto_drive.launch.py'
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                    launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('speed_mps', default_value='inherit',
                              description='Positive target m/s, or inherit existing external drive speed settings'),
        DeclareLaunchArgument('auto_enabled', default_value='false'),
        OpaqueFunction(function=start)])
