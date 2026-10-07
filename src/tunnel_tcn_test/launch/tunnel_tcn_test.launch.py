import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = Path(get_package_share_directory('tunnel_tcn_test'))
    camera_share = Path(get_package_share_directory('ir_camera_driver'))
    arg = lambda name: LaunchConfiguration(name).perform(context)
    exposure, iso = int(arg('exposure_us')), int(arg('iso'))
    projector, flood = float(arg('projector')), float(arg('flood'))
    if not (10 <= exposure <= 33333 and 100 <= iso <= 1600 and 0 <= projector <= 1 and 0 <= flood <= 1):
        raise ValueError('Expected exposure_us=10..33333, ISO=100..1600, projector/flood=0..1')
    library = (camera_share / 'depthai_runtime_dir.txt').read_text().strip()
    camera = Node(
        package='ir_camera_driver', executable='ir_camera_driver_node', name='ir_camera_driver', output='screen',
        additional_env={'LD_LIBRARY_PATH': os.pathsep.join(filter(None, [library, context.environment.get('LD_LIBRARY_PATH', '')]))},
        parameters=[str(camera_share / 'config/ir_camera_config.yaml'), {
            'device_id': arg('device_id'), 'reprojection_enabled': True,
            'virtual_camera_position_ratio': .5, 'width': 1280, 'height': 800,
            'reprojection_fps': 30., 'single_camera_fps': 30., 'preview_max_fps': 30.,
            'manual_exposure_enabled': True, 'manual_exposure_us': exposure,
            'manual_sensitivity_iso': iso, 'ir_enabled': True,
            'ir_dot_projector_intensity': projector, 'ir_flood_light_intensity': flood,
            'rgb_enabled': False, 'bev_processing_enabled': False, 'use_sensor_timestamps': True,
            'stereo_preview_enabled': False, 'rgb_preview_enabled': False,
            'bev_preview_enabled': False, 'controls_preview_enabled': True,
            'bev.startup_measurement_enabled': False, 'imu_stabilization_enabled': False,
            'measurement_roi_preview_enabled': False,
        }])
    inference = Node(package='tunnel_tcn_test', executable='tunnel_tcn_node',
                     name='tunnel_tcn_test', output='screen',
                     parameters=[str(share / 'config/tunnel_tcn.yaml')])
    return [
        RegisterEventHandler(OnProcessExit(target_action=camera, on_exit=[EmitEvent(event=Shutdown(reason='Camera closed'))])),
        RegisterEventHandler(OnProcessExit(target_action=inference, on_exit=[EmitEvent(event=Shutdown(reason='Inference closed'))])),
        camera, inference,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('exposure_us', default_value='30000', description='Exposure in microseconds, 30000 = 30 ms'),
        DeclareLaunchArgument('iso', default_value='800'),
        DeclareLaunchArgument('projector', default_value='1.0', description='IR dot projector intensity [0,1]'),
        DeclareLaunchArgument('flood', default_value='0.0', description='IR flood light intensity [0,1]'),
        DeclareLaunchArgument('device_id', default_value='', description='Optional OAK device MXID'),
        OpaqueFunction(function=start),
    ])
