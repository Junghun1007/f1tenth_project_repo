"""Build isolated parameter dictionaries from read-only installed defaults."""
from pathlib import Path
import math
from .config import PREFIX as P, control_parameters, ros_parameters


def build(shares, document):
    files = {'camera_driver': 'camera_config.yaml', 'bev_processor': 'bev_config.yaml',
             'line_detactor': 'line_detactor.yaml', 'auto_control': 'auto_control.yaml',
             'vehicle_dynamics_monitor': 'vehicle_dynamics.yaml'}
    values = {}
    for name, file in files.items():
        p = ros_parameters(Path(shares[name])/'config'/file, name)
        p.update(document.get(name, {}).get('ros__parameters', {}))
        values[name] = p
    camera, bev, detector, dynamics = (values[k] for k in (
        'camera_driver', 'bev_processor', 'line_detactor', 'vehicle_dynamics_monitor'))
    for name, expected in dict(x_min_m=0., x_max_m=3., y_min_m=-.6, y_max_m=.6,
                               meter_per_pixel=.01, output_width=120, output_height=300).items():
        if not math.isclose(float(bev[name]), expected):
            raise ValueError('Autotune preserves trained BEV geometry: '+name)
    if bev['output_frame_id'] != 'front_axle_bev':
        raise ValueError('Autotune requires front_axle_bev')
    values['auto_control'] = control_parameters(values['auto_control'], document)
    camera.update({'tunnel.enabled': False, 'obstacles.depth.enabled': False,
        'publish_enabled': False, 'preview_enabled': False, 'fused_bev_output_enabled': True,
        'imu_bridge_enabled': True, 'imu_topic': P+'/camera/imu',
        'fused_bev_topic': P+'/camera/bev_input',
        'imu_stabilization_external_reference_required': True,
        'imu_stabilization_external_reference_topic': P+'/camera/startup_ground_normal',
        'imu_stabilization_measured_erpm_topic': P+'/vesc/measured_erpm',
        'imu_stabilization_can_acceleration_topic': P+'/dynamics/acceleration',
        'bev_input_bottom_fraction': bev['input_bottom_fraction']})
    bev.update({'tunnel.enabled': False, 'obstacles.share_reference': False,
        'input_topic': P+'/camera/bev_input', 'output_topic': P+'/camera/image_bev',
        'startup_ground_reference_topic': P+'/camera/startup_ground_normal',
        'preview_stop_topic': P+'/operator_enabled', 'preview_enabled': False,
        'capture_joy_topic': P+'/joy', 'dataset_collection_enabled': False,
        'dataset_collection_manual_capture_mode': False,
        'direct_output_enabled': True, 'direct_host_copy_enabled': False, 'publish_enabled': False})
    detector.update({'tunnel.enabled': False, 'obstacles.overlay_enabled': False,
        'input_topic': P+'/camera/image_bev', 'direct_bev_input_enabled': True,
        'result_topic': P+'/lane', 'result_image_topic': P+'/lane_image',
        'stop_line_distance_enabled': True, 'connection_enabled': True,
        'centerline_enabled': True, 'result_publish_enabled': True,
        'model_input_width': 120, 'model_input_height': 300,
        'centerline_bev_width_m': 1.2, 'centerline_bev_height_m': 3.,
        'bev_theme_speed_topic': P+'/controller/current_speed',
        'preview_target_speed_topic': P+'/controller/target_speed',
        'preview_corner_state_topic': P+'/controller/corner_state',
        'preview_duty_topic': P+'/controller/current_duty',
        'control_latency_topic': P+'/controller/detector_input_to_control_decision_ms'})
    if detector.get('stop_line_mask_threshold', -1.) >= 1.:
        raise ValueError('Stop-line channel must be enabled for this experiment')
    model = Path(detector['model_path']).expanduser()
    if not model.is_absolute():
        model = Path(shares['line_detactor'])/'models'/model
    if not model.is_file():
        raise ValueError('RGB stop-line model not found: '+str(model))
    detector['model_path'] = str(model)
    for key in list(dynamics):
        if key.endswith('_topic'):
            if key == 'measured_erpm_topic': dynamics[key] = P+'/vesc/measured_erpm'
            elif key == 'connection_status_topic': dynamics[key] = P+'/vesc/connected'
            elif key == 'commanded_duty_topic': dynamics[key] = P+'/drive/current_duty'
            elif key == 'servo_position_topic': dynamics[key] = P+'/vesc/servo_position'
            elif key == 'imu_topic': dynamics[key] = P+'/camera/imu'
            else: dynamics[key] = P+'/dynamics/'+str(dynamics[key]).split('/')[-1]
    for key in ('wheel_diameter_m', 'motor_pole_pairs', 'motor_pinion_teeth', 'spur_gear_teeth',
                'differential_pinion_teeth', 'differential_ring_teeth', 'erpm_direction_sign', 'speed_scale_correction'):
        dynamics[key] = values['auto_control'][key]
    if dynamics['imu_yaw_axis'] not in ('x','y','z'):
        raise ValueError('Invalid IMU yaw axis')
    # Real IMU yaw, not a steering-derived value, verifies travel direction.
    dynamics['yaw_rate_source'] = 'imu'
    return values
