"""Pure configuration checks; no ROS imports or mutation of installed YAMLs."""
from pathlib import Path
import hashlib
import json
import math
import yaml

TUNABLES = {'longitudinal_planning_deceleration_mps2', 'duty_rise_rate_per_sec',
            'duty_fall_rate_per_sec', 'brake_current_rise_amps_per_sec',
            'brake_current_fall_amps_per_sec', 'corner_planning_acceleration_mps2'}
PREFIX = '/longitudinal_autotune'


def load(path):
    document = yaml.safe_load(Path(path).read_text())
    experiment, search = document['experiment'], document['search']
    if experiment['stop_lines_per_lap'] != 2 or set(experiment['required_directions']) != {'cw', 'ccw'}:
        raise ValueError('This experiment requires two stop lines and CW/CCW validation')
    if len(experiment['required_directions']) != 2:
        raise ValueError('Duplicate experiment direction')
    for name, value in experiment.items():
        if name in ('require_voltage', 'required_directions', 'score_weights'):
            continue
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
            raise ValueError(f'Invalid experiment value: {name}')
        if value == 0 and name != 'stop_range_min_m':
            raise ValueError(f'{name} must be positive')
    if (experiment['laps_per_direction'] < 2 or int(experiment['laps_per_direction']) != experiment['laps_per_direction'] or
            not experiment['stop_range_min_m'] <= experiment['stop_target_m'] <= experiment['stop_range_max_m'] or
            not .1 <= experiment['minimum_speed_mps'] <= experiment['maximum_speed_mps'] <= 1.8 or
            not 0 < experiment['hold_current_a'] <= experiment['emergency_brake_current_a'] <= 8. or
            experiment['stop_confirm_frames'] < 2 or int(experiment['stop_confirm_frames']) != experiment['stop_confirm_frames'] or
            experiment['hold_sec'] < experiment['stopped_confirm_sec']):
        raise ValueError('Invalid repetition, stop range or speed policy')
    if not isinstance(experiment['require_voltage'], bool):
        raise ValueError('require_voltage must be boolean')
    if set(search['bounds']) != TUNABLES:
        raise ValueError('Search may change only the six approved response parameters')
    for name, bounds in search['bounds'].items():
        if len(bounds) != 2 or not all(math.isfinite(v) and v > 0 for v in bounds) or bounds[0] >= bounds[1]:
            raise ValueError(f'Invalid search bounds: {name}')
    if (not isinstance(search['max_candidates'], int) or search['max_candidates'] < 1 or
            not 0 < search['minimum_step_fraction'] <= search['initial_step_fraction'] <= .5 or
            not 0 < search['minimum_relative_improvement'] < 1):
        raise ValueError('Invalid search budget/steps')
    if not all(math.isfinite(v) and v > 0 for v in experiment['score_weights'].values()):
        raise ValueError('Invalid score weights')
    return document


def ros_parameters(path, node):
    doc = yaml.safe_load(Path(path).read_text())
    return dict(doc.get(node, doc.get('/**', {})).get('ros__parameters', {}))


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def control_parameters(base, document):
    p = dict(base)
    p.update(document.get('auto_control', {}).get('ros__parameters', {}))
    e = document['experiment']
    # A private internal stop request exercises the existing red/green stop state
    # machine. There is no traffic signal model/observation in this experiment.
    p.update(enabled=False, control_mode='drive', traffic_stop_enabled=True,
             traffic_state_topic=PREFIX+'/stop_request', lane_result_topic=PREFIX+'/lane',
             enable_topic=PREFIX+'/controller_enabled', traffic_pass_if_unstoppable_enabled=False,
             avoidance_control_enabled=False, obstacle_slowdown_enabled=False,
             **{'tunnel.enabled': False}, model_startup_check_required=False,
             curvature_speed_control_enabled=True, longitudinal_pid_enabled=True,
             longitudinal_staged_control_enabled=True, performance_measurement_enabled=False,
             driving_log_enabled=True, driving_log_rate_hz=50.,
             minimum_speed_mps=e['minimum_speed_mps'], maximum_speed_mps=e['maximum_speed_mps'],
             traffic_stop_range_max_m=e['stop_range_max_m'], traffic_stop_margin_m=e['stop_target_m'],
             maximum_lateral_acceleration_mps2=e['maximum_lateral_acceleration_mps2'],
             maximum_duty=.10, electrical_brake_enabled=True,
             brake_maximum_current_amps=e['emergency_brake_current_a'],
             traffic_hold_current_amps=e['hold_current_a'],
             measured_erpm_topic=PREFIX+'/vesc/measured_erpm', connection_status_topic=PREFIX+'/vesc/connected',
             dynamics_duty_topic=PREFIX+'/dynamics/duty_cycle', dynamics_motor_current_topic=PREFIX+'/dynamics/motor_current_a')
    for name in list(p):
        if name.endswith('_topic') and isinstance(p[name], str) and p[name].startswith('/auto/'):
            p[name] = PREFIX+'/controller/'+p[name][6:]
    for channel in ('duty', 'brake_current', 'servo_position'):
        p[channel+'_topic'] = PREFIX+'/raw/'+channel
    for name, (low, high) in document['search']['bounds'].items():
        if not low <= float(p[name]) <= high:
            raise ValueError(f'Baseline {name}={p[name]} is outside approved bounds')
    if (p.get('lane_result_frame_id') != 'front_axle_bev' or
            not math.isclose(p.get('bev_x_max_m', 3.), 3.) or
            not math.isclose(p.get('bev_y_max_m', .6), .6) or
            not math.isclose(p.get('bev_meter_per_pixel', .01), .01)):
        raise ValueError('Trial requires the production 120x300 front-axle BEV geometry')
    return p
