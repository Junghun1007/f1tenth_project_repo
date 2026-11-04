import copy
from pathlib import Path
import math
import pytest
import yaml
from longitudinal_autotune.assembly import build
from longitudinal_autotune.config import load, PREFIX as P
from longitudinal_autotune.gate import Inputs, motor_output
from conftest import REPO


def test_real_repository_parameters_build_isolated(document,baseline):
    shares={k:REPO/'src'/k for k in ('camera_driver','bev_processor','line_detactor','auto_control','vehicle_dynamics_monitor')}
    p=build(shares,document)
    assert not p['auto_control']['enabled']
    assert p['auto_control']['traffic_state_topic']==P+'/stop_request'
    assert not p['auto_control']['avoidance_control_enabled']
    assert not p['camera_driver']['obstacles.depth.enabled']
    assert p['line_detactor']['direct_bev_input_enabled']
    assert not p['bev_processor']['publish_enabled']
    assert p['auto_control']['maximum_speed_mps']==.8
    assert p['vehicle_dynamics_monitor']['yaw_rate_source']=='imu'
    assert all(p['auto_control'][c+'_topic']==P+'/raw/'+c for c in ('duty','brake_current','servo_position'))
    assert all(str(v).startswith(P) for k,v in p['vehicle_dynamics_monitor'].items() if k.endswith('_topic'))


def test_configuration_cannot_tune_geometry_or_watchdogs(document,tmp_path):
    for mutate in (lambda d:d['search']['bounds'].update(path_timeout_sec=[.1,1.]),
                   lambda d:d['experiment'].update(required_directions=['cw']),
                   lambda d:d['experiment'].update(stop_target_m=1.),
                   lambda d:d['experiment'].update(hold_sec=float('nan'))):
        d=copy.deepcopy(document);mutate(d);path=tmp_path/'bad.yaml';path.write_text(yaml.safe_dump(d))
        with pytest.raises(ValueError): load(path)


def test_gate_freshness_mode_exclusion_and_restart():
    i=Inputs();i.set('duty',.06,10.)
    assert i.fresh(('duty',),10.1,.15)
    assert not i.fresh(('duty',),10.2,.15)
    assert not i.fresh(('duty',),10.1,.15,after=10.05)
    assert motor_output(.06,0.,.5,'duty',True,2.,.7,False)==('duty',.06)
    assert motor_output(0.,1.,.5,'brake_current',True,2.,.7,False)==('brake_current',1.)
    assert motor_output(.06,0.,.5,'duty',False,2.,.7,False)==('brake_current',2.)
    assert motor_output(math.nan,math.nan,math.nan,'duty',False,2.,.7,True)==('brake_current',.7)
    for v in (math.nan,-.1,.11):
        with pytest.raises(ValueError): motor_output(v,0.,.5,'duty',True,2.,.7,False)


def test_selected_working_baseline_is_merged_before_experiment_policy(document,tmp_path):
    shares={k:REPO/'src'/k for k in ('camera_driver','bev_processor','line_detactor','auto_control','vehicle_dynamics_monitor')}
    control=tmp_path/'working_control.yaml'
    control.write_text(yaml.safe_dump({'auto_control':{'ros__parameters':dict(stanley_heading_lookahead_m=.08,
        steering_servo_inverted=False, maximum_speed_mps=1.4, longitudinal_planning_deceleration_mps2=.6)}}))
    camera=tmp_path/'working_camera.yaml'
    camera.write_text(yaml.safe_dump({'camera_driver':{'ros__parameters':{'imu_stabilization_vehicle_motion_compensation_enabled':False}}}))
    p=build(shares,document,{'auto_control':str(control),'camera_driver':str(camera)})
    assert p['auto_control']['stanley_heading_lookahead_m']==.08
    assert p['auto_control']['steering_servo_inverted'] is False
    assert p['auto_control']['longitudinal_planning_deceleration_mps2']==.6
    assert p['auto_control']['maximum_speed_mps']==.8
    assert not p['auto_control']['enabled']
    assert p['camera_driver']['imu_stabilization_vehicle_motion_compensation_enabled'] is False
    assert p['camera_driver']['imu_stabilization_measured_erpm_topic']==P+'/vesc/measured_erpm'
    with pytest.raises(ValueError): build(shares,document,{'auto_control':str(tmp_path/'missing.yaml')})
    control.write_text('wrong_node: {ros__parameters: {stanley_gain: 1.3}}')
    with pytest.raises(ValueError): build(shares,document,{'auto_control':str(control)})


def test_working_camera_baseline_keeps_imu_and_lateral_guard(document):
    shares={k:REPO/'src'/k for k in ('camera_driver','bev_processor','line_detactor','auto_control','vehicle_dynamics_monitor')}
    p=build(shares,document)
    assert p['camera_driver']['imu_stabilization_vehicle_motion_compensation_enabled'] is False
    assert p['camera_driver']['imu_stabilization_enabled'] is True
    assert p['camera_driver']['imu_bridge_enabled'] is True
    assert p['camera_driver']['imu_stabilization_external_reference_required'] is True
    assert p['auto_control']['stanley_gain']==1.3
    assert p['auto_control']['stanley_heading_lookahead_m']==.05
    assert p['auto_control']['steering_servo_inverted'] is True
    assert document['experiment']['maximum_cte_m']==.20


@pytest.mark.parametrize('hard',[.1,.2,.61])
def test_soft_tracking_target_and_hard_abort_are_separate(document,tmp_path,hard):
    d=copy.deepcopy(document);d['experiment']['hard_cte_limit_m']=hard
    path=tmp_path/'invalid_tracking.yaml';path.write_text(yaml.safe_dump(d))
    with pytest.raises(ValueError,match='hard_cte_limit_m'):load(path)
