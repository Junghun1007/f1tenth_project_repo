"""Exercise real coordinator callbacks/gating with ROS transport/process doubles."""
import importlib
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as N
import pytest
import yaml
from longitudinal_autotune.config import PREFIX as P


@pytest.fixture
def session(monkeypatch,tmp_path,document,baseline):
    clock=[100.]
    inputs=tmp_path/'inputs.yaml'
    inputs.write_text(yaml.safe_dump(dict(document=document,controller=baseline,perception={})))
    binary=tmp_path/'lib/auto_control/auto_control_node';binary.parent.mkdir(parents=True);binary.write_text('controller')
    model=tmp_path/'model.onnx';model.write_text('model')
    parameters=dict(inputs_file=str(inputs),campaign_dir=str(tmp_path/'campaign'),direction='cw',model_path=str(model))
    class Message:
        RED,GREEN=1,2
        def __init__(self,**kwargs):
            self.header=N(stamp=N(sec=0,nanosec=0),frame_id='')
            self.__dict__.update(kwargs)
    class Pub:
        def __init__(self,topic):self.topic=topic;self.messages=[]
        def publish(self,msg):self.messages.append(msg)
    class FakeNode:
        def __init__(self,name):self.pubs={};self.subs={}
        @property
        def subscriptions(self):
            # rclpy.Node owns this read-only property in ROS 2 Humble.
            return tuple(self.subs.values())
        def declare_parameter(self,k,v):parameters.setdefault(k,v)
        def get_parameter(self,k):return N(value=parameters[k])
        def create_publisher(self,kind,topic,qos):
            pub=Pub(topic);self.pubs[topic]=pub;return pub
        def create_subscription(self,kind,topic,cb,qos):self.subs[topic]=cb;return cb
        def create_service(self,*args):return None
        def create_timer(self,*args):return None
        def get_logger(self):return N(info=lambda x:None)
        def get_clock(self):
            return N(now=lambda:N(nanoseconds=int(clock[0]*1e9),to_msg=lambda:N(sec=int(clock[0]),nanosec=0)))
        def get_publishers_info_by_topic(self,topic):
            if '/controller/maximum_speed_request' in topic:return []
            owner='autotune_session' if '/gate/' in topic else 'autotune_handover' if '/vesc/' in topic else 'autotune_controller_1'
            return [N(node_name=owner,node_namespace=P)]
    def module(name,**attrs):
        m=ModuleType(name);m.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,m)
    module('rclpy');module('rclpy.node',Node=FakeNode)
    module('rclpy.qos',QoSProfile=lambda **kw:kw,ReliabilityPolicy=N(RELIABLE=1,BEST_EFFORT=2),DurabilityPolicy=N(TRANSIENT_LOCAL=1))
    module('ament_index_python');module('ament_index_python.packages',get_package_prefix=lambda p:str(tmp_path))
    for parent,names in [('std_msgs',['Bool','Float32','Int32','String']),('sensor_msgs',['Imu']),('line_detactor',['LaneResult']),('traffic_detection_test',['TrafficLightState'])]:
        module(parent);module(parent+'.msg',**{n:Message for n in names})
    module('std_srvs');module('std_srvs.srv',SetBool=Message,Trigger=Message)
    sys.modules.pop('longitudinal_autotune.node',None)
    mod=importlib.import_module('longitudinal_autotune.node')
    monkeypatch.setattr(mod.time,'monotonic',lambda:clock[0])
    class Controller:
        def __init__(self,root):self.started=0.;self.generation=0;self.active=False;self.pending=False
        def request(self,parameters):self.pending=True;self.active=False
        def tick(self):
            if self.pending:
                self.pending=False;self.active=True;self.generation+=1;self.started=clock[0];return True
            return False
        def alive(self):return self.active
        def close(self):self.active=False
    monkeypatch.setattr(mod,'Controller',Controller)
    node=mod.Session();node.tick()
    def ready():
        node.mode='AUTO';node.connected=True;node.model_ready=True
        node.lane_valid=True;node.lane_at_rx=clock[0];node.line_at_rx=math.nan
        for k,v in dict(raw_speed=0.,yaw_rate=0.,servo=.5,duty=0.,brake=0.,speed=0.,target_speed=0.,curvature=0.,cte=0.,applied_duty=0.,applied_brake=.7).items():
            node.inputs.set(k,v,clock[0])
    yield node,clock,ready,mod,Message
    node.close()
    sys.modules.pop('longitudinal_autotune.node',None)


def test_arm_requires_fresh_observations_and_failed_gate_brakes(session):
    n,clock,ready,mod,Message=session
    r=N(success=None,message='')
    n.arm(Message(data=True),r);assert not r.success
    ready();n.arm(Message(data=True),r);assert r.success and n.experiment.running
    n.inputs.set('duty',.06,clock[0]);n.raw_mode='duty';n.tick()
    assert n.pubs[P+'/gate/duty'].messages[-1].data==.06
    clock[0]+=.2;n.tick()
    assert n.experiment.phase=='FAULT'
    assert n.pubs[P+'/gate/brake_current'].messages[-1].data==2.
    assert n.pubs[P+'/controller_enabled'].messages[-1].data is False
    assert n.campaign.state['best'] is None


def test_handover_cannot_be_rearmed_by_fresh_data(session):
    n,clock,ready,mod,Message=session
    ready();r=N();n.arm(Message(data=True),r);assert r.success
    n.on_mode(Message(data='MANUAL'));ready();n.tick()
    assert n.experiment.phase=='FAULT' and not n.auto_arm
    n.arm(Message(data=True),r);assert not r.success
    assert n.pubs[P+'/gate/brake_current'].messages[-1].data==.7


def test_publisher_conflict_stops_before_forwarding(session):
    n,clock,ready,mod,Message=session
    ready();r=N();n.arm(Message(data=True),r)
    n.publisher_conflict=True;n.tick()
    assert n.experiment.phase=='FAULT'
    assert not n.pubs[P+'/gate/duty'].messages


def test_stop_release_is_private_and_no_best_from_interrupt(session):
    n,clock,ready,mod,Message=session
    ready();r=N();n.arm(Message(data=True),r);n.tick()
    assert n.pubs[P+'/stop_request'].messages[-1].state==1
    n.arm(Message(data=False),r);n.tick()
    assert n.pubs[P+'/stop_request'].messages[-1].state==2
    assert n.campaign.current['reports']=={}


def test_successful_block_waits_for_opposite_direction_and_durable_trace(session):
    n,clock,ready,mod,Message=session
    ready()
    n.experiment.results=[dict(feasible=True,score=10.,voltage_v=None) for _ in range(4)]
    n.experiment.phase='BLOCK_DONE'
    n.tick()
    assert n.campaign.current['reports']=={}
    assert n.barrier.wait(2.)
    n.tick()
    assert n.campaign.needed()==['ccw'] and n.campaign.state['best'] is None
    assert n.experiment.phase=='BLOCK_DONE' and not n.auto_arm
    assert n.pubs[P+'/controller_enabled'].messages[-1].data is False


def test_candidate_restarts_only_after_pair_and_standstill(session):
    n,clock,ready,mod,Message=session
    ready()
    results=[dict(feasible=True,score=10.,voltage_v=None) for _ in range(4)]
    n.campaign.submit('ccw',results)
    n.experiment.results=results;n.experiment.phase='BLOCK_DONE'
    n.tick();assert n.barrier.wait(2.)
    n.inputs.set('raw_speed',.2,clock[0]);n.tick()
    assert n.campaign.current['id']==0
    n.inputs.set('raw_speed',0.,clock[0]);n.tick()
    assert n.campaign.current['id']==1 and n.campaign.state['best']['id']==0
    assert n.controller.pending and n.experiment.phase=='IDLE'
    n.tick()
    assert not n.experiment.running  # New generation must receive its own fresh commands.


def test_hard_lateral_limit_reports_frozen_value_and_blocks_drive(session):
    import json
    n,clock,ready,mod,Message=session
    ready();n.arm(Message(data=True),N())
    n.inputs.set('speed',.46923425793647766,clock[0])
    n.inputs.set('raw_speed',.6265654968593563,clock[0])
    n.inputs.set('cte',-.4118956446647644,clock[0])
    n.inputs.set('duty',.05893386900424957,clock[0]);n.raw_mode='duty'
    clock[0]+=.02;n.tick()
    assert n.experiment.reason=='hard_cross_track_error_limit'
    assert not n.pubs[P+'/gate/duty'].messages
    assert n.pubs[P+'/gate/brake_current'].messages[-1].data==2.
    assert 'value=-0.4119m' in n.pubs[P+'/fault'].messages[-1].data
    n.inputs.set('cte',0.,clock[0]);clock[0]+=.5;n.tick()
    status=json.loads(n.pubs[P+'/status'].messages[-1].data)
    assert status['fault']['value']==-.4118956446647644
    assert n.campaign.current['reports']=={}


def test_soft_lateral_excursion_keeps_forwarding_and_exposes_quality(session):
    import json
    n,clock,ready,mod,Message=session
    ready();n.arm(Message(data=True),N())
    for i in range(30):
        for key,(value,_) in list(n.inputs.values.items()):n.inputs.set(key,value,clock[0])
        n.lane_at_rx=clock[0]
        n.inputs.set('cte',-.2118956446647644,clock[0])
        n.inputs.set('speed',.469,clock[0]);n.inputs.set('raw_speed',.626,clock[0])
        n.inputs.set('duty',.0589,clock[0]);n.raw_mode='duty'
        clock[0]+=.02;n.tick()
    assert n.experiment.running
    assert n.pubs[P+'/gate/duty'].messages[-1].data==.0589
    assert not n.pubs[P+'/fault'].messages
    quality=json.loads(n.pubs[P+'/tracking'].messages[-1].data)
    assert quality['maximum_cte_m']==.2118956446647644
    assert quality['cte_exceedance_sec'] > .4
    assert quality['cte_warning']
    status=json.loads(n.pubs[P+'/status'].messages[-1].data)
    assert status['policy_version']==2


def test_shutdown_publisher_error_cannot_leave_controller_alive(session):
    n,clock,ready,mod,Message=session
    def invalid_context(msg):raise RuntimeError('publisher context invalid after SIGINT')
    n.enable.publish=invalid_context
    assert n.controller.active
    n.close()
    assert not n.controller.active
    assert not n.trace.thread.is_alive()
    assert n.lock.closed
    n.close()  # Safe when a second shutdown/finally path calls cleanup again.
