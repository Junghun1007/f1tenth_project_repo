import json
import numpy as np
import pytest
from longitudinal_autotune.analysis import analyze, matrix, fit, rmse
from longitudinal_autotune.controller import Controller


def test_effective_model_identifies_known_excited_response():
    rng=np.random.default_rng(17)
    groups=[]
    beta=np.array([-.05,8.,-.4,-.3,-.08])
    for j in range(4):
        t=np.arange(0,10,.02)+j*100.
        g=dict(id=(j,),ct=t,cu=rng.uniform(.01,.10,len(t)),cb=rng.uniform(0,1.,len(t)),
               t=t[20:],v=rng.uniform(.2,.8,len(t)-20),k=rng.uniform(0,2.,len(t)-20))
        x,_=matrix(dict(g,a=np.zeros(len(t)-20)),.06)
        g['a']=x@beta
        groups.append(g)
    recovered=fit(groups[:3],.06)
    assert np.allclose(recovered,beta,atol=1e-9)
    assert rmse(groups[3:],.06,recovered)<1e-9
    assert rmse(groups[3:],0.,recovered)>.1


def test_unexcited_data_never_produces_vehicle_model(tmp_path):
    p=tmp_path/'empty.jsonl';p.write_text('')
    assert analyze([p])['status']=='insufficient_identifiable_data'
    t=np.arange(200)*.02
    g=dict(id=(0,),t=t,v=np.ones(200)*.4,k=np.zeros(200),a=np.zeros(200),ct=t,cu=np.ones(200)*.05,cb=np.zeros(200))
    with pytest.raises(ValueError): fit([g],0.)


def test_controller_restart_never_overlaps_children(tmp_path,monkeypatch):
    from longitudinal_autotune import controller as mod
    children=[];signals=[]
    class Child:
        def __init__(self,command,**kwargs):
            self.command=command;self.pid=len(children)+1;self.code=None
            children.append(self)
        def poll(self):return self.code
        def wait(self,timeout):self.code=0
    monkeypatch.setattr(mod,'controller_executable',lambda:'/mock/auto_control_node')
    monkeypatch.setattr(mod.subprocess,'Popen',Child)
    monkeypatch.setattr(mod.os,'killpg',lambda pid,sig:signals.append((pid,sig)))
    c=Controller(tmp_path);c.request({'enabled':False});assert c.tick()
    assert c.alive() and len(children)==1
    assert children[0].command[0]=='/mock/auto_control_node'
    assert 'ros2' not in children[0].command
    assert any('/controller/maximum_speed_request' in s for s in children[0].command)
    c.request({'enabled':False,'duty_rise_rate_per_sec':.03})
    assert not c.alive() and not c.tick() and len(children)==1
    children[0].code=0
    assert c.tick() and len(children)==2
    c.close()
    assert signals


def test_cleanup_signals_group_even_if_wrapper_already_exited(tmp_path,monkeypatch):
    import io
    import signal
    from types import SimpleNamespace
    from longitudinal_autotune import controller as mod
    signals=[]
    monkeypatch.setattr(mod.os,'killpg',lambda pid,sig:signals.append((pid,sig)))
    c=Controller(tmp_path)
    c.child=SimpleNamespace(pid=123456,poll=lambda:0)
    stream=c.log=io.StringIO()
    c.close()
    assert signals==[(123456,signal.SIGINT),(123456,signal.SIGKILL)]
    assert c.child is None and c.log is None and stream.closed


def test_cleanup_tolerates_a_process_group_which_already_disappeared(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from longitudinal_autotune import controller as mod
    def gone(pid,sig):raise ProcessLookupError()
    monkeypatch.setattr(mod.os,'killpg',gone)
    c=Controller(tmp_path);c.child=SimpleNamespace(pid=123456,poll=lambda:0)
    c.close()
    assert c.child is None


def test_actual_owned_process_exits_without_leaving_process_group(tmp_path,monkeypatch):
    import os
    import sys
    import time
    from longitudinal_autotune import controller as mod
    if os.name != 'posix':pytest.skip('Process-group lifecycle is POSIX-specific')
    binary=tmp_path/'fake_controller'
    ready=tmp_path/'controller.ready'
    binary.write_text('#!'+sys.executable+'\nimport signal,sys,time\nfrom pathlib import Path\n'
        'signal.signal(signal.SIGINT,lambda *args:sys.exit(0))\n'
        'Path('+repr(str(ready))+').write_text("ready")\n'
        'while True: time.sleep(.05)\n')
    binary.chmod(0o700)
    monkeypatch.setattr(mod,'controller_executable',lambda:str(binary))
    c=Controller(tmp_path)
    try:
        c.request({'enabled':False});assert c.tick()
        child=c.child
        until=time.monotonic()+3.
        while not ready.exists() and time.monotonic()<until:time.sleep(.01)
        assert ready.exists()
        assert os.getpgid(child.pid)==child.pid
        c.close()
        assert child.poll() is not None
        with pytest.raises(ProcessLookupError):os.killpg(child.pid,0)
    finally:
        c.close()
