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
    monkeypatch.setattr(mod.subprocess,'Popen',Child)
    monkeypatch.setattr(mod.os,'killpg',lambda pid,sig:signals.append((pid,sig)))
    c=Controller(tmp_path);c.request({'enabled':False});assert c.tick()
    assert c.alive() and len(children)==1
    assert any('/controller/maximum_speed_request' in s for s in children[0].command)
    c.request({'enabled':False,'duty_rise_rate_per_sec':.03})
    assert not c.alive() and not c.tick() and len(children)==1
    children[0].code=0
    assert c.tick() and len(children)==2
    c.close()
    assert signals
