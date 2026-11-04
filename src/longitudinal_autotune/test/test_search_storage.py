import copy
import json
import math
import pytest
import yaml
from longitudinal_autotune.search import Campaign
from longitudinal_autotune.storage import Trace


def results(score=10.,feasible=True):
    return [dict(feasible=feasible,score=score,voltage_v=None) for _ in range(4)]


def test_best_requires_both_directions_and_survives_interruption(tmp_path,document,baseline):
    c=Campaign(tmp_path,document,baseline,{'model':'abc'})
    c.submit('cw',results())
    assert c.state['best'] is None and not (tmp_path/'best_parameters.yaml').exists()
    with pytest.raises(ValueError): c.advance()
    c.submit('ccw',results());assert c.state['best']['id']==0
    saved=(tmp_path/'best_parameters.yaml').read_bytes()
    assert c.advance()
    c.submit('ccw',results(1.))
    resumed=Campaign(tmp_path,document,baseline,{'model':'abc'})
    assert resumed.needed()==['cw']
    assert (tmp_path/'best_parameters.yaml').read_bytes()==saved
    resumed.submit('cw',results(1.))
    assert resumed.state['best']['id']==1
    assert resumed.state['best']['score']==1.


def test_failed_direction_cannot_be_promoted(tmp_path,document,baseline):
    c=Campaign(tmp_path,document,baseline,{})
    c.submit('cw',results(1.,False))
    c.submit('ccw',results(1.))
    assert c.state['best'] is None and c.current['feasible'] is False
    assert not c.advance() and c.state['complete']


def test_bounds_budget_signature_and_voltage_mismatch(tmp_path,document,baseline):
    d=copy.deepcopy(document);d['search']['max_candidates']=3
    c=Campaign(tmp_path,d,baseline,{})
    c.submit('cw',results());c.submit('ccw',results())
    while c.advance():
        for k,v in c.current['values'].items():
            lo,hi=d['search']['bounds'][k];assert lo<=v<=hi
        c.submit('cw',results(9.));c.submit('ccw',results(9.))
    assert len(c.state['candidates'])==3
    with pytest.raises(ValueError): Campaign(tmp_path,d,baseline,{'model':'changed'})
    p=tmp_path/'v'
    c=Campaign(p,document,baseline,{})
    a,b=results(),results()
    for r in a:r['voltage_v']=12.
    for r in b:r['voltage_v']=10.
    c.submit('cw',a);c.submit('ccw',b)
    assert c.state['best'] is None


def test_trace_barrier_is_durable_and_finite(tmp_path):
    trace=Trace(tmp_path/'trace.jsonl')
    trace.put({'t':1.,'v':math.nan})
    assert trace.barrier().wait(2.)
    assert json.loads(trace.path.read_text())=={'t':1.,'v':None}
    trace.close()
    assert not trace.failed and not trace.dropped
