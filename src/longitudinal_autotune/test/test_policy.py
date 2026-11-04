import math
from dataclasses import replace
import numpy as np
import pytest
from longitudinal_autotune.policy import Experiment, Sample


def run_block(e, direction):
    trial = Experiment(e,direction)
    now,sequence = 10.,0
    trial.arm(now)
    events=[]
    def feed(speed=0.,target=0.,line=.12,yaw=0.,duty=0.,brake=.7):
        nonlocal now,sequence
        now+=.02;sequence+=1
        event=trial.update(Sample(now,speed,speed,target_speed=target,line=line,
                    lane_sequence=sequence,yaw_rate=yaw,duty=duty,brake=brake))
        if event: events.append(event)
        assert trial.phase != 'FAULT',trial.reason
    for _ in range(140):
        if not trial.anchor: break
        feed()
    assert not trial.anchor
    for segment in range(4):
        ts=np.arange(.02,12.001,.02)
        speeds=np.where(ts<2.,.2*(1-np.cos(math.pi*ts/2)),
                 np.where(ts>10.,.2*(1+np.cos(math.pi*(ts-10)/2)),.4))
        distance=float(np.sum(speeds)*.02)
        remaining=distance+.12
        sign=-1 if direction=='cw' else 1
        for speed in speeds:
            remaining-=float(speed)*.02
            feed(float(speed),float(speed),remaining if remaining<1.8 else math.nan,
                 sign*math.pi*float(speed)/distance,.06 if remaining>1. else 0.,0. if remaining>1. else .4)
        for _ in range(140):
            if trial.phase in ('RELEASE','BLOCK_DONE') and len(trial.results)>segment: break
            feed()
        assert len(trial.results)==segment+1,trial.phase
    assert trial.phase=='BLOCK_DONE'
    return trial,events


@pytest.mark.parametrize('direction',['cw','ccw'])
def test_two_lines_two_laps_and_anchor(document,direction):
    trial,events=run_block(document['experiment'],direction)
    assert len(events)==5 and events[0]['anchor']
    assert [r['line_id'] for r in trial.results]==[1,0,1,0]
    assert all(r['feasible'] for r in trial.results)
    assert all(abs(r['estimated_stop_distance_m']-.12)<.01 for r in trial.results)


def test_stale_lane_and_wrong_direction_stop(document):
    t=Experiment(document['experiment'],'cw');t.arm(1.)
    t.update(Sample(1.,.3,.3,yaw_rate=25.))
    t.update(Sample(1.02,.3,.3,yaw_rate=25.))
    assert t.reason=='wrong_travel_direction'
    t=Experiment(document['experiment'],'cw');t.arm(1.)
    t.update(Sample(1.,.1,.1,lane_ok=False))
    assert t.phase=='FAULT'


def test_duplicate_lane_cannot_confirm_line(document):
    t=Experiment(document['experiment'],'ccw');t.arm(1.)
    for n in range(8):
        t.update(Sample(1+n*.02,0.,0.,line=.2,lane_sequence=1))
    assert t.confirmations==1 and t.phase=='DRIVE'


def test_hold_does_not_hide_positive_drive(document):
    t=Experiment(document['experiment'],'ccw');t.arm(1.)
    for n in range(30):
        t.update(Sample(1+n*.02,0.,0.,line=.12,lane_sequence=n+1))
    assert t.phase=='HOLD'
    t.update(Sample(1.6,0.,0.,line=.12,lane_sequence=31,duty=.02))
    assert t.reason=='hold_motion_or_positive_drive'


def test_early_line_loss_and_overshoot_abort(document):
    for distance,reason in [(1.,'stop_line_lost_before_terminal'),(-.1,'stop_line_overshoot')]:
        t=Experiment(document['experiment'],'ccw');t.arm(1.)
        for n in range(3): t.update(Sample(1+n*.02,0.,0.,line=distance,lane_sequence=n+1))
        for n in range(3,25):
            t.update(Sample(1+n*.02,0.,0.,line=math.nan,lane_sequence=n+1))
        assert t.reason==reason


def test_interruption_never_finishes_block(document):
    t=Experiment(document['experiment'],'cw');t.arm(1.)
    t.fail('operator_stop')
    assert t.update(Sample(1.02,0.,0.)) is None
    assert not t.results and not t.running
