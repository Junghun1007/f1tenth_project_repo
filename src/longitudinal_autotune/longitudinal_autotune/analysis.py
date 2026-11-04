"""Offline effective response fit; never changes a live controller."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from .storage import atomic_write

FEATURES = ['offset_mps2', 'duty_gain_mps2_per_duty', 'brake_gain_mps2_per_a',
            'speed_drag_per_sec', 'turn_drag_gain']


def prepare(paths):
    groups = []
    for path in paths:
        records = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
        commands = [r for r in records if r['kind'] == 'gate_motor' and r.get('permitted')]
        commands.sort(key=lambda r:r['t'])
        if not commands: continue
        ct = np.array([r['t'] for r in commands])
        cu = np.array([r['value'] if r['mode'] == 'duty' else 0. for r in commands])
        cb = np.array([r['value'] if r['mode'] == 'brake_current' else 0. for r in commands])
        by_segment = {}
        for r in records:
            if r['kind'] != 'sample': continue
            s = r['data']
            if s.get('anchor', True) or s['phase'] not in ('RELEASE','DRIVE','APPROACH'): continue
            if not all(s[k] for k in ('lane_ok','telemetry_ok','command_ok')): continue
            if not all(s.get(k) is not None and math.isfinite(s[k]) for k in ('raw_speed','yaw_rate','t')): continue
            key = (r['run'], r['candidate'], r['direction'], s['segment'])
            by_segment.setdefault(key, []).append(s)
        for key, rows in by_segment.items():
            if len(rows) < 40: continue
            t = np.array([r['t'] for r in rows]); v = np.array([r['raw_speed'] for r in rows])
            k = np.array([abs(r['yaw_rate'])/max(abs(r['raw_speed']),.1) for r in rows]); dt = np.diff(t)
            # Fixed 80ms filter matches scoring, not a tunable smoothing knob.
            vf = v.copy()
            for i in range(1,len(v)):
                vf[i] = vf[i-1]+(1-math.exp(-max(0.,dt[i-1])/.08))*(v[i]-vf[i-1])
            a = np.diff(vf)/np.maximum(dt,1e-9)
            valid = (dt >= .005)&(dt <= .05)&(v[1:] > .10)&(np.abs(a) < 4.)
            # Exclude intervals after a sample gap; derivatives there are unreliable.
            for index in np.flatnonzero(dt > .05): valid[index:min(index+10,len(valid))] = False
            if np.sum(valid) < 30: continue
            groups.append(dict(id=key, t=t[1:][valid], v=vf[1:][valid], k=k[1:][valid], a=a[valid], ct=ct, cu=cu, cb=cb))
    return groups


def matrix(group, delay):
    index = np.searchsorted(group['ct'], group['t']-delay, side='right')-1
    valid = (index >= 0)
    index = np.maximum(index,0)
    valid &= group['t']-delay-group['ct'][index] <= .15
    v = group['v'][valid]
    return np.column_stack([np.ones(len(v)),group['cu'][index[valid]],group['cb'][index[valid]],
                            v,v*v*group['k'][valid]]), group['a'][valid]


def fit(groups, delay):
    pairs = [matrix(g,delay) for g in groups]
    x = np.concatenate([p[0] for p in pairs]); y = np.concatenate([p[1] for p in pairs])
    scale = np.sqrt(np.mean(x*x,axis=0))
    if len(y) < 100 or np.any(scale < 1e-5): raise ValueError('Insufficient actuator/turn excitation')
    z = x/scale
    if np.linalg.matrix_rank(z) < len(FEATURES) or np.linalg.cond(z) > 1e5:
        raise ValueError('Response parameters are not identifiable from this data')
    beta = np.linalg.lstsq(z,y,rcond=None)[0]/scale
    if beta[1] <= 0 or beta[2] >= 0 or beta[3] > 0:
        raise ValueError('Nonphysical effective gains; collect more varied valid trials')
    if np.std(x[:,1]) < .003 or np.count_nonzero(x[:,2] > .1) < 10:
        raise ValueError('Need both varying drive duty and service-brake observations')
    return beta


def rmse(groups, delay, beta):
    # Equal weight per segment so a long slow run cannot dominate validation.
    errors = []
    for g in groups:
        x,y = matrix(g,delay)
        if len(y): errors.append(float(np.mean((x@beta-y)**2)))
    return math.sqrt(float(np.mean(errors))) if errors else math.inf


def analyze(paths):
    groups = prepare(paths)
    report = dict(status='insufficient_identifiable_data', segment_groups=len(groups),
        features=FEATURES, model='dv/dt = b0 + b1*duty(t-delay) + b2*brake(t-delay) + b3*v + b4*v^2*abs(yaw_rate)/max(v,0.1)',
        domain='Forward low-speed effective response; not mass, tire friction, or a full vehicle model',
        speed_source='ERPM/drivetrain conversion; no independent wheel-slip or absolute-distance validation',
        delay_resolution_sec=.02, acceleration_filter_tau_sec=.08,
        delay_interpretation='Combined fitted motor/measurement-filter delay, not a physical actuator-only delay',
        energy_claim='Duty reduction is labeled coast/reduced drive; regenerative energy is not inferred from command alone')
    if len(groups) < 4: return report
    holdout_count = max(1,len(groups)//4)
    train, holdout = groups[:-holdout_count], groups[-holdout_count:]
    candidates = []
    for delay in np.arange(0.,.241,.02):
        try:
            errors = []
            for i in range(len(train)):
                beta = fit(train[:i]+train[i+1:],delay)
                errors.append(rmse([train[i]],delay,beta))
            candidates.append((float(np.mean(errors)),float(delay)))
        except ValueError: continue
    if not candidates:
        report['reason'] = 'Drive/brake/speed/turn excitation or physical-sign checks failed'
        return report
    cv,delay = min(candidates)
    try:
        beta = fit(train,delay)
    except ValueError as error:
        report['reason'] = str(error)
        return report
    report.update(status='effective_model_candidate', coefficients=dict(zip(FEATURES,map(float,beta))),
                  fitted_delay_sec=delay, training_group_cv_rmse_mps2=cv,
                  held_out_acceleration_rmse_mps2=rmse(holdout,delay,beta),
                  training_groups=[g['id'] for g in train], held_out_groups=[g['id'] for g in holdout],
                  speed_domain_mps=[float(min(np.min(g['v']) for g in groups)),float(max(np.max(g['v']) for g in groups))],
                  deployment='Offline candidate only. Validate independent runs before simulation/control use.')
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', action='append', required=True, help='Repeat for multiple runs/directions')
    parser.add_argument('--output', required=True)
    options = parser.parse_args(args)
    report = analyze(options.trace)
    atomic_write(options.output,report)
    print(json.dumps(report,indent=2))
