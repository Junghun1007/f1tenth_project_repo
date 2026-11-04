"""Bounded coordinate pattern search with repeated, paired-direction validation."""
import math
import statistics
from .config import fingerprint
from .storage import atomic_write


class Campaign:
    def __init__(self, root, settings, baseline, identity):
        from pathlib import Path
        import json
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.settings, self.identity = settings, identity
        self.path = self.root/'campaign.json'
        signature = fingerprint(dict(settings=settings, identity=identity, baseline=baseline))
        if self.path.exists():
            self.state = json.loads(self.path.read_text())
            if self.state['signature'] != signature:
                raise ValueError('Campaign configuration/controller/model identity changed; use a new campaign directory')
            self.save()  # Recover derived YAML/report files after an interrupted multi-file save.
        else:
            values = {k: float(baseline[k]) for k in settings['search']['bounds']}
            self.state = dict(signature=signature, candidates=[dict(id=0, values=values, reports={})],
                              best=None, cursor=0, step=settings['search']['initial_step_fraction'],
                              improved=False, complete=False)
            self.save()

    @property
    def current(self):
        return self.state['candidates'][-1]

    def needed(self):
        return [d for d in self.settings['experiment']['required_directions'] if d not in self.current['reports']]

    def save(self):
        atomic_write(self.path, self.state)
        atomic_write(self.root/'candidate_parameters.yaml', {'auto_control': {'ros__parameters': self.current['values']}}, True)
        best = self.state['best']
        if best is not None:
            atomic_write(self.root/'best_parameters.yaml', {'auto_control': {'ros__parameters': best['values']}}, True)
            atomic_write(self.root/'best_evaluation.json', dict(best=best, identity=self.identity,
                         status='best_validated_in_tested_domain', signature=self.state['signature']))

    def submit(self, direction, results, failure=None):
        if self.state['complete'] or direction not in self.needed():
            raise ValueError('Unexpected or duplicate directional block')
        e = self.settings['experiment']
        expected = 2 * e['laps_per_direction']
        complete = len(results) == expected
        good = complete and not failure and all(r['feasible'] and math.isfinite(r['score']) for r in results)
        voltages = [r['voltage_v'] for r in results if r.get('voltage_v') is not None]
        report = dict(feasible=good, results=results, failure=failure,
                      voltage_v=statistics.median(voltages) if voltages else None)
        self.current['reports'][direction] = report
        if not good:
            self.current['feasible'] = False
        elif not self.needed() and all(r['feasible'] for r in self.current['reports'].values()):
            reports = self.current['reports']
            v = [r['voltage_v'] for r in reports.values() if r['voltage_v'] is not None]
            if v and max(v)-min(v) > e['voltage_max_pair_delta_v']:
                # A confounded block is retained but not promoted as a winner.
                self.current.update(feasible=False, rejection='directional_voltage_mismatch')
            else:
                scores = [r['score'] for report in reports.values() for r in report['results']]
                score = statistics.mean(scores) + .3 * statistics.pstdev(scores)
                self.current.update(feasible=True, score=score)
                best = self.state['best']
                if best is None or score < best['score'] * (1-self.settings['search']['minimum_relative_improvement']):
                    import copy
                    self.state['best'] = copy.deepcopy(self.current)
                    self.state['improved'] = True
        self.save()

    def advance(self):
        if self.needed() and self.current.get('feasible') is not False:
            raise ValueError('Both directional blocks must be evaluated before proposing a candidate')
        search = self.settings['search']
        if self.state['best'] is None:
            # Failed baseline is not a license to explore unknown settings.
            self.state['complete'] = True
            self.current['rejection'] = 'baseline_not_validated'
            self.save(); return False
        if len(self.state['candidates']) >= search['max_candidates']:
            self.state['complete'] = True
            self.save(); return False
        bounds = list(search['bounds'].items())
        tried = {fingerprint(c['values']) for c in self.state['candidates']}
        while self.state['step'] >= search['minimum_step_fraction']:
            if self.state['cursor'] >= 2 * len(bounds):
                self.state['cursor'] = 0
                if not self.state['improved']:
                    self.state['step'] *= .5
                self.state['improved'] = False
                continue
            index = self.state['cursor']; self.state['cursor'] += 1
            name, (low, high) = bounds[index//2]
            values = dict(self.state['best']['values'])
            sign = 1 if index % 2 == 0 else -1
            values[name] = round(max(low, min(high, values[name] + sign*self.state['step']*(high-low))), 8)
            if fingerprint(values) in tried:
                continue
            self.state['candidates'].append(dict(id=len(self.state['candidates']), values=values, reports={}))
            self.save(); return True
        self.state['complete'] = True
        self.save(); return False
