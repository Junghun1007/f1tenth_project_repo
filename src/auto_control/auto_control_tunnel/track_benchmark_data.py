"""Bounded, one-shot track captures and experiment settings (no ROS dependency)."""
from collections import Counter
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import platform


SCENARIOS = {
    1: {'traffic': False, 'obstacles': False, 'tunnel': False},
    2: {'traffic': False, 'obstacles': True, 'tunnel': False},
    3: {'traffic': True, 'obstacles': True, 'tunnel': False},
    4: {'traffic': True, 'obstacles': True, 'tunnel': True},
}


def benchmark_overrides(run):
    features = SCENARIOS[run]
    return {
        'traffic_light_enabled': str(features['traffic']).lower(),
        'obstacles_enabled': str(features['obstacles']).lower(),
        'tunnel_enabled': str(features['tunnel']).lower(),
        'avoidance_test_enabled': str(features['obstacles']).lower(),
        'avoidance_control_enabled': str(features['obstacles']).lower(),
        'traffic_stop_enabled': str(features['traffic']).lower(),
        'preview_enabled': 'false', 'remote_preview_enabled': 'false',
        'preview_result_only_enabled': 'true', 'profiling_enabled': 'false',
        'dataset_collection_enabled': 'false', 'dataset_collection_manual_capture_mode': 'false',
        'camera_preview_enabled': 'false',
    }


class TrackCapture:
    """Single executor owns this buffer. A detached snapshot can be saved by a worker."""
    def __init__(self, run, duration=60.0, max_events=100000):
        if run not in SCENARIOS or duration <= 0 or max_events <= 0:
            raise ValueError('Invalid track capture settings')
        self.run, self.duration, self.max_events = run, duration, max_events
        self.started = None
        self.finished = False
        self.events = []
        self.dropped = Counter()
        self.started_utc = None

    def start(self, now):
        if self.started is not None:
            return False
        self.started = now
        self.started_utc = datetime.now(timezone.utc).isoformat()
        return True

    def add(self, now, kind, data, ros_ns=None):
        if self.started is None or self.finished or not 0 <= now - self.started < self.duration:
            return
        if len(self.events) >= self.max_events:
            self.dropped[kind] += 1
            return
        self.events.append({'elapsed_sec': now - self.started, 'ros_ns': ros_ns,
                            'kind': kind, 'data': data})

    def due(self, now):
        return self.started is not None and not self.finished and now - self.started >= self.duration

    def finish(self, now, reason):
        if self.started is None or self.finished:
            return None
        self.finished = True
        return {
            'schema': 1, 'run': self.run, 'features': SCENARIOS[self.run],
            'started_utc': self.started_utc, 'requested_duration_sec': self.duration,
            'recorded_duration_sec': min(self.duration, max(0.0, now - self.started)),
            'finish_reason': reason, 'events': self.events,
            'dropped_events': dict(self.dropped),
            'event_counts': dict(Counter(event['kind'] for event in self.events)),
            'host': platform.node(), 'platform': platform.platform(),
        }


def write_capture(directory, snapshot, parameters, parameter_errors):
    """Keep raw events and convenient tables; manifest is the completion marker."""
    directory = Path(directory)
    with (directory / 'events.jsonl').open('w', encoding='utf-8') as stream:
        for event in snapshot['events']:
            stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
    for kind, name in (('lane_rgb', 'lane_rgb.csv'), ('lane_ir', 'lane_ir.csv'),
                       ('runtime', 'runtime_metrics.csv'), ('system', 'system.csv')):
        rows = [dict(elapsed_sec=e['elapsed_sec'], ros_ns=e['ros_ns'], **e['data'])
                for e in snapshot['events'] if e['kind'] == kind]
        if not rows:
            continue
        columns = list(dict.fromkeys(key for row in rows for key in row))
        with (directory / name).open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            for row in rows:
                writer.writerow({k: json.dumps(v) if isinstance(v, (dict, list)) else v
                                 for k, v in row.items()})
    # Parameter responses can arrive after recording starts; preserve explicit failures.
    (directory / 'parameters.json').write_text(
        json.dumps({'nodes': parameters, 'errors': parameter_errors}, ensure_ascii=False, indent=2),
        encoding='utf-8')
    manifest = {key: value for key, value in snapshot.items() if key != 'events'}
    manifest['notes'] = [
        'Windowed metrics may overlap the capture boundary by one reporting window.',
        'Lane rows are received detector results; centerline_valid is recorded separately.',
        'camera/BEV stage durations include waiting; runtime timers measure their named scopes.',
        'Missing system fields mean unavailable, not zero.',
        'The recorder never sends vehicle start or stop commands.',
    ]
    temporary = directory / 'manifest.json.tmp'
    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(directory / 'manifest.json')
