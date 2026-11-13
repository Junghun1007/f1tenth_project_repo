#!/usr/bin/env python3
"""Standalone stationary OAK sweep. Uses only Python stdlib and an existing C++ SDK."""
import argparse
import csv
from datetime import datetime, timezone
import json
import itertools
import os
from pathlib import Path
import random
import signal
import statistics
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
SIZES = ((640, 400), (512, 320), (320, 200))


DEPTH_RATES = (10, 20, 30, 40)
TRANSPORTS = ('depth16', 'disparity8', 'spatial24')
RGB_CORRECTIONS = ('device', 'host_cpu')
ROI_WIDTHS = (1.0, 0.5)


def combinations():
    cases = []
    for size, lr, median, fps, transport, correction, roi in itertools.product(
            SIZES, (True, False), ('3x3', 'off'), DEPTH_RATES,
            TRANSPORTS, RGB_CORRECTIONS, ROI_WIDTHS):
        width, height = size
        name = f'{width}x{height}_lr{int(lr)}_m{median}_fps{fps}_{transport}_{correction}_roi{int(roi * 100)}'
        cases.append(dict(name=name, stereo=True, width=width, height=height,
                          lr_check=lr, median=median, depth_fps=fps, transport=transport,
                          rgb_correction=correction, roi_width_fraction=roi, policy='fixed'))
    return cases


def plan(repeats, duration, warmup, seed, dot_intensity, device_id):
    rng = random.Random(seed)
    jobs = []
    baseline = dict(combinations()[0], stereo=False)
    for repeat in range(1, repeats + 1):
        cases = combinations()
        rng.shuffle(cases)
        controls = [dict(baseline, name='rgb_only_device', rgb_correction='device'),
                    dict(baseline, name='rgb_only_host', rgb_correction='host_cpu')]
        for case in [*controls, *cases, dict(controls[0], name='rgb_only_after')]:
            jobs.append(dict(case, stage='screen', repeat=repeat, duration_sec=duration, warmup_sec=warmup,
                             rgb_fps=80, roi_bottom_fraction=.65, dot_intensity=dot_intensity, device_id=device_id))
    return jobs


def finalist_jobs(candidates, duration, warmup):
    script = (HERE / 'paired_gate.py').read_text()
    jobs = []
    for candidate in candidates:
        config = candidate['config']
        for policy in ('fixed', 'fps_burst', 'resolution_burst'):
            job = dict(config, name=config['name'] + '_' + policy, stage='confirm', policy=policy,
                       source_name=config['name'], duration_sec=duration, warmup_sec=warmup, repeat=1)
            if policy == 'fps_burst':
                job['depth_fps'] = min(30, config['depth_fps'])
            if policy != 'fixed':
                job['gate_script'] = script
            jobs.append(job)
    return jobs


def build_probe(prefix, build_dir):
    # Use the SAME SDK prefix as production, without the ROS underlay's DepthAI.
    configs = [prefix / 'lib/cmake/depthai', prefix / 'lib64/cmake/depthai',
               prefix / 'share/depthai/cmake']
    configs += list(prefix.glob('lib/*/cmake/depthai'))
    sdk = next((p for p in configs if (p / 'depthaiConfig.cmake').exists() or
                (p / 'depthai-config.cmake').exists()), None)
    if sdk is None:
        raise RuntimeError(f'DepthAI C++ SDK config missing under {prefix}. Use --sdk-prefix.')
    subprocess.run(['cmake', '-S', str(HERE), '-B', str(build_dir),
                    f'-Ddepthai_DIR={sdk}', f'-DCMAKE_PREFIX_PATH={prefix}',
                    '-DCMAKE_BUILD_TYPE=Release'], check=True)
    subprocess.run(['cmake', '--build', str(build_dir), '--parallel', '2'], check=True)
    return build_dir / 'camera_probe'


def execute(probe, config, directory):
    directory.mkdir()
    (directory / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
    timeout = config['duration_sec'] + config['warmup_sec'] + 90
    process = None
    environment = os.environ.copy()
    runtime_file = probe.parent / 'depthai_runtime_dir.txt'
    if runtime_file.exists():
        variable = 'DYLD_LIBRARY_PATH' if sys.platform == 'darwin' else 'LD_LIBRARY_PATH'
        environment[variable] = runtime_file.read_text().strip() + os.pathsep + environment.get(variable, '')
    try:
        with (directory / 'console.log').open('w') as log:
            process = subprocess.Popen([str(probe), str(directory / 'config.json'), str(directory)],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
            try:
                code = process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                # Kill/reap only this probe's process group so the next case can reopen OAK.
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                return {'status': 'timeout', 'error': f'Probe exceeded {timeout:g} seconds'}
    except BaseException:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        raise
    path = directory / 'result.json'
    if not path.exists():
        return {'status': 'error', 'error': f'Exit {code}, result.json missing; see console.log'}
    try:
        result = json.loads(path.read_text())
    except (ValueError, OSError) as error:
        return {'status': 'error', 'error': f'Cannot read result: {error}'}
    if code and result.get('status') == 'ok':
        result.update(status='error', error=f'Probe exited with code {code}')
    return result


def ranking(results, stage='screen'):
    rows = []
    grouped = {}
    for result in results:
        config = result['config']
        if config.get('stage', 'screen') != stage or not config['stereo']:
            continue
        grouped.setdefault(config['name'], []).append(result)
    for name, runs in grouped.items():
        config = runs[0]['config']
        successful = [r for r in runs if r['status'] == 'ok']
        depth_hz = [phase['depth']['receive_hz'] for r in successful
                    for phase in r.get('phases', [dict(depth=r['depth'])])]
        rgb_hz = [r['rgb']['receive_hz'] for r in successful]
        usable = [min(r['rgb']['receive_hz'], r.get('host_rgb', {}).get('processed_hz', float('inf')))
                  for r in successful]
        quality = [r['depth_valid_fraction_0_1_to_3m']['mean'] for r in successful
                   if r.get('depth_valid_fraction_0_1_to_3m')]
        rows.append(dict(name=name, policy=config.get('policy', 'fixed'),
                         width=config['width'], input_height=config['height'],
                         roi_width_fraction=config['roi_width_fraction'], roi_bottom_fraction=.65,
                         lr_check=config['lr_check'], median=config['median'],
                         requested_depth_fps=config['depth_fps'], transport=config['transport'],
                         rgb_correction=config['rgb_correction'], successful_repeats=len(successful),
                         depth_10fps_eligible=len(successful) == len(runs) and bool(depth_hz) and min(depth_hz) >= 10,
                         usable_rgb_hz=statistics.mean(usable) if usable else None,
                         rgb_hz_mean=statistics.mean(rgb_hz) if rgb_hz else None,
                         depth_hz_min=min(depth_hz) if depth_hz else None,
                         rgb_latency_ms_mean=statistics.mean(r['rgb']['sensor_to_host_ms']['mean'] for r in successful)
                                             if successful else None,
                         rgb_sequence_gaps=sum(r['rgb']['sequence_gaps'] for r in successful),
                         valid_fraction_mean=statistics.mean(quality) if quality else None))
    return sorted(rows, key=lambda r: (not r['depth_10fps_eligible'], -(r['usable_rgb_hz'] or 0),
                                      r['rgb_latency_ms_mean'] or float('inf')))


def select_finalists(results, count):
    ordered = [row for row in ranking(results) if row['depth_10fps_eligible']]
    selected = []
    # Preserve one candidate per output type/correction placement before filling spare slots.
    for transport, correction in itertools.product(TRANSPORTS, RGB_CORRECTIONS):
        match = next((r for r in ordered if r['transport'] == transport and r['rgb_correction'] == correction), None)
        if match and len(selected) < count:
            selected.append(match['name'])
    for row in ordered:
        if len(selected) >= count:
            break
        if row['name'] not in selected:
            selected.append(row['name'])
    by_name = {r['config']['name']: r for r in results if r['status'] == 'ok'}
    return [by_name[name] for name in selected]


def save_summary(root, results):
    temporary = root / 'results.json.tmp'
    temporary.write_text(json.dumps(results, indent=2) + '\n')
    temporary.replace(root / 'results.json')
    for stage in ('screen', 'confirm'):
        rows = ranking(results, stage)
        if not rows:
            continue
        with (root / f'{stage}_ranking.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-prefix', type=Path, default=Path('/usr/local'))
    parser.add_argument('--build-dir', type=Path, default=HERE / '.build')
    parser.add_argument('--output', type=Path, default=HERE / 'results')
    parser.add_argument('--screen-duration', type=float, default=5)
    parser.add_argument('--duration', type=float, default=60, help='Finalist measurement seconds (default 60)')
    parser.add_argument('--warmup', type=float, default=2)
    parser.add_argument('--repeats', type=int, default=1, help='Screening repeats (default 1)')
    parser.add_argument('--top', type=int, default=6, help='Finalists, each tested fixed/FPS-burst/resolution-burst')
    parser.add_argument('--seed', type=int, default=40)
    parser.add_argument('--device-id', default='')
    parser.add_argument('--dot-intensity', type=float, default=.7)
    parser.add_argument('--dry-run', action='store_true', help='Print plan without building/opening OAK')
    args = parser.parse_args()
    if not (1 <= args.duration <= 300 and 1 <= args.screen_duration <= 300 and 0 <= args.warmup <= 120
            and 1 <= args.repeats <= 10 and 1 <= args.top <= 30 and 0 <= args.dot_intensity <= 1):
        parser.error('Durations 1..300, warmup 0..120, repeats 1..10, top 1..30, dot-intensity 0..1 required')
    jobs = plan(args.repeats, args.screen_duration, args.warmup, args.seed, args.dot_intensity, args.device_id)
    document = dict(started_utc=datetime.now(timezone.utc).isoformat(), screen_jobs=jobs,
                    final_top=args.top, final_duration_sec=args.duration, final_policies=['fixed', 'fps_burst', 'resolution_burst'],
                    ranking_depth_min_hz=10,
                    screen_measurement_minutes=len(jobs) * (args.screen_duration + args.warmup) / 60,
                    max_final_measurement_minutes=args.top * 3 * (args.duration + args.warmup) / 60)
    if args.dry_run:
        print(json.dumps(document, indent=2))
        return
    probe = build_probe(args.sdk_prefix.expanduser().resolve(), args.build_dir.expanduser().resolve())
    root = args.output.expanduser().resolve() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    root.mkdir(parents=True)
    (root / 'plan.json').write_text(json.dumps(document, indent=2) + '\n')
    results = []
    device_id = args.device_id
    print(f'Output: {root}\nRGB 80 FPS, stereo 10/20/30/40 FPS, bottom 65%, depth 0.1..3 m.\n'
          f'Screen {len(jobs)} jobs, then up to {args.top * 3} confirmations. '
          f'Warmup+measurement about {document["screen_measurement_minutes"] + document["max_final_measurement_minutes"]:.0f} min; '
          'device reopen/build time is additional.', flush=True)

    def run_jobs(scheduled):
        nonlocal device_id
        for i, job in enumerate(scheduled, 1):
            config = dict(job, device_id=device_id)
            directory = root / f'{len(results) + 1:04d}_{job["stage"]}_{job["name"]}'
            print(f'[{job["stage"]} {i}/{len(scheduled)}] {job["name"]}', flush=True)
            result = execute(probe, config, directory)
            result.update(config=config, folder=directory.name)
            results.append(result)
            save_summary(root, results)
            if result['status'] == 'ok':
                device_id = result['device_id']
                print(f'  RGB {result["rgb"]["receive_hz"]:.2f} Hz, depth {result["depth"]["receive_hz"]:.2f} Hz', flush=True)
            else:
                print(f'  {result["status"]}: {result.get("error", "See console.log")}', flush=True)
                if job['name'] in ('rgb_only_device', 'rgb_only_after'):
                    raise RuntimeError('RGB-only device control failed; inspect console.log before repeating')
            time.sleep(2)
    try:
        run_jobs(jobs)
        selected = select_finalists(results, args.top)
        finalists = finalist_jobs(selected, args.duration, args.warmup)
        (root / 'finalists.json').write_text(json.dumps(finalists, indent=2) + '\n')
        run_jobs(finalists)
    except KeyboardInterrupt:
        print('Interrupted. Completed cases are preserved.', flush=True)
    finally:
        save_summary(root, results)
    candidates = [r for r in ranking(results, 'confirm') if r['depth_10fps_eligible']]
    if candidates:
        best = candidates[0]
        print(f'Highest usable RGB reception with >=10 depth/ROI reports per second in EVERY phase: '
              f'{best["name"]}, {best["usable_rgb_hz"]:.2f} Hz. Inspect depth quality and output type before driving.')
    else:
        print('No confirmed candidate sustained >=10 depth/ROI Hz in every phase. Inspect ranking and console.log files.')
    print(f'Results saved: {root}')


if __name__ == '__main__':
    main()
