#!/usr/bin/env python3
"""Standalone stationary OAK sweep. Uses only Python stdlib and an existing C++ SDK."""
import argparse
import csv
from datetime import datetime, timezone
import json
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


def combinations():
    return [dict(name=f'{width}x{height}_lr{int(lr)}_median{median}', stereo=True,
                 width=width, height=height, lr_check=lr, median=median)
            for width, height in SIZES for lr in (True, False) for median in ('3x3', 'off')]


def plan(repeats, duration, warmup, seed, dot_intensity, device_id):
    rng = random.Random(seed)
    baseline = dict(name='rgb_only', stereo=False, width=640, height=400, lr_check=True, median='3x3')
    jobs = []
    for repeat in range(1, repeats + 1):
        cases = combinations()
        rng.shuffle(cases)
        for case in [dict(baseline, name='rgb_only_before'), *cases, dict(baseline, name='rgb_only_after')]:
            jobs.append(dict(case, repeat=repeat, duration_sec=duration, warmup_sec=warmup,
                             rgb_fps=80, depth_fps=40, roi_bottom_fraction=.65,
                             dot_intensity=dot_intensity, device_id=device_id))
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


def ranking(results, repeats):
    rows = []
    for case in combinations():
        runs = [r for r in results if r['config']['name'] == case['name']]
        successful = [r for r in runs if r['status'] == 'ok']
        complete = len(successful) == repeats
        depth_hz = [r['depth']['receive_hz'] for r in successful]
        rgb_hz = [r['rgb']['receive_hz'] for r in successful]
        quality = [r['depth_valid_fraction_0_1_to_3m']['mean'] for r in successful
                   if r.get('depth_valid_fraction_0_1_to_3m')]
        rows.append(dict(name=case['name'], width=case['width'], input_height=case['height'],
                         roi_height=case['height'] * 65 // 100, lr_check=case['lr_check'], median=case['median'],
                         successful_repeats=len(successful),
                         depth_40fps_eligible=complete and min(depth_hz) >= 38,
                         rgb_hz_mean=statistics.mean(rgb_hz) if rgb_hz else None,
                         rgb_hz_min=min(rgb_hz) if rgb_hz else None,
                         depth_hz_min=min(depth_hz) if depth_hz else None,
                         rgb_latency_ms_mean=statistics.mean(r['rgb']['sensor_to_host_ms']['mean']
                                                            for r in successful) if successful else None,
                         rgb_sequence_gaps=sum(r['rgb']['sequence_gaps'] for r in successful),
                         valid_fraction_mean=statistics.mean(quality) if quality else None))
    return sorted(rows, key=lambda r: (not r['depth_40fps_eligible'],
                                      -(r['rgb_hz_mean'] or 0), r['rgb_latency_ms_mean'] or float('inf')))


def save_summary(root, results, repeats):
    (root / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    rows = ranking(results, repeats)
    with (root / 'ranking.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-prefix', type=Path, default=Path('/usr/local'))
    parser.add_argument('--build-dir', type=Path, default=HERE / '.build')
    parser.add_argument('--output', type=Path, default=HERE / 'results')
    parser.add_argument('--duration', type=float, default=20, help='Measured seconds per case (default 20)')
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--seed', type=int, default=40)
    parser.add_argument('--device-id', default='')
    parser.add_argument('--dot-intensity', type=float, default=.7)
    parser.add_argument('--dry-run', action='store_true', help='Print fixed settings and jobs without building/opening OAK')
    args = parser.parse_args()
    if not (1 <= args.duration <= 300 and 0 <= args.warmup <= 120 and 1 <= args.repeats <= 10
            and 0 <= args.dot_intensity <= 1):
        parser.error('duration 1..300, warmup 0..120, repeats 1..10, dot-intensity 0..1 required')
    jobs = plan(args.repeats, args.duration, args.warmup, args.seed, args.dot_intensity, args.device_id)
    if args.dry_run:
        print(json.dumps(jobs, indent=2))
        return
    probe = build_probe(args.sdk_prefix.expanduser().resolve(), args.build_dir.expanduser().resolve())
    root = args.output.expanduser().resolve() / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    root.mkdir(parents=True)
    (root / 'plan.json').write_text(json.dumps(dict(started_utc=datetime.now(timezone.utc).isoformat(),
                                                   jobs=jobs, ranking_depth_min_hz=38), indent=2) + '\n')
    results = []
    device_id = args.device_id
    print(f'Output: {root}\nRGB 80 FPS, stereo 40 FPS, bottom 65% before matching, depth 0.1..3 m.', flush=True)
    try:
        for i, job in enumerate(jobs, 1):
            config = dict(job, device_id=device_id)
            directory = root / f'{i:02d}_r{job["repeat"]}_{job["name"]}'
            print(f'[{i}/{len(jobs)}] {job["name"]} repeat {job["repeat"]}', flush=True)
            result = execute(probe, config, directory)
            result.update(config=config, folder=directory.name)
            results.append(result)
            save_summary(root, results, args.repeats)
            if result['status'] == 'ok':
                device_id = result['device_id']
                print(f'  RGB {result["rgb"]["receive_hz"]:.2f} Hz, depth {result["depth"]["receive_hz"]:.2f} Hz', flush=True)
            else:
                print(f'  {result["status"]}: {result.get("error", "See console.log")}', flush=True)
                if not job['stereo']:
                    raise RuntimeError('RGB-only control failed; inspect its console.log before repeating the sweep')
            time.sleep(2)  # Allow device teardown before the next fresh process.
    except KeyboardInterrupt:
        print('Interrupted. Completed cases are preserved.', flush=True)
    finally:
        rows = save_summary(root, results, args.repeats)
    candidates = [r for r in rows if r['depth_40fps_eligible']]
    if candidates:
        best = candidates[0]
        print(f'Highest RGB reception among completed cases sustaining >=38 depth Hz: {best["name"]}, '
              f'{best["rgb_hz_mean"]:.2f} Hz. Inspect depth quality before applying to driving.')
    else:
        print('No complete combination sustained >=38 depth Hz in every repeat. Inspect ranking.csv and console.log files.')
    print(f'Results saved: {root}')


if __name__ == '__main__':
    main()
