"""Offline checks for the disposable sweep and ranking; no device required."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import sweep


class SweepTests(unittest.TestCase):
    def test_all_combinations_keep_40fps_and_exact_bottom_65_percent(self):
        jobs = sweep.plan(2, 20, 5, 40, .7, '')
        self.assertEqual(len(jobs), 28)
        for repeat in (1, 2):
            subset = [job for job in jobs if job['repeat'] == repeat]
            self.assertFalse(subset[0]['stereo'])
            self.assertFalse(subset[-1]['stereo'])
            stereo = [job for job in subset if job['stereo']]
            self.assertEqual(len({(j['width'], j['height'], j['lr_check'], j['median']) for j in stereo}), 12)
        for job in jobs:
            self.assertEqual(job['rgb_fps'], 80)
            self.assertEqual(job['depth_fps'], 40)
            self.assertEqual(job['roi_bottom_fraction'], .65)
            self.assertEqual(job['height'] * 65 % 100, 0)
        self.assertEqual(jobs, sweep.plan(2, 20, 5, 40, .7, ''))

    def result(self, name, rgb, depth, status='ok'):
        return dict(config={'name': name}, status=status,
                    rgb=dict(receive_hz=rgb, sequence_gaps=0, sensor_to_host_ms={'mean': 30}),
                    depth={'receive_hz': depth}, depth_valid_fraction_0_1_to_3m={'mean': .7})

    def test_faster_rgb_with_slow_depth_or_missing_repeat_does_not_win(self):
        names = [case['name'] for case in sweep.combinations()]
        results = [self.result(names[0], 79, 20), self.result(names[0], 80, 20),
                   self.result(names[1], 70, 40), self.result(names[1], 72, 39.9),
                   self.result(names[2], 80, 40)]
        ranked = sweep.ranking(results, 2)
        self.assertEqual(ranked[0]['name'], names[1])
        self.assertTrue(ranked[0]['depth_40fps_eligible'])
        self.assertEqual(ranked[0]['rgb_hz_mean'], 71)
        self.assertFalse(next(r for r in ranked if r['name'] == names[2])['depth_40fps_eligible'])
        with tempfile.TemporaryDirectory() as directory:
            sweep.save_summary(Path(directory), results, 2)
            self.assertEqual(json.loads((Path(directory) / 'results.json').read_text()), results)
            self.assertIn(names[1], (Path(directory) / 'ranking.csv').read_text())

    def test_failed_unsupported_case_is_retained_and_not_eligible(self):
        name = sweep.combinations()[0]['name']
        ranked = sweep.ranking([dict(config={'name': name}, status='error', error='unsupported size')], 1)
        row = next(r for r in ranked if r['name'] == name)
        self.assertFalse(row['depth_40fps_eligible'])
        self.assertIsNone(row['rgb_hz_mean'])

    def test_dry_run_does_not_build_or_open_camera(self):
        result = subprocess.run(['python3', str(sweep.HERE / 'sweep.py'), '--dry-run',
                                 '--sdk-prefix', '/nonexistent'], check=True, capture_output=True, text=True)
        self.assertEqual(len(json.loads(result.stdout)), 28)

    def test_execute_retains_errors_and_per_case_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            probe = root / 'probe'
            probe.write_text('#!/usr/bin/env python3\nimport sys, pathlib\n'
                             'print("unsupported camera mode")\n'
                             'pathlib.Path(sys.argv[2], "result.json").write_text(\'{"status":"error","error":"unsupported"}\')\n'
                             'sys.exit(1)\n')
            probe.chmod(0o755)
            config = sweep.plan(1, 1, 0, 40, 0, '')[1]
            result = sweep.execute(probe, config, root / 'case')
            self.assertEqual(result['status'], 'error')
            self.assertEqual(json.loads((root / 'case/config.json').read_text()), config)
            self.assertIn('unsupported camera mode', (root / 'case/console.log').read_text())


if __name__ == '__main__':
    unittest.main()
