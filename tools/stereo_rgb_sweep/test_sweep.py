"""Offline checks for the disposable sweep and ranking; no device required."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import sweep


class SweepTests(unittest.TestCase):
    def test_all_combinations_cover_rates_transports_and_fixed_roi(self):
        jobs = sweep.plan(1, 5, 2, 40, .7, '')
        stereo = [j for j in jobs if j['stereo']]
        self.assertEqual(len(jobs), 579)
        self.assertEqual(len(stereo), 576)
        self.assertEqual({j['depth_fps'] for j in stereo}, {10, 20, 30, 40})
        self.assertEqual({j['transport'] for j in stereo}, set(sweep.TRANSPORTS))
        self.assertEqual({j['roi_width_fraction'] for j in stereo}, {.5, 1.0})
        self.assertEqual(len({j['name'] for j in stereo}), 576)
        for job in jobs:
            self.assertEqual(job['rgb_fps'], 80)
            self.assertEqual(job['roi_bottom_fraction'], .65)
        self.assertEqual(jobs, sweep.plan(1, 5, 2, 40, .7, ''))

    def result(self, index, rgb, depth, status='ok', host=None):
        config = dict(sweep.combinations()[index], stage='screen')
        result = dict(config=config, status=status,
                      rgb=dict(receive_hz=rgb, sequence_gaps=0, sensor_to_host_ms={'mean': 30}),
                      depth={'receive_hz': depth}, depth_valid_fraction_0_1_to_3m={'mean': .7})
        if host is not None:
            result['host_rgb'] = {'processed_hz': host}
        return result

    def test_under_ten_depth_and_slow_host_correction_do_not_win(self):
        results = [self.result(0, 80, 9), self.result(1, 70, 10), self.result(2, 80, 40, host=20)]
        ranked = sweep.ranking(results)
        self.assertEqual(ranked[0]['name'], results[1]['config']['name'])
        self.assertTrue(ranked[0]['depth_10fps_eligible'])
        self.assertEqual(ranked[0]['usable_rgb_hz'], 70)
        with tempfile.TemporaryDirectory() as directory:
            sweep.save_summary(Path(directory), results)
            self.assertEqual(json.loads((Path(directory) / 'results.json').read_text()), results)
            self.assertTrue((Path(directory) / 'screen_ranking.csv').exists())

    def test_failed_case_is_retained_and_not_eligible(self):
        result = self.result(0, 80, 40, status='error')
        row = sweep.ranking([result])[0]
        self.assertFalse(row['depth_10fps_eligible'])
        self.assertIsNone(row['rgb_hz_mean'])

    def test_dry_run_does_not_build_or_open_camera(self):
        result = subprocess.run(['python3', str(sweep.HERE / 'sweep.py'), '--dry-run',
                                 '--sdk-prefix', '/nonexistent'], check=True, capture_output=True, text=True)
        plan = json.loads(result.stdout)
        self.assertEqual(len(plan['screen_jobs']), 579)
        self.assertEqual(plan['ranking_depth_min_hz'], 10)

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
