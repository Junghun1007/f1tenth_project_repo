"""Verify real launch assembly against transport doubles and installed-style files."""
import ast
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[2]


class LaunchTests(unittest.TestCase):
    def helper(self):
        modules = {name: ModuleType(name) for name in (
            'ament_index_python', 'ament_index_python.packages',
            'launch_ros', 'launch_ros.actions', 'launch_ros.descriptions')}
        modules['ament_index_python.packages'].get_package_share_directory = lambda name: str(SOURCE / name)
        modules['launch_ros.actions'].Node = lambda **kw: kw
        modules['launch_ros.descriptions'].ComposableNode = lambda **kw: kw
        path = SOURCE / 'tunnel_drive/tunnel_drive/launch_support.py'
        spec = importlib.util.spec_from_file_location('tunnel_drive._launch_test', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict('sys.modules', modules):
            spec.loader.exec_module(module)
        return module

    def test_shared_camera_integration_adds_no_camera_owner_or_actuators(self):
        rgb = dict(preview_enabled=False, result_topic='/custom/rgb_result')
        context = NS(launch_configurations=dict(path_minimum_points='12'))
        components, processes = self.helper().integrated_nodes(rgb, {}, context)
        self.assertEqual(rgb['inference_max_fps'], 80.)
        self.assertTrue(rgb['tunnel.enabled'])
        self.assertEqual([c['plugin'] for c in components],
                         ['ir_camera_driver::SharedIrBevNode', 'line_detactor::LineDetactorNode'])
        self.assertEqual(components[0]['parameters'][0], dict(standby_fps=5., active_fps=30.))
        ir = components[1]['parameters'][0]
        self.assertFalse(ir['direct_bev_input_enabled'])
        self.assertEqual(ir['inference_max_fps'], 30.)
        self.assertIn('ir_stereo_bev_120x300.onnx', ir['model_path'])
        self.assertEqual([p['executable'] for p in processes],
                         ['mono_bev_adapter', 'tunnel_tcn_node', 'tunnel_lane_selector'])
        self.assertEqual(processes[-1]['parameters'][0]['minimum_points'], 12)
        self.assertEqual(processes[-1]['parameters'][0]['rgb_result_topic'], '/custom/rgb_result')
        self.assertEqual(processes[1]['parameters'][-1]['initial_state'], 'outside')
        self.assertFalse(processes[1]['parameters'][-1]['preview_enabled'])

    def wrapper(self, speed):
        path = SOURCE / 'tunnel_drive/launch/tunnel_drive.launch.py'
        function = next(n for n in ast.parse(path.read_text()).body
                        if isinstance(n, ast.FunctionDef) and n.name == 'start')
        values = dict(speed_mps=speed, auto_enabled='false')
        class Configuration:
            def __init__(self, name): self.name = name
            def perform(self, _): return values[self.name]
        import math
        namespace = dict(math=math, Path=Path, LaunchConfiguration=Configuration,
                         get_package_share_directory=lambda name: str(SOURCE / name),
                         PythonLaunchDescriptionSource=lambda path: path,
                         IncludeLaunchDescription=lambda source, launch_arguments:
                         dict(source=source, args=dict(launch_arguments)))
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
        return namespace['start'](None)[0]['args']

    def test_requested_speed_has_no_point_three_ceiling_and_starts_disabled(self):
        args = self.wrapper('1.5')
        self.assertEqual(args['minimum_speed_mps'], '1.5')
        self.assertEqual(args['maximum_speed_mps'], '1.5')
        self.assertEqual(args['auto_enabled'], 'false')
        self.assertEqual(args['tunnel_enabled'], 'true')
        self.assertNotIn('maximum_speed_mps', self.wrapper('inherit'))
        for value in ('0', '-1', 'nan', 'inf'):
            with self.assertRaises(ValueError):
                self.wrapper(value)


if __name__ == '__main__':
    unittest.main()
