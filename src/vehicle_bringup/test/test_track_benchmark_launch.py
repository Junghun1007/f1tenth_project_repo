"""Execute the production launch resolver with the repository YAMLs, without ROS."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SOURCE / 'auto_control'))


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context.launch_configurations[self.name]


class BenchmarkLaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = SOURCE / 'vehicle_bringup/launch/auto_drive.launch.py'
        tree = ast.parse(path.read_text())
        tree.body = [node for node in tree.body if not isinstance(node, ast.ImportFrom)]
        namespace = {'__file__': str(path), 'LaunchConfiguration': Configuration,
                     'get_package_share_directory': lambda package: str(SOURCE / package)}
        for name in ('LaunchDescription', 'DeclareLaunchArgument', 'EmitEvent', 'IncludeLaunchDescription',
                     'LogInfo', 'OpaqueFunction', 'RegisterEventHandler', 'IfCondition', 'UnlessCondition',
                     'OnProcessExit', 'Shutdown', 'PythonLaunchDescriptionSource', 'LoadComposableNodes',
                     'Node', 'ComposableNode', 'ParameterValue'):
            namespace[name] = type(name, (Action,), {})
        exec(compile(tree, str(path), 'exec'), namespace)
        cls.namespace = namespace

    def resolve(self, run):
        ns = self.namespace
        description = ns['generate_launch_description']()
        actions = description.args[0]
        config = {a.args[0]: a.kwargs['default_value'] for a in actions
                  if type(a).__name__ == 'DeclareLaunchArgument'}
        config.update(benchmark_run=str(run))
        context = SimpleNamespace(launch_configurations=config)
        opaque = next(a for a in actions if type(a).__name__ == 'OpaqueFunction')
        support = SimpleNamespace(
            model_check_node=lambda *args: Action(),
            startup_inspection_node=lambda *args: Action(),
            remote_preview_node=lambda *args: Action(),
            integrated_nodes=lambda *args: ([], []))
        with patch.dict(sys.modules, {'auto_control_tunnel.launch_support': support}):
            resolved = opaque.kwargs['function'](context, **opaque.kwargs['kwargs'])
        return context, resolved

    def test_presets_resolve_to_headless_perception_and_actual_driving_controls(self):
        for run in range(1, 5):
            with self.subTest(run=run):
                context, actions = self.resolve(run)
                config = context.launch_configurations
                self.assertEqual(config['effective_avoidance_control_enabled'], str(run >= 2).lower())
                self.assertEqual(config['effective_obstacle_slowdown_enabled'], str(run >= 2).lower())
                self.assertEqual(config['traffic_stop_enabled'], str(run >= 3).lower())
                self.assertEqual(config['startup_preview_image_enabled'], 'false')
                self.assertEqual(config['startup_inspection_active'], 'true')
                self.assertEqual(config['camera_preview_enabled'], 'false')
                self.assertEqual(config['dataset_collection_manual_capture_mode'], 'false')
                executables = [a.kwargs.get('executable') for a in actions]
                self.assertIn('track_benchmark', executables)
                self.assertNotIn('runtime_tools', executables)
                load = next(a for a in actions if type(a).__name__ == 'LoadComposableNodes')
                nodes = load.kwargs['composable_node_descriptions']
                names = [a.kwargs['name'] for a in nodes]
                self.assertEqual('auto_obstacles' in names, run >= 2)
                self.assertEqual('traffic_light_detector' in names, run >= 3)
                detector = nodes[0].kwargs['parameters'][0]
                self.assertFalse(detector['preview_enabled'])
                self.assertFalse(detector['profiling_enabled'])
                recorder = next(a for a in actions if a.kwargs.get('executable') == 'track_benchmark')
                params = recorder.kwargs['parameters'][0]
                self.assertEqual(params['rgb_result_topic'], detector['result_topic'])
                self.assertEqual(params['ir_result_topic'], '/ir_bev_line_detection/result')

    def test_normal_launch_keeps_gui_and_has_no_recorder(self):
        context, actions = self.resolve(0)
        executables = [a.kwargs.get('executable') for a in actions]
        self.assertNotIn('track_benchmark', executables)
        self.assertIn('runtime_tools', executables)
        self.assertEqual(context.launch_configurations['startup_preview_image_enabled'], 'true')


if __name__ == '__main__':
    unittest.main()
