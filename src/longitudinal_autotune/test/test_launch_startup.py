"""Regression coverage for Humble parameter parsing and cascading shutdown."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as N
import time
import yaml
from conftest import ROOT, REPO
from longitudinal_autotune.launch_support import ShutdownOnce
from longitudinal_autotune.storage import atomic_write


def modules(monkeypatch, name, **attrs):
    module = ModuleType(name)
    module.__dict__.update(attrs)
    monkeypatch.setitem(sys.modules, name, module)


class Action:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


def test_ros_parameter_yaml_quotes_ambiguous_string_values(tmp_path):
    path = tmp_path/'parameters.yaml'
    values = dict(imu_yaw_axis='y', socket='n', mode='off', numeric_string='123',
                  empty='', axis_list=['y','n'], enabled=True, samples=3, gain=.15)
    atomic_write(path, {'/**': {'ros__parameters': values}}, yaml_file=True)
    assert yaml.safe_load(path.read_text())['/**']['ros__parameters'] == values
    # ROS rcl_yaml_param_parser recognizes bare y/n as BOOL, unlike PyYAML.
    # Assert scalar styles consumed by libyaml, not merely PyYAML's round trip.
    scalars = [e for e in yaml.parse(path.read_text()) if isinstance(e,yaml.events.ScalarEvent)]
    for word in ('y','n','off','123',''):
        assert any(e.value == word for e in scalars)
        assert all(e.style in ('"', "'") for e in scalars if e.value == word)
    assert any(e.value == 'true' and e.style is None for e in scalars)
    assert any(e.value == '3' and e.style is None for e in scalars)


def test_cascading_critical_exits_request_shutdown_only_once(monkeypatch):
    modules(monkeypatch, 'launch')
    modules(monkeypatch, 'launch.actions', EmitEvent=Action)
    modules(monkeypatch, 'launch.events', Shutdown=Action)
    stop = ShutdownOnce()
    context = N(is_shutdown=False)
    assert len(stop(N(returncode=1),context)) == 1
    assert stop(N(returncode=-2),context) == []
    context.is_shutdown = True
    assert stop(N(returncode=-9),context) == []
    assert ShutdownOnce()(N(returncode=0),context) == []


def test_launch_passes_parameter_files_without_reinterpreting_strings(monkeypatch,tmp_path):
    modules(monkeypatch, 'ament_index_python')
    modules(monkeypatch, 'ament_index_python.packages', get_package_share_directory=lambda p:str(REPO/'src'/p))
    modules(monkeypatch, 'launch', LaunchDescription=Action)
    modules(monkeypatch, 'launch.actions', **{k:Action for k in (
        'DeclareLaunchArgument','IncludeLaunchDescription','OpaqueFunction','LogInfo','RegisterEventHandler')})
    modules(monkeypatch, 'launch.event_handlers', OnProcessExit=Action)
    modules(monkeypatch, 'launch.launch_description_sources', PythonLaunchDescriptionSource=Action)
    class Configuration:
        def __init__(self,name): self.name=name
        def perform(self,context): return context.config[self.name]
    modules(monkeypatch, 'launch.substitutions', LaunchConfiguration=Configuration)
    modules(monkeypatch, 'launch_ros')
    modules(monkeypatch, 'launch_ros.actions', ComposableNodeContainer=Action, Node=Action)
    modules(monkeypatch, 'launch_ros.descriptions', ComposableNode=Action)
    probe = N(get_node_names_and_namespaces=lambda:[], destroy_node=lambda:None)
    modules(monkeypatch, 'rclpy', init=lambda **kw:None, shutdown=lambda:None,
            create_node=lambda name:probe, spin_once=lambda *a,**kw:None)
    monkeypatch.setattr(Path, 'home', classmethod(lambda cls:tmp_path))
    times = iter(range(100))
    monkeypatch.setattr(time, 'monotonic', lambda:next(times))
    spec = importlib.util.spec_from_file_location('autotune_launch_startup_test', ROOT/'launch/longitudinal_autotune.launch.py')
    launch = importlib.util.module_from_spec(spec); spec.loader.exec_module(launch)
    context = N(config=dict(params_file=str(ROOT/'config/experiment.yaml'), direction='cw',
                  campaign_dir=str(tmp_path/'campaign'), depthai_library_dir=str(tmp_path), vesc_port='/dev/ttyTHS1'),environment={})
    try:
        actions = launch.nodes(context)
        processes = [a for a in actions if 'parameters' in a.kwargs]
        container = next(a for a in actions if 'composable_node_descriptions' in a.kwargs)
        descriptions = container.kwargs['composable_node_descriptions']
        assert len(processes) == 4 and len(descriptions) == 3
        for action in processes+descriptions:
            # A dict here would cause launch_ros to regenerate unquoted YAML.
            parameter_path, = action.kwargs['parameters']
            assert isinstance(parameter_path,str) and Path(parameter_path).is_file()
            assert '/**' in yaml.safe_load(Path(parameter_path).read_text())
        dynamics = next(a for a in processes if a.kwargs['name'] == 'autotune_dynamics')
        path = Path(dynamics.kwargs['parameters'][0])
        assert yaml.safe_load(path.read_text())['/**']['ros__parameters']['imu_yaw_axis'] == 'y'
        axis = [e for e in yaml.parse(path.read_text()) if isinstance(e,yaml.events.ScalarEvent) and e.value == 'y']
        assert axis and all(e.style == '"' for e in axis)
        handlers = [a.args[0] for a in actions if a.args and 'on_exit' in getattr(a.args[0],'kwargs',{})]
        assert len(handlers) == 5
        assert all(h.kwargs['on_exit'] is handlers[0].kwargs['on_exit'] for h in handlers)
    finally:
        if launch._LOCK: launch._LOCK.close()
