"""One camera pipeline/bridge; no obstacles, tunnel or traffic inference."""
import os
from pathlib import Path
import tempfile
import fcntl
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from longitudinal_autotune.config import PREFIX as P, load, ros_parameters
from longitudinal_autotune.assembly import build
from longitudinal_autotune.storage import atomic_write, ros_yaml_dump
from longitudinal_autotune.launch_support import ShutdownOnce

_LOCK = None


def nodes(context):
    global _LOCK
    config = LaunchConfiguration('params_file').perform(context)
    document = load(config)
    direction = LaunchConfiguration('direction').perform(context)
    if direction not in ('cw', 'ccw'):
        raise ValueError('Choose direction:=cw or ccw; no automatic U-turn is performed')
    # Package/device isolation does not rely on namespacing alone.
    lock = Path.home()/'.ros/longitudinal_autotune.hardware.lock'
    lock.parent.mkdir(parents=True, exist_ok=True)
    _LOCK = lock.open('w')
    try: fcntl.flock(_LOCK, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError: raise RuntimeError('Another autotune hardware launch is active')
    import rclpy
    rclpy.init(args=[])
    probe = rclpy.create_node('autotune_preflight')
    try:
        # Existing production node names, in any namespace, are device owners.
        import time
        until = time.monotonic()+.8
        while time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=.1)
        owners = {'camera_driver', 'bev_processor', 'vesc_bridge_node', 'auto_control',
                  'ir_camera_driver', 'autotune_bridge', 'autotune_session', 'autotune_camera', 'autotune_bev'}
        conflicts = [(n, ns) for n, ns in probe.get_node_names_and_namespaces() if n in owners or n.startswith('autotune_controller_')]
        if conflicts: raise RuntimeError('Stop existing device/control nodes first: '+str(conflicts))
    finally:
        probe.destroy_node(); rclpy.shutdown()
    shares = {p: Path(get_package_share_directory(p)) for p in (
        'camera_driver', 'bev_processor', 'line_detactor', 'auto_control',
        'vehicle_bringup', 'vehicle_dynamics_monitor', 'joy_initializer')}
    parameter_files = {package: LaunchConfiguration(argument).perform(context).strip() for package, argument in (
        ('auto_control', 'auto_control_params_file'), ('camera_driver', 'camera_params_file'),
        ('bev_processor', 'bev_params_file'), ('line_detactor', 'line_detactor_params_file'))}
    params = build(shares, document, parameter_files)
    session_dir = Path(LaunchConfiguration('campaign_dir').perform(context)).expanduser().resolve()
    session_dir.mkdir(parents=True, exist_ok=True)
    # Generated immutable launch inputs; never overwrite a production YAML.
    fd, config_copy = tempfile.mkstemp(prefix='autotune_inputs_', suffix='.yaml', dir=session_dir)
    with os.fdopen(fd, 'w') as stream:
        stream.write(ros_yaml_dump(dict(document=document, controller=params['auto_control'], baseline_files=parameter_files,
                            perception={k:v for k,v in params.items() if k != 'auto_control'})))
    # Pass generated YAML files directly: launch_ros reparses dictionary strings,
    # and its temporary YAML can leave an axis such as y unquoted for rcl.
    parameter_root = Path(config_copy).with_suffix('')
    parameter_root.mkdir()
    def parameter_file(name, values):
        path = parameter_root/(name+'.yaml')
        atomic_write(path, {'/**': {'ros__parameters': values}}, yaml_file=True)
        return str(path)
    library = LaunchConfiguration('depthai_library_dir').perform(context).strip()
    if not library:
        library = (shares['bev_processor']/'depthai_runtime_dir.txt').read_text().strip()
    if not library or not Path(library).is_dir():
        raise ValueError('Matching build-time DepthAI library directory is required')
    env = {'LD_LIBRARY_PATH': os.pathsep.join(filter(None, (library, context.environment.get('LD_LIBRARY_PATH', ''))))}
    # BEV completes its one-time startup measurement before CAM_A opens, as in
    # RGB-only auto_drive. All later color capture uses this one container.
    container = ComposableNodeContainer(name='autotune_perception', namespace=P,
        package='rclcpp_components', executable='component_container_mt', output='screen', additional_env=env,
        composable_node_descriptions=[
            ComposableNode(package='bev_processor', plugin='bev_processor::BevProcessorNode',
                           name='autotune_bev', namespace=P, parameters=[parameter_file('bev', params['bev_processor'])], extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(package='camera_driver', plugin='camera_driver::CameraDriverNode',
                           name='autotune_camera', namespace=P, parameters=[parameter_file('camera', params['camera_driver'])], extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(package='line_detactor', plugin='line_detactor::LineDetactorNode',
                           name='autotune_lane', namespace=P, parameters=[parameter_file('lane', params['line_detactor'])],
                           remappings=[('/auto/maximum_speed_request', P+'/preview_speed_request_unused'),
                                       ('/auto/maximum_speed_state', P+'/controller/maximum_speed_state')], extra_arguments=[{'use_intra_process_comms': True}])])
    bridge = ros_parameters(shares['vehicle_bringup']/'config/vesc_config.yaml', '/**')
    bridge.update(port=LaunchConfiguration('vesc_port').perform(context))
    for k in ('duty', 'brake_current', 'servo_position', 'measured_erpm'):
        bridge[k+'_topic'] = P+'/vesc/'+k
    bridge['connection_status_topic'] = P+'/vesc/connected'
    manual = ros_parameters(shares['vehicle_bringup']/'config/manual_vesc_config.yaml', '/**')
    manual.update(joy_topic=P+'/joy', auto_enable_topic=P+'/operator_enabled',
        measured_erpm_topic=P+'/vesc/measured_erpm', connection_status_topic=P+'/vesc/connected',
        drive_mode_topic=P+'/drive/mode', current_duty_topic=P+'/drive/current_duty',
        current_brake_current_topic=P+'/drive/current_brake_current', gear_state_topic=P+'/drive/gear')
    for k in ('duty', 'brake_current', 'servo_position'):
        manual[k+'_topic'] = P+'/vesc/'+k
        manual['auto_'+k+'_topic'] = P+'/gate/'+k
    actions = [LogInfo(msg='Autotune: two stop lines, '+direction+'; motion starts only through /longitudinal_autotune/arm'),
        container,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(shares['joy_initializer']/'launch/joy.launch.py')),
                                 launch_arguments={'vehicle_namespace': P.strip('/'), 'device_name_contains': '8BitDo'}.items()),
        Node(package='manual_control', executable='auto_handover_node', name='autotune_handover', namespace=P,
             parameters=[parameter_file('handover', manual)], output='screen'),
        Node(package='vesc_bridge', executable='vesc_bridge_node', name='autotune_bridge', namespace=P,
             parameters=[parameter_file('bridge', bridge)], output='screen'),
        Node(package='vehicle_dynamics_monitor', executable='vehicle_dynamics_node', name='autotune_dynamics',
             namespace=P, parameters=[parameter_file('dynamics', params['vehicle_dynamics_monitor'])], output='screen'),
        Node(package='longitudinal_autotune', executable='autotune_session', name='autotune_session', namespace=P,
             parameters=[parameter_file('session', dict(inputs_file=config_copy, campaign_dir=str(session_dir), direction=direction,
                              model_path=params['line_detactor']['model_path']))], output='screen')]
    # A failed coordinator/perception/bridge must terminate the hardware launch.
    critical = [container, actions[3], actions[4], actions[5], actions[-1]]
    shutdown_once = ShutdownOnce()
    handlers = [RegisterEventHandler(OnProcessExit(target_action=a, on_exit=shutdown_once)) for a in critical]
    return handlers+actions


def generate_launch_description():
    share = Path(get_package_share_directory('longitudinal_autotune'))
    defaults = dict(params_file=str(share/'config/experiment.yaml'), direction='cw',
                    campaign_dir=str(Path.home()/'.ros/longitudinal_autotune/campaign'),
                    vesc_port='/dev/ttyTHS1', depthai_library_dir='',
                    auto_control_params_file='', camera_params_file='', bev_params_file='', line_detactor_params_file='')
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()],
                              OpaqueFunction(function=nodes)])
