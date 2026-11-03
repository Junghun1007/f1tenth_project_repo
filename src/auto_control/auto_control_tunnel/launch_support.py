"""Optional integration used only by vehicle_bringup tunnel_enabled:=true."""
import hashlib
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def integrated_nodes(rgb_detector, controller, context):
    shares = {p: Path(get_package_share_directory(p)) for p in (
        'ir_camera_driver', 'line_detactor', 'auto_control', 'tunnel_tcn_test')}
    # Perception assets are shared by production and standalone tests. The
    # controller must not depend on a test package that depends on bringup.
    def parameters(path):
        return yaml.safe_load(path.read_text())['line_detactor']['ros__parameters']
    ir = parameters(shares['line_detactor'] / 'config/line_detactor.yaml')
    ir.update(parameters(shares['auto_control'] / 'config/tunnel_ir.yaml'))
    model = shares['line_detactor'] / 'models/ir_stereo_bev_120x300.onnx'
    report = json.loads(model.with_suffix('.json').read_text())
    if hashlib.sha256(model.read_bytes()).hexdigest() != report['onnx_sha256']:
        raise ValueError('Bundled IR model checksum mismatch')
    # SharedIrBev is the sole 30Hz gate. A second host-arrival gate loses
    # valid frames when camera/ROS delivery jitters around a 30Hz boundary.
    ir.update(model_path=str(model), inference_max_fps=0.0,
              preview_enabled=False, bev_theme_enable=False,
              shared_ir_preview_output_enabled=rgb_detector['preview_enabled'],
              preview_result_only_enabled=rgb_detector.get('preview_result_only_enabled', True))
    rgb_detector.update({'tunnel.enabled': True, 'inference_max_fps': 80.0})
    age = controller.get('path_capture_maximum_age_sec', .2)
    rule_age = context.launch_configurations.get('path_capture_maximum_age_sec', age)
    selector_parameters = dict(max_age_sec=min(.25, float(rule_age)),
                               rgb_result_topic=rgb_detector['result_topic'])
    components = [
        ComposableNode(package='ir_camera_driver', plugin='ir_camera_driver::SharedIrBevNode',
                       name='shared_ir_bev', parameters=[{'standby_fps': 0.0, 'active_fps': 30.0,
                                                          'model_input_topic': ir['input_topic']}]),
        ComposableNode(package='line_detactor', plugin='line_detactor::LineDetactorNode',
                       name='ir_line_detactor', parameters=[ir],
                       extra_arguments=[{'use_intra_process_comms': True}])]
    processes = [
        Node(package='tunnel_tcn_test', executable='tunnel_tcn_node', name='tunnel_tcn_test',
             output='screen', parameters=[str(shares['tunnel_tcn_test'] / 'config/tunnel_tcn.yaml'),
                                          {'image_topic': '/tunnel/rgb_raw', 'initial_state': 'outside',
                                           'preview_enabled': False}]),
        Node(package='auto_control', executable='tunnel_lane_selector', name='tunnel_lane_selector',
             output='screen', parameters=[selector_parameters])]
    return components, processes


def model_check_node(tunnel_enabled, traffic_enabled):
    expected = ['line_detactor']
    if tunnel_enabled:
        expected += ['ir_line_detactor', 'tunnel_tcn_test']
    if traffic_enabled:
        expected += ['traffic_light_detector']
    return Node(package='auto_control', executable='model_startup_check',
                name='model_startup_check', output='screen',
                parameters=[{'expected_nodes': expected}])
