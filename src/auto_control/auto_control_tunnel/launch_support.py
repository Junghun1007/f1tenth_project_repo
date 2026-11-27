"""Optional integration used only by vehicle_bringup tunnel_enabled:=true."""
import hashlib
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

# Web "차선 모델 (RGB·IR 공통)" lane.model -> start model of the (RGB, IR) detector, same numbers as settings.cpp.
# Files are in auto_control/models; 'baseline' = that detector's packaged default (RGB stop_0910 INT8 from
# line_detactor.yaml, IR = IR_BASELINE at tunnel_ir.yaml precision). auto_drive swaps live with the same table.
LANE_MODELS = {
    0: ('baseline', 'baseline'),
    1: (('fast_scnn_stop_line_v5_120x300_batch_1.int8.qdq.onnx', 'int8'), 'baseline'),
    2: (('lane_c3_120x300.int8.qdq.onnx', 'int8'),) * 2,
    3: (('lane_c3_120x300.onnx', 'fp32'),) * 2,
}
IR_BASELINE = 'ir_stereo_bev_120x300.onnx'   # line_detactor/models


def lane_model(drive_tuning):
    """Validated lane.model from the auto_drive YAML (the removed per-sensor engine_precision keys are ignored)."""
    value = drive_tuning.get('lane.model', 2)
    if value not in LANE_MODELS:
        raise ValueError(f'lane.model must be one of {sorted(LANE_MODELS)}')
    legacy = [k for k in ('lane.engine_precision', 'ir_lane.engine_precision') if drive_tuning.get(k, 0)]
    if legacy:
        print(f'[auto_drive] {", ".join(legacy)} is no longer used; lane.model={int(value)} selects both lane models')
    return int(value)


def bundled_model(path):
    """Path of a bundled lane ONNX, checked against the sha256 in its <name>.json when one is recorded."""
    path = Path(path)
    if not path.is_file():
        raise ValueError(f'lane model not found: {path}')
    meta = path.with_suffix('.json')
    if meta.is_file():
        report = json.loads(meta.read_text(encoding='utf-8'))
        expected = report.get('onnx_sha256') or report.get('output_onnx_sha256')
        if expected and hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'Bundled lane model checksum mismatch: {path.name}')
    return path


def ir_engine_cache(model, precision):
    # RGB and IR may load the same ONNX at the same precision: IR keeps its own engine file
    # (line_detactor uses the same name when it swaps the IR model live).
    return f'{model}.ir_line_detactor.{precision}.engine'


TCN_DEFAULTS = dict(threshold=0.5, cooldown_seconds=1.0, warmup_seconds=2.5,
                    max_gap_seconds=0.5, max_image_age_seconds=0.25)


def transition_options(launch_configurations):
    """Experimental tunnel transition switches (launch arguments only; default off = immediate TCN switch)."""
    prearm = str(launch_configurations.get('tunnel_prearm', 'off')).lower()
    if prearm not in ('off', 'entry', 'both'):
        raise ValueError('tunnel_prearm must be off, entry or both')
    handover = str(launch_configurations.get('tunnel_valid_handover', 'false')).lower()
    if handover not in ('true', 'false'):
        raise ValueError('tunnel_valid_handover must be true or false')
    score = float(launch_configurations.get('tunnel_prearm_score', 0.3))
    wait = float(launch_configurations.get('tunnel_valid_handover_max_sec', 0.5))
    if not 0 < score <= 1:
        raise ValueError('tunnel_prearm_score must be in (0, 1]')
    if not 0 < wait <= 5:
        raise ValueError('tunnel_valid_handover_max_sec must be in (0, 5]')
    return dict(prearm_entry=prearm != 'off', prearm_exit=prearm == 'both', prearm_score=score,
                valid_handover=handover == 'true', valid_handover_max_sec=wait)


def integrated_nodes(rgb_detector, controller, context):
    validation_text = str(context.launch_configurations.get('tunnel_validation_mode', 'true')).lower()
    if validation_text not in ('true', 'false'):
        raise ValueError('tunnel_validation_mode must be true or false')
    validation = validation_text == 'true'
    shares = {p: Path(get_package_share_directory(p)) for p in (
        'ir_camera_driver', 'line_detactor', 'auto_control', 'tunnel_tcn_test')}
    # Perception assets are shared by production and standalone tests. The
    # controller must not depend on a test package that depends on bringup.
    def parameters(path):
        return yaml.safe_load(path.read_text())['line_detactor']['ros__parameters']
    ir = parameters(shares['line_detactor'] / 'config/line_detactor.yaml')
    ir.update(parameters(shares['auto_control'] / 'config/tunnel_ir.yaml'))
    baseline, baseline_precision = bundled_model(shares['line_detactor'] / 'models' / IR_BASELINE), ir['engine_precision']
    choice = LANE_MODELS[controller.get('lane.model', 2)][1]
    model, precision = ((baseline, baseline_precision) if choice == 'baseline' else
                        (bundled_model(shares['auto_control'] / 'models' / choice[0]), choice[1]))
    ir.update(engine_precision=precision, engine_cache_path=ir_engine_cache(model, precision))
    padding = controller.get('ir_lane.result_padding_px', -1)
    if padding >= 0:
        ir['result_padding_px'] = int(padding)
    # SharedIrBev is the sole 30Hz gate. A second host-arrival gate loses
    # valid frames when camera/ROS delivery jitters around a 30Hz boundary.
    ir.update(model_path=str(model), inference_max_fps=0.0,
              preview_enabled=False, bev_theme_enable=False,
              shared_ir_preview_output_enabled=rgb_detector['preview_enabled'],
              preview_result_only_enabled=rgb_detector.get('preview_result_only_enabled', True))
    # RGB도 호스트 도착 시각 게이트를 두지 않는다 (터널 안 30 FPS 솎기는 camera_driver가 촬영 시각으로)
    rgb_detector.update({'tunnel.enabled': True, 'inference_max_fps': 0.0})
    age = controller.get('path_capture_maximum_age_sec', .2)
    rule_age = context.launch_configurations.get('path_capture_maximum_age_sec', age)
    selector_parameters = dict(validation_mode=validation, max_age_sec=min(.25, float(rule_age)),
                               rgb_result_topic=rgb_detector['result_topic'])
    # Match the actual controller YAML and resolved CLI geometry overrides.
    for name, fallback, cast in (
        ('path_minimum_points', 8, int), ('path_minimum_span_m', .12, float),
        ('path_minimum_x_m', .05, float), ('path_maximum_x_m', 2.20, float),
        ('path_maximum_gap_m', .15, float)):
        selector_parameters[name] = cast(context.launch_configurations.get(name, controller.get(name, fallback)))
    selector_parameters.update(transition_options(context.launch_configurations))
    selector_parameters.update(
        lane_result_frame_id=context.launch_configurations.get('ml_lane_frame_id', 'front_axle_bev'),
        bev_x_max_m=float(context.launch_configurations.get('ml_bev_x_max_m', 3.)),
        bev_y_max_m=float(context.launch_configurations.get('ml_bev_y_max_m', .6)),
        bev_meter_per_pixel=float(context.launch_configurations.get('ml_bev_meter_per_pixel', .01)))
    components = [
        ComposableNode(package='ir_camera_driver', plugin='ir_camera_driver::SharedIrBevNode',
                       name='shared_ir_bev', parameters=[{'validation_mode': validation, 'standby_fps': 0.0, 'active_fps': 30.0,
                                                          'model_input_topic': ir['input_topic']}]),
        ComposableNode(package='line_detactor', plugin='line_detactor::LineDetactorNode',
                       name='ir_line_detactor', parameters=[ir],
                       extra_arguments=[{'use_intra_process_comms': True}]),
        # C++ selector in the detector container: RGB/IR results arrive intra-process.
        ComposableNode(package='auto_control', plugin='auto_control::TunnelLaneSelector',
                       name='tunnel_lane_selector', parameters=[selector_parameters],
                       extra_arguments=[{'use_intra_process_comms': True}])]
    tcn_file = shares['tunnel_tcn_test'] / 'config/tunnel_tcn.yaml'
    tcn_defaults = {**TCN_DEFAULTS, **yaml.safe_load(tcn_file.read_text())['tunnel_tcn_test']['ros__parameters']}
    tcn_tuning = {name: float(controller.get('tcn.' + name, tcn_defaults[name])) for name in TCN_DEFAULTS}
    processes = [
        Node(package='tunnel_tcn_test', executable='tunnel_tcn_node', name='tunnel_tcn_test',
             output='screen', parameters=[str(tcn_file),
                                          {**tcn_tuning, 'stats_topic': '/tunnel/rgb_stats', 'initial_state': 'outside',
                                           'tuning_topic': '/tunnel/tcn_tuning',
                                           'preview_enabled': False}])]
    controller_overrides = {'tcn.' + name: value for name, value in tcn_tuning.items()}
    # auto_drive swaps the IR detector back to its packaged model when lane.model 0/1 is picked live
    controller_overrides.update({'model.ir_lane_baseline_path': str(baseline),
                                 'model.ir_lane_baseline_precision': str(baseline_precision)})
    return components, processes, controller_overrides


def model_check_node(tunnel_enabled, traffic_enabled):
    expected = ['line_detactor']
    if tunnel_enabled:
        expected += ['ir_line_detactor', 'tunnel_tcn_test']
    if traffic_enabled:
        expected += ['traffic_light_detector']
    return Node(package='auto_control', executable='model_startup_check',
                name='model_startup_check', output='screen',
                parameters=[{'expected_nodes': expected}])


def startup_inspection_node(tunnel_enabled, traffic_enabled,
                            connection_diagnostics_topic='/vesc/connected/diagnostics'):
    expected = ['line_detactor']
    if tunnel_enabled:
        expected += ['ir_line_detactor', 'tunnel_tcn_test']
    if traffic_enabled:
        expected += ['traffic_light_detector']
    return Node(package='auto_control', executable='startup_inspection',
                name='startup_inspection', output='screen', parameters=[{'expected_nodes': expected,
                    'connection_diagnostics_topic': connection_diagnostics_topic}])


def remote_preview_parameters(params_file, detector_lane_width_m=0.65):
    """Load remote_preview ros__parameters from a YAML file (node key `remote_preview` or `/**`)."""
    import yaml
    with open(params_file, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    section = {}
    for key in ('remote_preview', '/remote_preview', '/**'):
        if isinstance(data.get(key), dict):
            section = dict(data[key].get('ros__parameters') or {})
            break
    if float(section.get('lane_width_m', 0.0) or 0.0) <= 0.0:
        section['lane_width_m'] = float(detector_lane_width_m)
    return section


def remote_preview_node(tunnel_enabled, traffic_enabled, params_file, detector_lane_width_m=0.65,
                        connection_status_topic='/vesc/connected',
                        connection_diagnostics_topic='/vesc/connected/diagnostics'):
    expected = ['line_detactor']
    if tunnel_enabled:
        expected += ['ir_line_detactor', 'tunnel_tcn_test']
    if traffic_enabled:
        expected += ['traffic_light_detector']
    parameters = remote_preview_parameters(params_file, detector_lane_width_m)
    parameters.update({'expected_nodes': expected,
                       'connection_diagnostics_topic': connection_diagnostics_topic})
    return Node(package='auto_control', executable='remote_preview', name='remote_preview',
                output='screen', parameters=[parameters])
