"""auto_control 자동 주행 (cone_drive_test 주행부 통합).

    ros2 launch vehicle_bringup auto_drive.launch.py                    # 자동 주행 모드: 점검이 끝나면 브라우저 [주행 시작]
    ros2 launch vehicle_bringup auto_drive.launch.py mode:=detect       # 검출 테스트: 자동 명령 없음, 조이스틱 수동 주행
    ros2 launch vehicle_bringup auto_drive.launch.py tunnel_enabled:=true
    브라우저: http://<jetson-ip>:8765

한 컨테이너(bev_processor_container)에서 camera_driver → bev_processor → line_detactor(차선) → auto_control::AutoDriveNode
(YOLO 신호등·콘·차량, 콘 기억·회피, 신호등 정지, 앞차 추종, duty-only, 웹)가 영상·깊이·차선 결과를 복사 없이 주고받는다.
vesc/*는 safety_node만 발행한다 (MANUAL로 시작, [주행 시작] 승인 시 AUTO, X = 언제든 정지 후 MANUAL).
"""
import json
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


_PARAMETER_FILE_DEFAULT = "__PARAMETER_FILE_DEFAULT__"

# 생략하면 line_detactor YAML 값을 쓴다
_DETECTOR_POSTPROCESS_OVERRIDES = {
    "connection_ccl_serial_enabled": bool,
    "stop_line_mask_threshold": float,
    "stop_line_min_present_pixels": int,
    "stop_line_distance_enabled": bool,
    "stop_line_min_pixels": int,
    "stop_line_max_fit_samples": int,
    "stop_line_support_trim_quantile": float,
    "stop_line_support_margin_px": float,
    "stop_line_near_edge_quantile": float,
    "stop_line_min_crossing_alignment": float,
    "stop_line_fit_distance_tolerance_px": float,
    "stop_line_fit_angle_tolerance_rad": float,
}

# VESC 브리지·차량 동역학·안전 전환이 같이 쓰는 토픽 (루트)
_VESC = {
    "duty_topic": "/vesc/duty",
    "brake_current_topic": "/vesc/brake_current",
    "servo_position_topic": "/vesc/servo_position",
    "measured_erpm_topic": "/vesc/measured_erpm",
    "connection_status_topic": "/vesc/connected",
}
# 주행 컴포넌트 ↔ 안전 전환
_HANDOVER = {
    "auto_duty": "/auto_drive/auto/duty",
    "auto_brake_current": "/auto_drive/auto/brake_current",
    "auto_servo_position": "/auto_drive/auto/servo_position",
    "handover_mode": "/auto_drive/handover/mode",
    "handover_request": "/auto_drive/handover/request",
}


def _ros_parameters(config_path, node_name):
    with open(config_path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    for key in (node_name, "/" + node_name, "/**/" + node_name):
        if isinstance(config.get(key), dict):
            return dict(config[key].get("ros__parameters") or {})
    return {}


def _merged_parameters(base_path, selected_path, node_name):
    parameters = _ros_parameters(base_path, node_name)
    selected_path = os.path.abspath(os.path.expanduser(selected_path))
    if not os.path.isfile(selected_path):
        raise RuntimeError(f"parameter file not found: {selected_path}")
    if selected_path != os.path.abspath(base_path):
        parameters.update(_ros_parameters(selected_path, node_name))
    return parameters


def _bool_arg(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be true or false")
    return value == "true"


def _setup(context, *, bev_config, line_detactor_config, bev_launch):
    bev = _merged_parameters(bev_config, LaunchConfiguration("bev_params_file").perform(context), "bev_processor")
    detector = _merged_parameters(line_detactor_config, LaunchConfiguration("line_detactor_params_file").perform(context),
                                  "line_detactor")
    # 없는 YAML을 주면 카메라 측정·노드가 다 뜬 뒤에야 실패하므로, 아무것도 띄우기 전에 알려 주고 멈춘다
    for argument in ("drive_params_file", "perception_params_file", "bev_params_file",
                     "camera_params_file", "line_detactor_params_file"):
        path = os.path.expanduser(LaunchConfiguration(argument).perform(context))
        if not os.path.isfile(path):
            hint = (" (웹 파라미터 창에서 [YAML 저장]을 눌러야 생기는 파일입니다. 처음에는 이 인자 없이 실행하세요)"
                    if argument == "drive_params_file" else "")
            raise RuntimeError(f"{argument} 파일이 없습니다: {path}{hint}")
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("drive", "detect"):
        raise RuntimeError("mode must be drive or detect")
    tunnel = _bool_arg(context, "tunnel_enabled")
    preview = _bool_arg(context, "preview_enabled")
    namespace = LaunchConfiguration("vehicle_namespace").perform(context).strip("/")
    prefix = f"/{namespace}" if namespace else ""

    # 차선 BEV 기하: bev_processor 출력 = line_detactor 입력 = 앞차축 기준 BEV
    x_min, x_max = float(bev["x_min_m"]), float(bev["x_max_m"])
    y_min, y_max = float(bev["y_min_m"]), float(bev["y_max_m"])
    mpp = float(bev["meter_per_pixel"])
    if not all(math.isfinite(v) for v in (x_min, x_max, y_min, y_max, mpp)) or mpp <= 0:
        raise RuntimeError("BEV geometry must be finite with positive meter_per_pixel")
    if abs(x_min) > 1e-6 or not math.isclose(y_min, -y_max, abs_tol=1e-6) or min(x_max, y_max) <= 0:
        raise RuntimeError("auto drive requires front-axle BEV: x_min=0, symmetric +/-Y")
    width, height = int(bev["output_width"]), int(bev["output_height"])
    if width != round((y_max - y_min) / mpp) or height != round((x_max - x_min) / mpp):
        raise RuntimeError("BEV output size and metric extent disagree")
    if width != int(detector["model_input_width"]) or height != int(detector["model_input_height"]):
        raise RuntimeError("BEV output size must match the static line_detactor model input")
    for required in ("connection_enabled", "centerline_enabled", "result_publish_enabled"):
        if detector.get(required) is not True:
            raise RuntimeError(f"auto drive requires line_detactor {required}: true")
    detector.update({
        "input_topic": str(bev["output_topic"]),
        "direct_bev_input_enabled": True,
        "centerline_bev_width_m": y_max - y_min,
        "centerline_bev_height_m": x_max - x_min,
        # 화면은 브라우저가 그린다. preview_enabled:=true면 기존 Jetson 차선 창도 띄움
        "preview_enabled": preview,
        "preview_result_only_enabled": True,
        "bev_theme_enable": False,
        "obstacles.overlay_enabled": False,
    })
    for name, value_type in _DETECTOR_POSTPROCESS_OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value == _PARAMETER_FILE_DEFAULT:
            continue
        if value_type is bool:
            if value.lower() not in ("true", "false"):
                raise RuntimeError(f"{name} must be true or false")
            detector[name] = value.lower() == "true"
        else:
            detector[name] = value_type(value)
    context.launch_configurations["direct_host_copy_enabled"] = "true" if preview else "false"
    context.launch_configurations["ml_lane_frame_id"] = str(bev["output_frame_id"])
    context.launch_configurations["ml_bev_x_max_m"] = str(x_max)
    context.launch_configurations["ml_bev_y_max_m"] = str(y_max)
    context.launch_configurations["ml_bev_meter_per_pixel"] = str(mpp)
    lane_topic = str(detector["result_topic"])

    nodes = [ComposableNode(
        package="line_detactor", plugin="line_detactor::LineDetactorNode",
        name="line_detactor", parameters=[detector],
        extra_arguments=[{"use_intra_process_comms": True}],
    )]
    tunnel_actions = []
    if tunnel:
        if not (width == 120 and height == 300 and math.isclose(x_max, 3.0) and math.isclose(y_max, .6)
                and math.isclose(mpp, .01) and bev["output_frame_id"] == "front_axle_bev"):
            raise RuntimeError("Tunnel IR model requires 120x300 front-axle BEV at 1cm/pixel")
        from auto_control_tunnel.launch_support import integrated_nodes
        components, tunnel_actions = integrated_nodes(detector, {}, context)
        # RGB 검출기 설명을 다시 만든다 (ComposableNode는 파라미터를 복사해 둔다)
        nodes[0] = ComposableNode(
            package="line_detactor", plugin="line_detactor::LineDetactorNode",
            name="line_detactor", parameters=[detector],
            extra_arguments=[{"use_intra_process_comms": True}])
        nodes.extend(components)
        lane_topic = "/tunnel/lane_result"   # 선택기가 /rgb, /ir로 나눠 낸다

    camera = _ros_parameters(os.path.expanduser(LaunchConfiguration("camera_params_file").perform(context)), "camera_driver")
    drive_overrides = {
        "mode": mode,
        # BEV용 가상 확대 배율: auto_drive가 공유 fx·fy를 원본 영상 기준으로 되돌리는 데 씀
        "camera.fixed_view_zoom": float(camera.get("fixed_view_zoom", 1.25)),
        "tunnel.enabled": tunnel,
        "web.port": int(LaunchConfiguration("web_port").perform(context)),
        "topics.lane_result": lane_topic,
        "topics.measured_erpm": _VESC["measured_erpm_topic"],
        "topics.vesc_connected": _VESC["connection_status_topic"],
        "topics.joy": prefix + "/joy",
        "topics.imu": "/camera/imu",
        "topics.startup_status": "/bev_processor/startup_status",
        **{"topics." + key: value for key, value in _HANDOVER.items()},
    }
    speed = LaunchConfiguration("speed_mps").perform(context)
    if speed != "inherit":
        speed = float(speed)
        if not math.isfinite(speed) or speed <= 0:
            raise RuntimeError("speed_mps must be a positive number or inherit")
        drive_overrides["drive.maximum_speed_mps"] = speed
    # 주행 컴포넌트(웹 화면 포함)는 카메라 시작 측정보다 먼저 컨테이너에 올린다
    # → 실행하자마자 브라우저가 열리고 점검 영역에서 카메라 자세 측정 진행을 볼 수 있다
    context.launch_configurations["first_composable_nodes_json"] = json.dumps([{
        "package": "auto_control", "plugin": "auto_control::AutoDriveNode", "name": "auto_drive",
        "parameters": [os.path.expanduser(LaunchConfiguration("drive_params_file").perform(context)), drive_overrides],
    }])

    safety = Node(
        package="auto_control", executable="safety_node", name="auto_drive_safety", namespace=namespace, output="screen",
        parameters=[os.path.join(get_package_share_directory("auto_control"), "config", "safety.yaml"), {
            "initial_mode": "MANUAL",
            "joy_topic": prefix + "/joy",
            **_VESC,
            "current_duty_topic": "/drive/current_duty",              # vehicle_dynamics_monitor가 받는 실제 명령 duty
            "current_brake_current_topic": "/drive/current_brake_current",
            "gear_state_topic": prefix + "/manual/gear",
            "auto_duty_topic": _HANDOVER["auto_duty"],
            "auto_brake_current_topic": _HANDOVER["auto_brake_current"],
            "auto_servo_position_topic": _HANDOVER["auto_servo_position"],
            "drive_mode_topic": _HANDOVER["handover_mode"],
            "handover_request_topic": _HANDOVER["handover_request"],
            "auto_enable_topic": "/auto_drive/auto_enabled",
        }],
    )
    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("joy_initializer"), "launch", "joy.launch.py")),
        condition=IfCondition(LaunchConfiguration("joy_enabled")),
        launch_arguments={
            "vehicle_namespace": namespace,
            "device_name_contains": LaunchConfiguration("controller_name_contains").perform(context),
            "reconnect_interval_sec": "0.2",
        }.items(),
    )
    return [LogInfo(msg=(
        "[auto drive] mode=" + mode + " | tunnel=" + str(tunnel).lower() +
        " | lane=" + lane_topic + " | web=http://<jetson-ip>:" + LaunchConfiguration("web_port").perform(context) +
        " | drive_yaml=" + LaunchConfiguration("drive_params_file").perform(context) +
        " | detector_yaml=" + LaunchConfiguration("line_detactor_params_file").perform(context) +
        " | perception_yaml=" + LaunchConfiguration("perception_params_file").perform(context)
    )), safety, joy, *tunnel_actions, bev_launch, LoadComposableNodes(
        target_container="/bev_processor_container",
        composable_node_descriptions=nodes,
    )]


def generate_launch_description():
    bev_share = get_package_share_directory("bev_processor")
    auto_control_share = get_package_share_directory("auto_control")
    bev_config = os.path.join(bev_share, "config", "bev_config.yaml")
    line_detactor_config = os.path.join(get_package_share_directory("line_detactor"), "config", "line_detactor.yaml")
    camera_config = os.path.join(get_package_share_directory("camera_driver"), "config", "camera_config.yaml")
    dynamics_config = os.path.join(get_package_share_directory("vehicle_dynamics_monitor"), "config", "vehicle_dynamics.yaml")
    vesc_config = os.path.join(get_package_share_directory("vehicle_bringup"), "config", "vesc_config.yaml")

    # 카메라·BEV: 시작 측정 → camera_driver(깊이 CAM_A 정렬, 장착값 공유). 점검 창은 띄우지 않는다 (브라우저 점검 영역)
    bev_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bev_share, "launch", "bev_processor.launch.py")),
        launch_arguments={
            "startup_inspection_enabled": "false",
            "startup_preview_image_enabled": "false",
            "depthai_library_dir": LaunchConfiguration("depthai_library_dir"),
            "bev_params_file": LaunchConfiguration("bev_params_file"),
            "camera_params_file": LaunchConfiguration("camera_params_file"),
            "obstacles_enabled": "true",
            "perception_params_file": LaunchConfiguration("perception_params_file"),
            "tunnel_enabled": LaunchConfiguration("tunnel_enabled"),
            "tunnel_validation_mode": LaunchConfiguration("tunnel_validation_mode"),
            "tunnel_ir_imu_enabled": LaunchConfiguration("tunnel_ir_imu_enabled"),
            "measured_erpm_topic": _VESC["measured_erpm_topic"],
            "vehicle_acceleration_topic": "/vehicle/dynamics/acceleration",
            "preview_enabled": "false",
            "publish_enabled": "false",
            "direct_output_enabled": "true",
            "direct_host_copy_enabled": LaunchConfiguration("direct_host_copy_enabled"),
        }.items(),
    )
    vesc_bridge_node = Node(
        package="vesc_bridge", executable="vesc_bridge_node", name="vesc_bridge_node", output="screen",
        condition=IfCondition(LaunchConfiguration("vesc_enabled")),
        parameters=[vesc_config, {"port": LaunchConfiguration("vesc_port"), **_VESC}],
    )
    # 속도·가속도 (IMU 영상 안정화가 주행 중 차 가속도를 빼는 데 씀)
    dynamics_node = Node(
        package="vehicle_dynamics_monitor", executable="vehicle_dynamics_node", name="vehicle_dynamics_node", output="screen",
        condition=IfCondition(LaunchConfiguration("dynamics_enabled")),
        parameters=[dynamics_config, {
            "input_mode": LaunchConfiguration("input_mode"),
            "can_interface": LaunchConfiguration("can_interface"),
            "slcan_channel": LaunchConfiguration("slcan_channel"),
            "slcan_bitrate": ParameterValue(LaunchConfiguration("slcan_bitrate"), value_type=int),
            "can_controller_id": ParameterValue(LaunchConfiguration("can_controller_id"), value_type=int),
            "commanded_duty_topic": "/drive/current_duty",
            "measured_erpm_topic": _VESC["measured_erpm_topic"],
            "connection_status_topic": _VESC["connection_status_topic"],
            "servo_position_topic": _VESC["servo_position_topic"],
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="drive",
                              description="drive: 브라우저 [주행 시작]으로 자동 주행 | detect: 자동 명령 없음 (조이스틱 수동 주행)"),
        DeclareLaunchArgument("drive_params_file", default_value=os.path.join(auto_control_share, "config", "auto_drive.yaml"),
                              description="주행 컴포넌트 YAML (브라우저 [YAML 저장] 파일을 그대로 넣을 수 있음)"),
        DeclareLaunchArgument("perception_params_file", default_value=os.path.join(auto_control_share, "config", "perception.yaml"),
                              description="camera_driver 깊이·IMU, bev_processor 공유 설정"),
        DeclareLaunchArgument("speed_mps", default_value="inherit",
                              description="최대 속도 m/s (drive.maximum_speed_mps). inherit = YAML 값"),
        DeclareLaunchArgument("web_port", default_value="8765"),
        DeclareLaunchArgument("preview_enabled", default_value="false",
                              description="Jetson 화면에 line_detactor 차선 창도 띄움 (브라우저 화면은 항상 있음)"),
        DeclareLaunchArgument("vehicle_namespace", default_value="autopilot03"),
        DeclareLaunchArgument("vesc_enabled", default_value="true"),
        DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
        DeclareLaunchArgument("joy_enabled", default_value="true"),
        DeclareLaunchArgument("controller_name_contains", default_value="8BitDo"),
        DeclareLaunchArgument("dynamics_enabled", default_value="true"),
        DeclareLaunchArgument("input_mode", default_value="ros_topic"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("slcan_channel", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("slcan_bitrate", default_value="500000"),
        DeclareLaunchArgument("can_controller_id", default_value="0"),
        DeclareLaunchArgument("tunnel_enabled", default_value="false"),
        DeclareLaunchArgument("tunnel_ir_length_exit_enabled", default_value="false",
                              description="Experimental IR observed-length exit trigger."),
        DeclareLaunchArgument("tunnel_ir_length_exit_ratio", default_value="0.15"),
        DeclareLaunchArgument("tunnel_ir_length_exit_confirm_frames", default_value="3"),
        DeclareLaunchArgument("tunnel_ir_length_exit_minimum_reference_m", default_value="0.5"),
        DeclareLaunchArgument("tunnel_ir_imu_enabled", default_value="true",
                              description="Apply capture-time IMU tilt to tunnel IR BEV; false uses fixed measured ground pose."),
        DeclareLaunchArgument("tunnel_validation_mode", default_value="true",
                              description="Use standalone IR publication timing and bypass integration-only input gates."),
        DeclareLaunchArgument("depthai_library_dir", default_value="",
                              description="Optional DepthAI runtime library directory; default uses build SDK."),
        DeclareLaunchArgument("bev_params_file", default_value=bev_config),
        DeclareLaunchArgument("camera_params_file", default_value=camera_config),
        DeclareLaunchArgument("line_detactor_params_file", default_value=line_detactor_config),
        *[DeclareLaunchArgument(name, default_value=_PARAMETER_FILE_DEFAULT,
                                description="Override lane postprocessing YAML parameter")
          for name in _DETECTOR_POSTPROCESS_OVERRIDES],
        OpaqueFunction(function=_setup, kwargs={
            "bev_config": bev_config,
            "line_detactor_config": line_detactor_config,
            "bev_launch": bev_launch,
        }),
        vesc_bridge_node,
        dynamics_node,
    ])
