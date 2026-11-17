"""cone_drive_test 실행: 주행 프로세스(C++) + 안전 전환 프로세스(Python) + VESC 브리지 + 조이스틱.

    ros2 launch cone_drive_test cone_drive_test.launch.py                 # 추적 주행 모드 (기본, 브라우저 [주행 시작]으로 출발)
    ros2 launch cone_drive_test cone_drive_test.launch.py mode:=detect    # 검출 테스트 모드 (자동 명령 없음)
    브라우저: http://<jetson-ip>:8765

다른 카메라 노드(camera_driver, multiclass_yolo 등)와 기존 수동/자동 주행 launch는 끄고 실행한다 (OAK·VESC를 이 패키지가 쓴다).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns = LaunchConfiguration("vehicle_namespace")
    share = FindPackageShare("cone_drive_test")
    args = [
        DeclareLaunchArgument("mode", default_value="drive", description="drive (기본, [주행 시작]으로 출발) | detect"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([share, "config", "cone_drive_test.yaml"])),
        DeclareLaunchArgument("vehicle_namespace", default_value="autopilot03"),
        DeclareLaunchArgument("vesc_enabled", default_value="true"),
        DeclareLaunchArgument("joy_enabled", default_value="true"),
        DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
        DeclareLaunchArgument("controller_name_contains", default_value="8BitDo"),
        DeclareLaunchArgument("web_port", default_value="8765"),
    ]
    vesc_topics = {
        "duty_topic": "vesc/duty",
        "brake_current_topic": "vesc/brake_current",
        "erpm_topic": "vesc/erpm",
        "measured_erpm_topic": "vesc/measured_erpm",
        "servo_position_topic": "vesc/servo_position",
        "connection_status_topic": "vesc/connected",
    }
    drive = Node(
        package="cone_drive_test", executable="cone_drive_test_node", name="cone_drive_test", namespace=ns,
        output="screen",
        parameters=[LaunchConfiguration("params_file"),
                    {"mode": LaunchConfiguration("mode"),
                     "web.port": ParameterValue(LaunchConfiguration("web_port"), value_type=int)}],
    )
    safety = Node(
        package="cone_drive_test", executable="safety_node", name="cone_drive_safety", namespace=ns, output="screen",
        parameters=[PathJoinSubstitution([share, "config", "safety.yaml"]), {
            "initial_mode": "MANUAL",
            "joy_topic": "joy",
            "duty_topic": "vesc/duty",
            "brake_current_topic": "vesc/brake_current",
            "servo_position_topic": "vesc/servo_position",
            "measured_erpm_topic": "vesc/measured_erpm",
            "connection_status_topic": "vesc/connected",
            "current_duty_topic": "manual/current_duty",
            "current_brake_current_topic": "manual/current_brake_current",
            "gear_state_topic": "manual/gear",
            "auto_duty_topic": "cone_drive/auto/duty",
            "auto_brake_current_topic": "cone_drive/auto/brake_current",
            "auto_servo_position_topic": "cone_drive/auto/servo_position",
            "drive_mode_topic": "cone_drive/handover/mode",
            "handover_request_topic": "cone_drive/handover/request",
            "auto_enable_topic": "cone_drive/auto_enabled",
        }],
    )
    vesc = Node(
        package="vesc_bridge", executable="vesc_bridge_node", name="vesc_bridge_node", namespace=ns, output="screen",
        condition=IfCondition(LaunchConfiguration("vesc_enabled")),
        parameters=[PathJoinSubstitution([share, "config", "vesc_config.yaml"]),
                    {"port": LaunchConfiguration("vesc_port"), **vesc_topics}],
    )
    joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("joy_initializer"), "launch", "joy.launch.py"])),
        condition=IfCondition(LaunchConfiguration("joy_enabled")),
        launch_arguments={
            "vehicle_namespace": ns, "device_id": "0",
            "device_name_contains": LaunchConfiguration("controller_name_contains"),
            "deadzone": "0.05", "autorepeat_rate": "50.0", "sticky_buttons": "false",
            "coalesce_interval_ms": "1", "reconnect_interval_sec": "0.2",
        }.items(),
    )
    return LaunchDescription(args + [drive, safety, vesc, joy])
