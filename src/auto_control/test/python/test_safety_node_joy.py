"""안전 노드: MANUAL 상태에서도 조이스틱 중립(페달·조향 해제)을 갱신해야 AUTO 재허용이 가능하다.

rclpy 없이 돌도록 ROS 모듈을 가짜로 끼우고, 노드 객체는 __init__ 없이 필요한 속성만 채운다.
"""
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def _install_fake_ros():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    class _Node:
        pass

    class _Msg:
        def __init__(self, data=None):
            self.data = data

    mod("rclpy", init=lambda *a, **k: None, spin=lambda *a, **k: None, ok=lambda: False, shutdown=lambda: None)
    mod("rclpy.node", Node=_Node)
    mod("rclpy.qos", QoSProfile=object, ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=0, RELIABLE=1),
        DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=0, VOLATILE=1))
    mod("rclpy.time", Time=object)
    mod("rclpy.duration", Duration=object)
    mod("sensor_msgs")
    mod("sensor_msgs.msg", Joy=object)
    mod("std_msgs")
    mod("std_msgs.msg", Bool=_Msg, Float32=_Msg, Int32=_Msg, String=_Msg)


_install_fake_ros()
from auto_control_safety import actuator_commander_node as acn  # noqa: E402
from auto_control_safety.handover_state import HandoverState  # noqa: E402
from auto_control_safety.safety_node import SafetyNode  # noqa: E402


def _node(mode):
    n = SafetyNode.__new__(SafetyNode)
    n.handover = HandoverState(initial_mode=mode)
    n.handover_button = 2
    n.joy_accelerator_axis, n.joy_brake_axis, n.joy_steering_axis, n.joy_gear_button = 5, 4, 0, 10
    n.joy_trigger_released, n.joy_trigger_pressed = 0.0, 1.0
    n.pedal_deadzone, n.steering_deadzone = 0.01, 0.04
    n.input_timeout_sec = 0.3
    n._neutral, n._joy_received_sec = False, None
    return n


def _joy(accel=0.0, brake=0.0, steer=0.0, x=0):
    axes = [0.0] * 6
    axes[5], axes[4], axes[0] = accel, brake, steer
    buttons = [0] * 12
    buttons[2] = x
    return types.SimpleNamespace(axes=axes, buttons=buttons)


def test_manual_mode_updates_neutral(monkeypatch):
    monkeypatch.setattr(acn.ActuatorCommanderNode, "_on_joy", lambda self, msg: None)   # 수동 주행 처리는 이 테스트 밖
    n = _node("MANUAL")
    n._on_joy(_joy())
    assert n._neutral is True
    assert n._joy_received_sec is not None
    n._on_joy(_joy(accel=0.5))
    assert n._neutral is False


def test_manual_mode_tracks_x_button(monkeypatch):
    monkeypatch.setattr(acn.ActuatorCommanderNode, "_on_joy", lambda self, msg: None)
    n = _node("MANUAL")
    n._on_joy(_joy(x=1))
    assert n._neutral is False                  # X를 누르고 있으면 중립 아님
    n._on_joy(_joy(x=0))
    assert n._neutral is True


def test_no_joystick_counts_as_neutral():
    """조이스틱이 연결 안 돼 입력이 없으면 조작 해제로 보고 AUTO를 허용한다."""
    n = _node("MANUAL")
    assert n._neutral_now(100.0) is True


def test_joystick_present_requires_released_controls(monkeypatch):
    monkeypatch.setattr(acn.ActuatorCommanderNode, "_on_joy", lambda self, msg: None)
    import time
    n = _node("MANUAL")
    n._on_joy(_joy(accel=0.5))                  # 조이스틱이 있고 RT를 밟고 있음
    assert n._neutral_now(time.monotonic()) is False
    n._on_joy(_joy())
    assert n._neutral_now(time.monotonic()) is True
    assert n._neutral_now(time.monotonic() + 5.0) is True   # 조이스틱이 꺼져 입력이 끊기면 다시 '없음' = 중립


def test_stopping_to_manual_without_joystick():
    """정지 요청(STOPPING) 뒤 정지가 확인되면 조이스틱이 없어도 MANUAL로 넘어가 다시 출발할 수 있다."""
    n = _node("AUTO")
    assert n.handover.request_manual()
    t = 1000.0
    for k in range(20):
        n.handover.on_erpm(0, t + 0.05 * k)
    assert n.handover.update(t + 0.95, n._neutral_now(t + 0.95)) is True
    assert n.handover.mode == "MANUAL"
