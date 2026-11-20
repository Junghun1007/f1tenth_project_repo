"""안전 전환 상태 (rclpy 없이). 원본 manual_control HandoverState + cone_drive_test 확장."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from auto_control_safety.handover_state import HandoverState  # noqa: E402


def stopped(h, t0, t1, erpm=0.0, step=0.05):
    for k in range(int(round((t1 - t0) / step)) + 1):
        h.on_erpm(erpm, round(t0 + k * step, 9))


def test_x_blocks_auto():
    h = HandoverState()
    assert h.mode == "AUTO" and h.accepts_auto()
    assert h.on_x(True)
    assert h.mode == "STOPPING"
    assert not h.accepts_auto()


def test_initial_manual():
    h = HandoverState(initial_mode="MANUAL")
    assert h.mode == "MANUAL"
    assert not h.accepts_auto()


def test_request_manual_from_auto_goes_through_stopping():
    h = HandoverState()
    assert h.request_manual()
    assert h.mode == "STOPPING"
    stopped(h, 0.0, 0.6)
    assert h.update(0.6, neutral=True)
    assert h.mode == "MANUAL"


def test_request_auto_only_when_stopped_and_neutral():
    h = HandoverState(initial_mode="MANUAL")
    stopped(h, 0.0, 1.0, erpm=500.0)
    assert not h.request_auto(1.0, neutral=True)          # 움직이는 중
    stopped(h, 1.05, 1.3)
    assert not h.request_auto(1.3, neutral=True)          # 정지 0.25 s
    stopped(h, 1.35, 1.6)
    assert not h.request_auto(1.6, neutral=False)         # 페달·조향 안 놓음
    assert h.request_auto(1.6, neutral=True)
    assert h.mode == "AUTO"


def test_request_auto_needs_fresh_erpm():
    h = HandoverState(initial_mode="MANUAL")
    stopped(h, 0.0, 1.0)
    assert not h.request_auto(2.0, neutral=True)          # 마지막 ERPM 1 s 전


def test_auto_command_timeout_brakes():
    h = HandoverState(initial_mode="MANUAL")
    stopped(h, 0.0, 0.6)
    assert h.request_auto(0.6, neutral=True)
    assert not h.auto_command_stale(0.8, 0.3)             # AUTO 진입 직후 유예
    h.on_auto_command(0.8)
    assert not h.auto_command_stale(1.0, 0.3)
    assert h.auto_command_stale(1.11, 0.3)


def test_x_still_works_after_rearm():
    h = HandoverState(initial_mode="MANUAL")
    stopped(h, 0.0, 0.6)
    assert h.request_auto(0.6, neutral=True)
    assert h.on_x(True)
    assert h.mode == "STOPPING"
