"""auto_control 안전 전환 노드 (cone_drive_test에서 옮김): /vesc/*의 유일한 발행자.

auto_handover_node(manual_control 복사본)를 그대로 쓰고 다음만 더한다.
- MANUAL로 시작 (initial_mode): 실행 직후 조이스틱 수동 주행 가능, 자동 명령은 무시
- /auto_drive/handover/request (String): "manual" = 주행 프로세스가 정지 확인 후 수동 전환 요청,
  "auto" = 브라우저 START 승인 후 AUTO 재허용 요청 (정지·조작 해제일 때만 수락)
- AUTO 중 자동 명령이 auto_command_timeout_sec 넘게 끊기면 제동 (주행 프로세스 멈춤 대비)
- X 버튼은 언제든 자동 명령 차단 → 제동 → 정지 확인 → MANUAL (원본과 같음)
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from auto_control_safety.auto_handover_node import AutoHandoverNode
from auto_control_safety.handover_state import HandoverState


class SafetyNode(AutoHandoverNode):
    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("initial_mode", "MANUAL")
        self.declare_parameter("handover_request_topic", "/auto_drive/handover/request")
        self.declare_parameter("auto_command_timeout_sec", 0.3)
        self.auto_command_timeout_sec = float(self.get_parameter("auto_command_timeout_sec").value)
        if not math.isfinite(self.auto_command_timeout_sec) or self.auto_command_timeout_sec <= 0.0:
            raise ValueError("auto_command_timeout_sec must be positive")
        self.handover = HandoverState(
            float(self.get_parameter("handover_stopped_erpm").value),
            float(self.get_parameter("handover_stop_hold_sec").value),
            float(self.get_parameter("handover_telemetry_timeout_sec").value),
            initial_mode=str(self.get_parameter("initial_mode").value).upper(),
        )
        self._stale_logged = False
        self.request_sub = self.create_subscription(
            String, str(self.get_parameter("handover_request_topic").value), self._on_request,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._publish_mode()
        self.get_logger().info(
            f"auto_control safety: start in {self.handover.mode}. X blocks automatic commands at any time.")

    def handover_help(self):
        return "auto_control safety handover (X = stop and manual)"

    def _joystick_present(self, now_sec: float) -> bool:
        last = getattr(self, "_joy_any_sec", None)
        return last is not None and 0.0 <= now_sec - last < self.input_timeout_sec

    def _neutral_now(self, now_sec: float) -> bool:
        # 조이스틱이 없으면(입력이 input_timeout 넘게 안 옴: 미연결·꺼짐) 조작할 사람이 없으므로 중립으로 본다
        # → 조이스틱을 먼저 한 번 만지지 않아도 [주행 시작]으로 AUTO가 된다. 이때 X 비상 정지는 쓸 수 없다.
        # 조이스틱 입력이 들어오고 있으면 지금처럼 페달·조향이 풀려 있어야만 AUTO를 허용한다 (RT를 밟은 채 넘어가지 않게)
        if not self._joystick_present(now_sec):
            return True
        return (self._neutral and self._joy_received_sec is not None
                and 0.0 <= now_sec - self._joy_received_sec < self.input_timeout_sec)

    def _on_request(self, msg: String) -> None:
        now_sec = time.monotonic()
        request = msg.data.strip().lower()
        if request == "manual":
            if self.handover.request_manual():
                self.duty_profile.reset_speed()
                self.brake_profile.current_amps = self._auto_brake_current
                self._publish_stopping(1.0 / self.control_rate_hz)
                self._publish_mode()
                self.get_logger().warn("Drive process requested MANUAL: braking, then manual after stop.")
        elif request == "auto":
            if self.handover.request_auto(now_sec, self._neutral_now(now_sec)):
                self.duty_profile.reset_speed()
                self.duty_profile.clear_gear_request()
                self.brake_profile.reset()
                self._stale_logged = False
                self._publish_mode()
                self.get_logger().warn("AUTO accepted: automatic commands now drive the vehicle. X stops.")
            else:
                self._publish_mode()   # 거절: 현재 모드를 다시 알려 주행 프로세스가 ARMING을 끝내게
                self.get_logger().warn("AUTO refused: vehicle must be stopped with fresh ERPM and controls released.")
        else:
            self.get_logger().warn(f"Unknown handover request: {msg.data!r}")

    def _on_joy(self, msg) -> None:
        self._joy_any_sec = time.monotonic()
        # 원본 auto_handover는 MANUAL이 마지막 상태라 MANUAL에서는 중립·X를 갱신하지 않는다.
        # 여기서는 MANUAL → AUTO 재허용(request_auto)에 필요하므로 MANUAL에서도 갱신한 뒤 수동 주행 처리로 넘긴다.
        if self.handover.mode == "MANUAL":
            self._update_neutral(msg)
        super()._on_joy(msg)

    def _update_neutral(self, msg) -> None:
        if len(msg.buttons) > self.handover_button:
            self.handover.on_x(bool(msg.buttons[self.handover_button]))   # MANUAL에서는 상태를 바꾸지 않고 X 눌림만 기록
        axes = (self.joy_accelerator_axis, self.joy_brake_axis, self.joy_steering_axis)
        valid = (min(axes) >= 0 and len(msg.axes) > max(axes)
                 and self.joy_gear_button >= 0
                 and len(msg.buttons) > max(self.handover_button, self.joy_gear_button)
                 and all(math.isfinite(msg.axes[i]) for i in axes))
        self._neutral = bool(valid and (
            self._normalize_trigger(msg.axes[self.joy_accelerator_axis]) <= self.pedal_deadzone
            and self._normalize_trigger(msg.axes[self.joy_brake_axis]) <= self.pedal_deadzone
            and abs(msg.axes[self.joy_steering_axis]) <= self.steering_deadzone
            and not msg.buttons[self.joy_gear_button]
            and not msg.buttons[self.handover_button]
        ))
        self._joy_received_sec = time.monotonic() if valid else None

    def _on_auto_command(self, channel: str, msg: Float32) -> None:
        if self.handover.mode == "AUTO" and math.isfinite(msg.data):
            self.handover.on_auto_command(time.monotonic())
        super()._on_auto_command(channel, msg)

    def _on_control_timer(self) -> None:
        now_sec = time.monotonic()
        if self.handover.mode == "AUTO" and self.handover.auto_command_stale(now_sec, self.auto_command_timeout_sec):
            dt = min(max(now_sec - self._last_handover_tick, 0.0), 2.0 / self.control_rate_hz)
            self._last_handover_tick = now_sec
            if not self._stale_logged:
                self.get_logger().error("Automatic commands stopped arriving: braking.")
                self._stale_logged = True
            self._publish_stopping(dt)
            return
        super()._on_control_timer()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.stop_actuators()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
