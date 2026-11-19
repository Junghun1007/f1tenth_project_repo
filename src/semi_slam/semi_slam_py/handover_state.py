# Copied from src/manual_control/manual_control/handover_state.py (origin/1113ML)
"""AUTO -> STOPPING -> MANUAL handover, independent of ROS.

semi_slam 확장: MANUAL로 시작(initial_mode), 주행 프로세스의 수동 전환 요청(request_manual),
START 승인 후 정지·페달 해제 상태에서만 AUTO 재허용(request_auto), 자동 명령 끊김 감시(auto_command_stale).
"""

from __future__ import annotations

import math


class HandoverState:
    def __init__(
        self, stopped_erpm: float = 100.0, stop_hold_sec: float = 0.5,
        telemetry_timeout_sec: float = 0.3, initial_mode: str = "AUTO",
    ) -> None:
        for value in (stopped_erpm, stop_hold_sec, telemetry_timeout_sec):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("Handover thresholds must be finite and positive")
        self.stopped_erpm = stopped_erpm
        self.stop_hold_sec = stop_hold_sec
        self.telemetry_timeout_sec = telemetry_timeout_sec
        if initial_mode not in ("AUTO", "MANUAL"):
            raise ValueError("initial_mode must be AUTO or MANUAL")
        self.mode = initial_mode
        self._x_pressed = False
        self._last_auto_command: float | None = None
        self._last_erpm_time: float | None = None
        self._stopped_since: float | None = None

    def on_x(self, pressed: bool) -> bool:
        rising = pressed and not self._x_pressed
        self._x_pressed = pressed
        if self.mode != "AUTO" or not rising:
            return False
        self.mode = "STOPPING"
        # Measurements received before X cannot establish that braking stopped us.
        self.invalidate_telemetry()
        return True

    def invalidate_telemetry(self) -> None:
        self._last_erpm_time = None
        self._stopped_since = None

    def accepts_auto(self) -> bool:
        return self.mode == "AUTO"

    def request_manual(self) -> bool:
        """주행 프로세스가 정지를 확인한 뒤 보내는 수동 전환 요청 (X와 같은 경로)."""
        if self.mode != "AUTO":
            return False
        self.mode = "STOPPING"
        self.invalidate_telemetry()
        return True

    def request_auto(self, now: float, neutral: bool) -> bool:
        """START 승인 후 AUTO 재허용: MANUAL, 신선한 ERPM으로 정지 확인, 조작 해제일 때만."""
        if self.mode != "MANUAL" or not neutral or self._x_pressed:
            return False
        if self._last_erpm_time is None or not (0.0 <= now - self._last_erpm_time < self.telemetry_timeout_sec):
            return False
        if self._stopped_since is None or now - self._stopped_since < self.stop_hold_sec:
            return False
        self.mode = "AUTO"
        self._last_auto_command = now          # 진입 직후 첫 명령까지 유예
        return True

    def on_auto_command(self, now: float) -> None:
        self._last_auto_command = now

    def auto_command_stale(self, now: float, timeout_sec: float) -> bool:
        if self.mode != "AUTO" or self._last_auto_command is None:
            return False
        return now - self._last_auto_command > timeout_sec

    def on_erpm(self, erpm: float, now: float) -> None:
        if self.mode not in ("STOPPING", "MANUAL"):
            return
        if not math.isfinite(erpm):
            self.invalidate_telemetry()
            return
        if self._last_erpm_time is None or not (
            0.0 <= now - self._last_erpm_time < self.telemetry_timeout_sec
        ):
            self._stopped_since = None
        self._last_erpm_time = now
        if abs(erpm) > self.stopped_erpm:
            self._stopped_since = None
        elif self._stopped_since is None:
            self._stopped_since = now

    def update(self, now: float, neutral: bool) -> bool:
        if self.mode != "STOPPING":
            return False
        if self._last_erpm_time is None or not (
            0.0 <= now - self._last_erpm_time < self.telemetry_timeout_sec
        ):
            self.invalidate_telemetry()
            return False
        if (self._stopped_since is not None
                and now - self._stopped_since >= self.stop_hold_sec
                and neutral and not self._x_pressed):
            self.mode = "MANUAL"
            return True
        return False
