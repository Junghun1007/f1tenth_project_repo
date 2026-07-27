from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Gear(Enum):
    FORWARD = 1
    REVERSE = -1


@dataclass(frozen=True)
class ErpmProfileConfig:
    forward_max_erpm: int
    reverse_max_erpm: int
    start_erpm: int
    acceleration_erpm_per_sec: float
    coast_deceleration_erpm_per_sec: float
    brake_erpm_per_sec: float
    pedal_deadzone: float

    def __post_init__(self) -> None:
        if self.forward_max_erpm <= 0:
            raise ValueError("forward_max_erpm must be positive")
        if self.reverse_max_erpm <= 0:
            raise ValueError("reverse_max_erpm must be positive")
        if self.start_erpm <= 0:
            raise ValueError("start_erpm must be positive")
        if self.start_erpm > min(self.forward_max_erpm, self.reverse_max_erpm):
            raise ValueError("start_erpm must not exceed either gear limit")
        if self.acceleration_erpm_per_sec <= 0.0:
            raise ValueError("acceleration_erpm_per_sec must be positive")
        if self.coast_deceleration_erpm_per_sec <= 0.0:
            raise ValueError(
                "coast_deceleration_erpm_per_sec must be positive"
            )
        if self.brake_erpm_per_sec <= 0.0:
            raise ValueError("brake_erpm_per_sec must be positive")
        if not 0.0 <= self.pedal_deadzone <= 1.0:
            raise ValueError("pedal_deadzone must be between 0 and 1")


class ErpmCommandProfile:
    def __init__(self, config: ErpmProfileConfig) -> None:
        self.config = config
        self.gear = Gear.FORWARD
        self.current_erpm = 0.0

    @property
    def command_erpm(self) -> int:
        return int(round(self.current_erpm))

    def update(self, accelerator: float, brake: float, dt_sec: float) -> int:
        accelerator = self._pedal_value(accelerator)
        brake = self._pedal_value(brake)
        dt_sec = max(0.0, float(dt_sec))

        if brake > 0.0:
            self.current_erpm = self._move_toward(
                self.current_erpm,
                0.0,
                brake * self.config.brake_erpm_per_sec * dt_sec,
            )
        elif accelerator > 0.0:
            target_erpm = self.gear.value * self._gear_limit()
            if abs(self.current_erpm) < self.config.start_erpm:
                self.current_erpm = float(
                    self.gear.value * self.config.start_erpm
                )
            else:
                self.current_erpm = self._move_toward(
                    self.current_erpm,
                    float(target_erpm),
                    accelerator * self.config.acceleration_erpm_per_sec * dt_sec,
                )
        else:
            self.current_erpm = self._move_toward(
                self.current_erpm,
                0.0,
                self.config.coast_deceleration_erpm_per_sec * dt_sec,
            )

        return self.command_erpm

    def toggle_gear(self) -> bool:
        if self.current_erpm != 0.0:
            return False

        self.gear = (
            Gear.REVERSE if self.gear is Gear.FORWARD else Gear.FORWARD
        )
        return True

    def reset_speed(self) -> None:
        self.current_erpm = 0.0

    def _gear_limit(self) -> int:
        if self.gear is Gear.FORWARD:
            return self.config.forward_max_erpm
        return self.config.reverse_max_erpm

    def _pedal_value(self, value: float) -> float:
        value = self._clamp(float(value), 0.0, 1.0)
        if value < self.config.pedal_deadzone:
            return 0.0
        return value

    @staticmethod
    def _move_toward(value: float, target: float, amount: float) -> float:
        if value < target:
            return min(target, value + amount)
        if value > target:
            return max(target, value - amount)
        return target

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))
