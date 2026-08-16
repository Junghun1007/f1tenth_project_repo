from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class PathModel:
    """Smoothed y(x) centerline in the vehicle frame."""

    coefficients: np.ndarray
    minimum_x_m: float
    maximum_x_m: float
    point_count: int

    def lateral_position(self, x_m: float | np.ndarray) -> float | np.ndarray:
        return np.polynomial.polynomial.polyval(x_m, self.coefficients)

    def first_derivative(self, x_m: float | np.ndarray) -> float | np.ndarray:
        derivative = np.polynomial.polynomial.polyder(self.coefficients, 1)
        return np.polynomial.polynomial.polyval(x_m, derivative)

    def second_derivative(self, x_m: float | np.ndarray) -> float | np.ndarray:
        derivative = np.polynomial.polynomial.polyder(self.coefficients, 2)
        return np.polynomial.polynomial.polyval(x_m, derivative)

    def curvature(self, x_m: float | np.ndarray) -> float | np.ndarray:
        first = self.first_derivative(x_m)
        second = self.second_derivative(x_m)
        return second / np.power(1.0 + np.square(first), 1.5)


@dataclass(frozen=True)
class StanleyResult:
    steering_angle_rad: float
    cross_track_error_m: float
    heading_error_rad: float


def centerline_points_from_mono8(
    image: np.ndarray,
    *,
    x_max_m: float,
    y_max_m: float,
    lateral_margin_m: float,
    meter_per_pixel: float,
    threshold: int = 1,
) -> tuple[np.ndarray, np.ndarray]:
    """Return one metric center point per occupied image row.

    The BEV image is ordered with +X toward the top and +Y toward the left.
    The published lane image includes ``lateral_margin_m`` on both sides.
    """

    if image.ndim != 2:
        raise ValueError("centerline image must be two-dimensional")
    if meter_per_pixel <= 0.0:
        raise ValueError("meter_per_pixel must be positive")

    rows, columns = np.nonzero(image >= threshold)
    if rows.size == 0:
        return np.empty(0, dtype=float), np.empty(0, dtype=float)

    row_counts = np.bincount(rows, minlength=image.shape[0])
    column_sums = np.bincount(
        rows,
        weights=columns.astype(float),
        minlength=image.shape[0],
    )
    occupied_rows = np.flatnonzero(row_counts)
    center_columns = column_sums[occupied_rows] / row_counts[occupied_rows]

    x_m = x_max_m - (occupied_rows.astype(float) + 0.5) * meter_per_pixel
    y_m = (
        y_max_m
        + lateral_margin_m
        - (center_columns + 0.5) * meter_per_pixel
    )

    # Calculations below expect increasing forward distance.
    order = np.argsort(x_m)
    return x_m[order], y_m[order]


def fit_path_model(
    x_m: Iterable[float],
    y_m: Iterable[float],
    *,
    polynomial_order: int,
    minimum_points: int,
    minimum_span_m: float,
    minimum_x_m: float,
    maximum_x_m: float,
) -> PathModel | None:
    x = np.asarray(tuple(x_m), dtype=float)
    y = np.asarray(tuple(y_m), dtype=float)
    finite = np.isfinite(x) & np.isfinite(y)
    selected = finite & (x >= minimum_x_m) & (x <= maximum_x_m)
    x = x[selected]
    y = y[selected]

    if x.size < minimum_points:
        return None
    span_m = float(np.max(x) - np.min(x))
    if span_m < minimum_span_m:
        return None

    order = min(max(1, int(polynomial_order)), int(x.size) - 1)
    coefficients = np.polynomial.polynomial.polyfit(x, y, order)
    if not np.all(np.isfinite(coefficients)):
        return None
    return PathModel(
        coefficients=coefficients,
        minimum_x_m=float(np.min(x)),
        maximum_x_m=float(np.max(x)),
        point_count=int(x.size),
    )


def stanley_control(
    path: PathModel,
    *,
    speed_mps: float,
    gain: float,
    softening_speed_mps: float,
    heading_lookahead_m: float,
    maximum_steering_angle_rad: float,
) -> StanleyResult:
    """Calculate a front-axle Stanley steering command.

    Positive steering is left because the vehicle frame uses +Y left.
    The published path starts ahead of the axle, so lateral error is evaluated
    at its nearest available X while heading uses a short forward lookahead.
    """

    anchor_x_m = path.minimum_x_m
    cross_track_error_m = float(path.lateral_position(anchor_x_m))
    heading_x_m = min(
        path.maximum_x_m,
        anchor_x_m + max(0.0, heading_lookahead_m),
    )
    heading_error_rad = math.atan(float(path.first_derivative(heading_x_m)))
    correction_rad = math.atan2(
        max(0.0, gain) * cross_track_error_m,
        max(0.0, abs(speed_mps)) + max(0.0, softening_speed_mps),
    )
    steering_angle_rad = clamp(
        heading_error_rad + correction_rad,
        -maximum_steering_angle_rad,
        maximum_steering_angle_rad,
    )
    return StanleyResult(
        steering_angle_rad=steering_angle_rad,
        cross_track_error_m=cross_track_error_m,
        heading_error_rad=heading_error_rad,
    )


def representative_curvature(
    path: PathModel,
    *,
    lookahead_minimum_x_m: float,
    lookahead_maximum_x_m: float,
    percentile: float,
    sample_count: int = 32,
) -> float:
    minimum_x_m = max(path.minimum_x_m, lookahead_minimum_x_m)
    maximum_x_m = min(path.maximum_x_m, lookahead_maximum_x_m)
    if maximum_x_m <= minimum_x_m:
        minimum_x_m = path.minimum_x_m
        maximum_x_m = path.maximum_x_m
    samples = np.linspace(minimum_x_m, maximum_x_m, max(2, sample_count))
    curvature = np.abs(path.curvature(samples))
    finite_curvature = curvature[np.isfinite(curvature)]
    if finite_curvature.size == 0:
        return 0.0
    return float(np.percentile(finite_curvature, clamp(percentile, 0.0, 100.0)))


def curvature_target_speed(
    curvature_per_m: float,
    *,
    maximum_lateral_acceleration_mps2: float,
    minimum_speed_mps: float,
    maximum_speed_mps: float,
) -> float:
    curvature = abs(float(curvature_per_m))
    if curvature <= 1.0e-6:
        return maximum_speed_mps
    safe_speed_mps = math.sqrt(
        max(0.0, maximum_lateral_acceleration_mps2) / curvature
    )
    return clamp(safe_speed_mps, minimum_speed_mps, maximum_speed_mps)


def erpm_to_speed_mps(
    measured_erpm: int,
    *,
    wheel_diameter_m: float,
    motor_pole_pairs: int,
    motor_pinion_teeth: int,
    spur_gear_teeth: int,
    differential_pinion_teeth: int,
    differential_ring_teeth: int,
    direction_sign: float,
    scale_correction: float,
) -> float:
    gear_ratio = (
        float(spur_gear_teeth) / float(motor_pinion_teeth)
        * float(differential_ring_teeth) / float(differential_pinion_teeth)
    )
    return (
        float(measured_erpm)
        * (1.0 if direction_sign >= 0.0 else -1.0)
        * scale_correction
        * math.pi
        * wheel_diameter_m
        / (60.0 * float(motor_pole_pairs) * gear_ratio)
    )


def steering_angle_to_servo(
    steering_angle_rad: float,
    *,
    maximum_steering_angle_rad: float,
    servo_left: float,
    servo_center: float,
    servo_right: float,
) -> float:
    normalized = clamp(
        steering_angle_rad / maximum_steering_angle_rad,
        -1.0,
        1.0,
    )
    if normalized >= 0.0:
        return lerp(servo_center, servo_left, normalized)
    return lerp(servo_center, servo_right, -normalized)


class SpeedPid:
    def __init__(
        self,
        *,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float,
        minimum_duty: float,
        maximum_duty: float,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.minimum_duty = minimum_duty
        self.maximum_duty = maximum_duty
        self._integral = 0.0
        self._previous_error: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error = None

    def update(
        self,
        *,
        target_speed_mps: float,
        current_speed_mps: float,
        feedforward_duty: float,
        dt_sec: float,
    ) -> float:
        dt_sec = max(1.0e-4, float(dt_sec))
        error = target_speed_mps - current_speed_mps
        derivative = 0.0
        if self._previous_error is not None:
            derivative = (error - self._previous_error) / dt_sec

        candidate_integral = clamp(
            self._integral + error * dt_sec,
            -self.integral_limit,
            self.integral_limit,
        )
        candidate = (
            feedforward_duty
            + self.kp * error
            + self.ki * candidate_integral
            + self.kd * derivative
        )
        output = clamp(candidate, self.minimum_duty, self.maximum_duty)

        # Conditional integration prevents further wind-up while saturated.
        saturated_high = candidate > self.maximum_duty and error > 0.0
        saturated_low = candidate < self.minimum_duty and error < 0.0
        if not saturated_high and not saturated_low:
            self._integral = candidate_integral
        self._previous_error = error
        return output


def speed_feedforward_duty(
    target_speed_mps: float,
    *,
    minimum_speed_mps: float,
    maximum_speed_mps: float,
    minimum_duty: float,
    maximum_duty: float,
) -> float:
    if maximum_speed_mps <= minimum_speed_mps:
        return minimum_duty
    amount = clamp(
        (target_speed_mps - minimum_speed_mps)
        / (maximum_speed_mps - minimum_speed_mps),
        0.0,
        1.0,
    )
    return lerp(minimum_duty, maximum_duty, amount)


def move_toward(value: float, target: float, maximum_step: float) -> float:
    maximum_step = max(0.0, maximum_step)
    if value < target:
        return min(target, value + maximum_step)
    if value > target:
        return max(target, value - maximum_step)
    return target


def lerp(start: float, end: float, amount: float) -> float:
    return start + (end - start) * amount


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, float(value)))
