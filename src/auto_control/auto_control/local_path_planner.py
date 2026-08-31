from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from auto_control.control_core import StanleyResult, clamp


@dataclass(frozen=True)
class OrderedPathModel:
    """An arc-length-ordered local path in the front-axle vehicle frame."""

    points_m: np.ndarray
    arc_length_m: np.ndarray
    geometry_window_m: float

    @property
    def point_count(self) -> int:
        return int(self.points_m.shape[0])

    @property
    def total_length_m(self) -> float:
        return float(self.arc_length_m[-1])

    def point_at_s(self, s_m: float | np.ndarray) -> np.ndarray:
        query = np.asarray(s_m, dtype=float)
        flat = np.atleast_1d(query).ravel()
        x = np.interp(flat, self.arc_length_m, self.points_m[:, 0])
        y = np.interp(flat, self.arc_length_m, self.points_m[:, 1])
        points = np.column_stack((x, y))
        if query.ndim == 0:
            return points[0]
        return points.reshape(query.shape + (2,))

    def tangent_at_s(self, s_m: float | np.ndarray) -> np.ndarray:
        query = np.asarray(s_m, dtype=float)
        flat = np.atleast_1d(query).ravel()
        half_window = 0.5 * max(self.geometry_window_m, 1.0e-3)
        lower = np.maximum(0.0, flat - half_window)
        upper = np.minimum(self.total_length_m, flat + half_window)
        short = upper - lower < half_window
        lower = np.where(short, np.maximum(0.0, upper - 2.0 * half_window), lower)
        upper = np.where(
            short,
            np.minimum(self.total_length_m, lower + 2.0 * half_window),
            upper,
        )
        delta = self.point_at_s(upper) - self.point_at_s(lower)
        length = np.linalg.norm(delta, axis=1)
        valid = length > 1.0e-9
        tangent = np.zeros_like(delta)
        tangent[valid] = delta[valid] / length[valid, None]
        tangent[~valid, 0] = 1.0
        if query.ndim == 0:
            return tangent[0]
        return tangent.reshape(query.shape + (2,))

    def heading_at_s(self, s_m: float | np.ndarray) -> float | np.ndarray:
        tangent = self.tangent_at_s(s_m)
        heading = np.arctan2(tangent[..., 1], tangent[..., 0])
        return float(heading) if np.ndim(s_m) == 0 else heading

    def curvature_at_s(self, s_m: float | np.ndarray) -> float | np.ndarray:
        query = np.asarray(s_m, dtype=float)
        flat = np.atleast_1d(query).ravel()
        window = min(
            max(self.geometry_window_m, 1.0e-3),
            0.5 * self.total_length_m,
        )
        result = np.zeros_like(flat)
        if window <= 1.0e-6:
            return float(result[0]) if query.ndim == 0 else result.reshape(query.shape)
        center_s = np.clip(flat, window, self.total_length_m - window)
        first = self.point_at_s(center_s - window)
        middle = self.point_at_s(center_s)
        last = self.point_at_s(center_s + window)
        first_delta = middle - first
        full_delta = last - first
        first_length = np.linalg.norm(first_delta, axis=1)
        second_length = np.linalg.norm(last - middle, axis=1)
        chord_length = np.linalg.norm(full_delta, axis=1)
        denominator = first_length * second_length * chord_length
        cross = first_delta[:, 0] * full_delta[:, 1] - first_delta[:, 1] * full_delta[:, 0]
        valid = denominator > 1.0e-9
        result[valid] = 2.0 * cross[valid] / denominator[valid]
        return float(result[0]) if query.ndim == 0 else result.reshape(query.shape)


@dataclass(frozen=True)
class LocalPathPlannerConfig:
    enabled: bool = True
    expected_lane_width_m: float = 0.65
    vehicle_width_m: float = 0.30
    safety_margin_m: float = 0.08
    minimum_half_width_m: float = 0.24
    maximum_half_width_m: float = 0.48
    width_smoothing_window_m: float = 0.20
    maximum_center_correction_m: float = 0.08
    resample_interval_m: float = 0.02
    corner_curvature_threshold_per_m: float = 0.45
    corner_curvature_smoothing_window_m: float = 0.16
    corner_minimum_heading_change_deg: float = 20.0
    corner_approach_length_m: float = 0.35
    corner_setup_transition_length_m: float = 0.0
    corner_pre_turn_outside_hold_m: float = 0.25
    corner_exit_length_m: float = 0.40
    post_corner_offset_hold_distance_m: float = 0.50
    same_direction_corner_link_maximum_gap_m: float = 1.50
    outside_offset_fraction: float = 0.25
    apex_offset_fraction: float = 0.35
    maximum_offset_m: float = 0.08
    maximum_offset_slope: float = 0.70
    racing_line_weight: float = 0.20
    maximum_path_curvature_per_m: float = 1.8


@dataclass(frozen=True)
class PlannedPath:
    path: OrderedPathModel
    base_path: OrderedPathModel
    lateral_offset_m: np.ndarray
    center_correction_m: np.ndarray
    left_half_width_m: np.ndarray
    right_half_width_m: np.ndarray
    detected_corner_count: int
    detected_corners: tuple["DetectedCorner", ...]


@dataclass(frozen=True)
class DetectedCorner:
    start_s_m: float
    apex_s_m: float
    end_s_m: float
    turn_direction: int


class CornerOffsetMemory:
    """Retain a passed corner's outside side across local BEV replans."""

    def __init__(
        self,
        *,
        activation_distance_m: float,
        same_direction_link_maximum_gap_m: float,
    ) -> None:
        self.activation_distance_m = max(0.0, activation_distance_m)
        self.same_direction_link_maximum_gap_m = max(
            0.0, same_direction_link_maximum_gap_m
        )
        self.reset()

    def reset(self) -> None:
        self._active_turn_direction = 0
        self._retained_turn_direction = 0
        self._retained_started_distance_m = 0.0

    def retained_state(
        self,
        cumulative_travel_distance_m: float,
        *,
        hold_distance_m: float,
        return_length_m: float,
    ) -> tuple[int, float]:
        if self._retained_turn_direction == 0:
            return 0, 0.0
        traveled = max(
            0.0,
            cumulative_travel_distance_m - self._retained_started_distance_m,
        )
        maximum_retention = max(0.0, hold_distance_m) + max(
            0.0, return_length_m
        )
        if traveled >= maximum_retention:
            self._retained_turn_direction = 0
            return 0, 0.0
        return self._retained_turn_direction, traveled

    def observe(
        self,
        corners: tuple[DetectedCorner, ...],
        cumulative_travel_distance_m: float,
    ) -> bool:
        before = (
            self._active_turn_direction,
            self._retained_turn_direction,
            self._retained_started_distance_m,
        )
        first = corners[0] if corners else None

        # A retained outside side must not fight an approaching opposite turn.
        opposite_approaching = (
            first is not None
            and (
                first.turn_direction != self._retained_turn_direction
                if self._retained_turn_direction != 0
                else first.turn_direction != self._active_turn_direction
            )
            and first.start_s_m
            <= self.same_direction_link_maximum_gap_m
        )
        if opposite_approaching and self._retained_turn_direction != 0:
            self._retained_turn_direction = 0

        near = (
            first
            if first is not None
            and first.start_s_m <= self.activation_distance_m
            else None
        )
        if near is not None:
            self._active_turn_direction = near.turn_direction
        elif self._active_turn_direction != 0:
            if opposite_approaching:
                self._retained_turn_direction = 0
            else:
                self._retained_turn_direction = self._active_turn_direction
                self._retained_started_distance_m = max(
                    0.0, cumulative_travel_distance_m
                )
            self._active_turn_direction = 0

        after = (
            self._active_turn_direction,
            self._retained_turn_direction,
            self._retained_started_distance_m,
        )
        return after != before


@dataclass(frozen=True)
class SpeedProfileConfig:
    enabled: bool = True
    minimum_speed_mps: float = 0.8
    maximum_speed_mps: float = 2.4
    maximum_lateral_acceleration_mps2: float = 0.9
    maximum_longitudinal_acceleration_mps2: float = 1.2
    maximum_longitudinal_deceleration_mps2: float = 1.8
    minimum_combined_braking_ratio: float = 0.20
    lookahead_minimum_s_m: float = 0.05
    lookahead_maximum_s_m: float = 2.8
    sample_interval_m: float = 0.04
    acceleration_control_distance_m: float = 0.20
    curvature_percentile: float = 80.0


@dataclass(frozen=True)
class SpeedPlan:
    target_speed_mps: float
    representative_curvature_per_m: float
    sample_s_m: np.ndarray
    speed_limit_mps: np.ndarray


def _arc_lengths(points: np.ndarray) -> np.ndarray:
    if points.shape[0] == 0:
        return np.empty(0, dtype=float)
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate((np.zeros(1), np.cumsum(segment)))


def _model_from_points(points: np.ndarray, geometry_window_m: float) -> OrderedPathModel:
    arc_length = _arc_lengths(points)
    return OrderedPathModel(
        points_m=np.asarray(points, dtype=float),
        arc_length_m=arc_length,
        geometry_window_m=max(geometry_window_m, 1.0e-3),
    )


def _resample(points: np.ndarray, interval_m: float) -> np.ndarray:
    arc_length = _arc_lengths(points)
    total = float(arc_length[-1])
    if total <= 1.0e-9:
        return points.copy()
    count = max(2, int(math.ceil(total / max(interval_m, 1.0e-3))) + 1)
    target = np.linspace(0.0, total, count)
    return np.column_stack(
        (
            np.interp(target, arc_length, points[:, 0]),
            np.interp(target, arc_length, points[:, 1]),
        )
    )


def _replace_isolated_outliers(points: np.ndarray, threshold_m: float) -> np.ndarray:
    if threshold_m <= 0.0 or points.shape[0] < 3:
        return points.copy()
    filtered = points.copy()
    for index in range(1, points.shape[0] - 1):
        segment = points[index + 1] - points[index - 1]
        length_squared = float(np.dot(segment, segment))
        if length_squared <= 1.0e-12:
            continue
        blend = float(
            np.dot(points[index] - points[index - 1], segment) / length_squared
        )
        projection = points[index - 1] + np.clip(blend, 0.0, 1.0) * segment
        if float(np.linalg.norm(points[index] - projection)) > threshold_m:
            filtered[index] = projection
    return filtered


def _local_linear_smooth(points: np.ndarray, window_m: float) -> np.ndarray:
    if window_m <= 0.0 or points.shape[0] < 3:
        return points.copy()
    arc_length = _arc_lengths(points)
    half_window = 0.5 * window_m
    smoothed = points.copy()
    for index, current_s in enumerate(arc_length):
        selected = np.abs(arc_length - current_s) <= half_window
        if int(np.count_nonzero(selected)) < 3:
            continue
        local_s = arc_length[selected] - current_s
        design = np.column_stack((local_s, np.ones_like(local_s)))
        for axis in (0, 1):
            coefficients, *_ = np.linalg.lstsq(
                design, points[selected, axis], rcond=None
            )
            smoothed[index, axis] = coefficients[1]
    return smoothed


def build_ordered_path_model(
    points_m: Iterable[Iterable[float]],
    *,
    minimum_points: int,
    minimum_span_m: float,
    minimum_x_m: float,
    maximum_x_m: float,
    local_smoothing_window_m: float,
    outlier_threshold_m: float,
    geometry_window_m: float,
    resample_interval_m: float,
) -> OrderedPathModel | None:
    points = np.asarray(tuple(points_m), dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        return None
    finite = np.all(np.isfinite(points), axis=1)
    selected = finite & (points[:, 0] >= minimum_x_m) & (points[:, 0] <= maximum_x_m)
    points = points[selected]
    if points.shape[0] < minimum_points:
        return None
    if np.linalg.norm(points[0]) > np.linalg.norm(points[-1]):
        points = points[::-1].copy()
    keep = np.ones(points.shape[0], dtype=bool)
    keep[1:] = np.linalg.norm(np.diff(points, axis=0), axis=1) > 1.0e-4
    points = points[keep]
    if points.shape[0] < minimum_points:
        return None
    points = _replace_isolated_outliers(points, outlier_threshold_m)
    points = _resample(points, resample_interval_m)
    points = _local_linear_smooth(points, local_smoothing_window_m)
    model = _model_from_points(points, geometry_window_m)
    if model.point_count < minimum_points or model.total_length_m < minimum_span_m:
        return None
    return model


def closest_path_geometry_ordered(path: OrderedPathModel) -> tuple[float, float]:
    start = path.points_m[:-1]
    delta = path.points_m[1:] - start
    length_squared = np.sum(np.square(delta), axis=1)
    projection = np.divide(
        -np.sum(start * delta, axis=1),
        length_squared,
        out=np.zeros_like(length_squared),
        where=length_squared > 1.0e-12,
    )
    projection = np.clip(projection, 0.0, 1.0)
    closest = start + projection[:, None] * delta
    segment = int(np.argmin(np.sum(np.square(closest), axis=1)))
    segment_length = math.sqrt(max(float(length_squared[segment]), 1.0e-12))
    tangent = delta[segment] / segment_length
    cross_track_error = (
        -float(tangent[1]) * float(closest[segment, 0])
        + float(tangent[0]) * float(closest[segment, 1])
    )
    closest_s = float(
        path.arc_length_m[segment] + projection[segment] * segment_length
    )
    return cross_track_error, closest_s


def stanley_control_ordered(
    path: OrderedPathModel,
    *,
    speed_mps: float,
    gain: float,
    softening_speed_mps: float,
    heading_lookahead_m: float,
    maximum_steering_angle_rad: float,
    corner_heading_threshold_rad: float,
    corner_opposing_correction_ratio: float,
) -> StanleyResult:
    cross_track_error_m, closest_s_m = closest_path_geometry_ordered(path)
    heading_error_rad = float(
        path.heading_at_s(
            min(path.total_length_m, closest_s_m + max(0.0, heading_lookahead_m))
        )
    )
    correction_rad = math.atan2(
        max(0.0, gain) * cross_track_error_m,
        max(0.0, abs(speed_mps)) + max(0.0, softening_speed_mps),
    )
    direction_guard_used = False
    if (
        abs(heading_error_rad) >= max(0.0, corner_heading_threshold_rad)
        and heading_error_rad * correction_rad < 0.0
    ):
        maximum_opposing = abs(heading_error_rad) * clamp(
            corner_opposing_correction_ratio, 0.0, 0.99
        )
        limited = clamp(correction_rad, -maximum_opposing, maximum_opposing)
        direction_guard_used = limited != correction_rad
        correction_rad = limited
    steering = clamp(
        heading_error_rad + correction_rad,
        -maximum_steering_angle_rad,
        maximum_steering_angle_rad,
    )
    return StanleyResult(
        steering_angle_rad=steering,
        cross_track_error_m=cross_track_error_m,
        heading_error_rad=heading_error_rad,
        direction_guard_used=direction_guard_used,
    )


def _distance_to_polyline(point: np.ndarray, polyline: np.ndarray) -> float:
    if polyline.ndim != 2 or polyline.shape[0] < 2:
        return math.nan
    start = polyline[:-1]
    delta = polyline[1:] - start
    length_squared = np.sum(np.square(delta), axis=1)
    blend = np.divide(
        np.sum((point - start) * delta, axis=1),
        length_squared,
        out=np.zeros_like(length_squared),
        where=length_squared > 1.0e-12,
    )
    closest = start + np.clip(blend, 0.0, 1.0)[:, None] * delta
    return float(np.min(np.linalg.norm(closest - point, axis=1)))


def _boundary_widths(
    points: np.ndarray,
    boundary: np.ndarray | None,
    *,
    default_half_width_m: float,
    maximum_half_width_m: float,
    arc_length_m: np.ndarray,
    smoothing_window_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    widths = np.full(points.shape[0], default_half_width_m)
    measured_valid = np.zeros(points.shape[0], dtype=bool)
    if boundary is not None and len(boundary) >= 2:
        measured = np.asarray(
            [_distance_to_polyline(point, boundary) for point in points]
        )
        measured_valid = (
            np.isfinite(measured)
            & (measured >= 0.0)
            & (measured <= maximum_half_width_m)
        )
        widths[measured_valid] = measured[measured_valid]
    widths = _smooth_scalar_profile(widths, arc_length_m, smoothing_window_m)
    widths = np.clip(widths, 0.0, maximum_half_width_m)
    return widths, measured_valid


def _smooth_scalar_profile(
    values: np.ndarray, arc_length: np.ndarray, window_m: float
) -> np.ndarray:
    if window_m <= 0.0 or values.size < 3:
        return values.copy()
    result = values.copy()
    half = 0.5 * window_m
    for index, current_s in enumerate(arc_length):
        selected = np.abs(arc_length - current_s) <= half
        result[index] = float(np.median(values[selected]))
    return result


def _smoothstep(amount: np.ndarray) -> np.ndarray:
    value = np.clip(amount, 0.0, 1.0)
    return value * value * value * (value * (value * 6.0 - 15.0) + 10.0)


def _profile_from_anchors(
    query: np.ndarray, anchors: list[tuple[float, float]]
) -> np.ndarray:
    merged: list[tuple[float, float]] = []
    for position, value in sorted(anchors):
        if merged and abs(position - merged[-1][0]) <= 1.0e-6:
            merged[-1] = (position, value)
        else:
            merged.append((position, value))
    result = np.zeros_like(query)
    if len(merged) == 1:
        result[np.argmin(np.abs(query - merged[0][0]))] = merged[0][1]
        return result
    for index in range(1, len(merged)):
        first_s, first_value = merged[index - 1]
        last_s, last_value = merged[index]
        selected = (query >= first_s) & (query <= last_s)
        distance = max(last_s - first_s, 1.0e-6)
        amount = _smoothstep((query[selected] - first_s) / distance)
        result[selected] = first_value + amount * (last_value - first_value)
    return result


def _limit_offset_slope(
    offset: np.ndarray, arc_length: np.ndarray, maximum_slope: float
) -> np.ndarray:
    if maximum_slope <= 0.0:
        return offset.copy()
    limited = offset.copy()
    for index in range(1, limited.size):
        maximum_change = maximum_slope * (arc_length[index] - arc_length[index - 1])
        limited[index] = np.clip(
            limited[index],
            limited[index - 1] - maximum_change,
            limited[index - 1] + maximum_change,
        )
    for index in range(limited.size - 2, -1, -1):
        maximum_change = maximum_slope * (arc_length[index + 1] - arc_length[index])
        limited[index] = np.clip(
            limited[index],
            limited[index + 1] - maximum_change,
            limited[index + 1] + maximum_change,
        )
    return limited


def plan_local_racing_path(
    base_path: OrderedPathModel,
    left_boundary_m: np.ndarray | None,
    right_boundary_m: np.ndarray | None,
    config: LocalPathPlannerConfig,
    *,
    retained_outside_turn_direction: int = 0,
    retained_outside_distance_traveled_m: float = 0.0,
) -> PlannedPath:
    raw_points = base_path.points_m
    s = base_path.arc_length_m
    default_half_width = float(
        np.clip(
            0.5 * config.expected_lane_width_m,
            config.minimum_half_width_m,
            config.maximum_half_width_m,
        )
    )
    left_width, left_measured = _boundary_widths(
        raw_points,
        left_boundary_m,
        default_half_width_m=default_half_width,
        maximum_half_width_m=config.maximum_half_width_m,
        arc_length_m=s,
        smoothing_window_m=config.width_smoothing_window_m,
    )
    right_width, right_measured = _boundary_widths(
        raw_points,
        right_boundary_m,
        default_half_width_m=default_half_width,
        maximum_half_width_m=config.maximum_half_width_m,
        arc_length_m=s,
        smoothing_window_m=config.width_smoothing_window_m,
    )
    if not config.enabled:
        return PlannedPath(
            path=base_path,
            base_path=base_path,
            lateral_offset_m=np.zeros(raw_points.shape[0]),
            center_correction_m=np.zeros(raw_points.shape[0]),
            left_half_width_m=left_width,
            right_half_width_m=right_width,
            detected_corner_count=0,
            detected_corners=(),
        )

    # The reconstructed BEV center can remain biased toward the boundary that
    # survived a corner. Where both measured boundaries are valid, first move
    # it to the geometric corridor center. The two distances are independent;
    # handmade left/right boundaries do not need to share the same curvature.
    raw_tangent = base_path.tangent_at_s(s)
    raw_normal = np.column_stack((-raw_tangent[:, 1], raw_tangent[:, 0]))
    center_correction = np.zeros(raw_points.shape[0])
    paired = left_measured & right_measured
    center_correction[paired] = 0.5 * (
        left_width[paired] - right_width[paired]
    )
    center_correction = np.clip(
        center_correction,
        -config.maximum_center_correction_m,
        config.maximum_center_correction_m,
    )
    center_correction = _smooth_scalar_profile(
        center_correction, s, config.width_smoothing_window_m
    )
    center_correction = _limit_offset_slope(
        center_correction, s, config.maximum_offset_slope
    )
    centered_points = raw_points + center_correction[:, None] * raw_normal
    centered_path = _model_from_points(
        centered_points, base_path.geometry_window_m
    )
    s = centered_path.arc_length_m
    points = centered_path.points_m

    # Re-measure the free space after center correction. A suspiciously narrow
    # measurement is never inflated to minimum_half_width; optional movement
    # toward that side is disabled instead.
    left_width, _ = _boundary_widths(
        points,
        left_boundary_m,
        default_half_width_m=default_half_width,
        maximum_half_width_m=config.maximum_half_width_m,
        arc_length_m=s,
        smoothing_window_m=config.width_smoothing_window_m,
    )
    right_width, _ = _boundary_widths(
        points,
        right_boundary_m,
        default_half_width_m=default_half_width,
        maximum_half_width_m=config.maximum_half_width_m,
        arc_length_m=s,
        smoothing_window_m=config.width_smoothing_window_m,
    )
    clearance = 0.5 * config.vehicle_width_m + config.safety_margin_m
    available_left = np.where(
        left_width >= config.minimum_half_width_m,
        np.maximum(0.0, left_width - clearance),
        0.0,
    )
    available_right = np.where(
        right_width >= config.minimum_half_width_m,
        np.maximum(0.0, right_width - clearance),
        0.0,
    )

    curvature = np.asarray(centered_path.curvature_at_s(s), dtype=float)
    curvature = _smooth_scalar_profile(
        curvature, s, config.corner_curvature_smoothing_window_m
    )
    corner_mask = np.abs(curvature) >= config.corner_curvature_threshold_per_m
    normalized_offset = np.zeros(points.shape[0])
    detected_corners: list[DetectedCorner] = []
    index = 0
    while index < corner_mask.size:
        if not corner_mask[index]:
            index += 1
            continue
        first = index
        while index + 1 < corner_mask.size and corner_mask[index + 1]:
            index += 1
        last = index
        segment_s = s[first : last + 1]
        segment_curvature = curvature[first : last + 1]
        if segment_s.size < 2:
            index += 1
            continue
        heading_change = abs(
            float(
                np.sum(
                    0.5
                    * (segment_curvature[:-1] + segment_curvature[1:])
                    * np.diff(segment_s)
                )
            )
        )
        if heading_change < math.radians(config.corner_minimum_heading_change_deg):
            index += 1
            continue
        turn_direction = 1.0 if float(np.sum(segment_curvature)) >= 0.0 else -1.0
        apex = first + int(np.argmax(np.abs(segment_curvature)))
        detected_corners.append(
            DetectedCorner(
                start_s_m=float(s[first]),
                apex_s_m=float(s[apex]),
                end_s_m=float(s[last]),
                turn_direction=int(turn_direction),
            )
        )
        outside_reached = max(
            0.0,
            s[first] - config.corner_pre_turn_outside_hold_m,
        )
        setup_transition_length = (
            config.corner_setup_transition_length_m
            if config.corner_setup_transition_length_m > 0.0
            else config.corner_approach_length_m
        )
        approach_start = max(
            0.0,
            outside_reached - setup_transition_length,
        )
        post_corner_hold_end = min(
            centered_path.total_length_m,
            s[last] + config.post_corner_offset_hold_distance_m,
        )
        exit_end = min(
            centered_path.total_length_m,
            post_corner_hold_end + config.corner_exit_length_m,
        )
        outside = -turn_direction * config.outside_offset_fraction
        inside = turn_direction * config.apex_offset_fraction
        anchors: list[tuple[float, float]] = []
        anchors.append(
            (
                approach_start,
                0.0 if approach_start < outside_reached else outside,
            )
        )
        anchors.extend(
            (
                (float(outside_reached), outside),
                (float(s[first]), outside),
                (float(s[apex]), inside),
                (float(s[last]), outside),
                (float(post_corner_hold_end), outside),
            )
        )
        anchors.append(
            (
                exit_end,
                0.0 if exit_end > post_corner_hold_end else outside,
            )
        )
        candidate = _profile_from_anchors(s, anchors)
        use = np.abs(candidate) > np.abs(normalized_offset)
        normalized_offset[use] = candidate[use]
        index += 1

    # Do not return toward the center between nearby same-direction corners.
    # The first corner's outside exit is already the correct setup side for
    # the next corner, so bridge the gap at the full normalized outside value.
    for first_corner, next_corner in zip(
        detected_corners[:-1], detected_corners[1:]
    ):
        gap_m = next_corner.start_s_m - first_corner.end_s_m
        if (
            first_corner.turn_direction != next_corner.turn_direction
            or gap_m < 0.0
            or gap_m > config.same_direction_corner_link_maximum_gap_m
        ):
            continue
        next_outside_reached = max(
            first_corner.end_s_m,
            next_corner.start_s_m - config.corner_pre_turn_outside_hold_m,
        )
        selected = (s >= first_corner.end_s_m) & (
            s <= next_outside_reached
        )
        outside = (
            -float(first_corner.turn_direction)
            * config.outside_offset_fraction
        )
        normalized_offset[selected] = outside

    retained_direction = int(np.sign(retained_outside_turn_direction))
    if retained_direction != 0:
        retained_travel = max(0.0, retained_outside_distance_traveled_m)
        retained_outside = (
            -float(retained_direction) * config.outside_offset_fraction
        )
        next_same_direction = next(
            (
                corner
                for corner in detected_corners
                if corner.turn_direction == retained_direction
                and corner.start_s_m
                <= config.same_direction_corner_link_maximum_gap_m
            ),
            None,
        )
        if next_same_direction is not None:
            next_outside_reached = max(
                0.0,
                next_same_direction.start_s_m
                - config.corner_pre_turn_outside_hold_m,
            )
            link_end = max(
                next_outside_reached,
                next_same_direction.start_s_m,
            )
            retained_candidate = _profile_from_anchors(
                s,
                [(0.0, retained_outside), (link_end, retained_outside)],
            )
        else:
            hold_remaining = max(
                0.0,
                config.post_corner_offset_hold_distance_m - retained_travel,
            )
            return_elapsed = max(
                0.0,
                retained_travel - config.post_corner_offset_hold_distance_m,
            )
            return_length = max(config.corner_exit_length_m, 1.0e-6)
            if hold_remaining > 0.0:
                retained_candidate = _profile_from_anchors(
                    s,
                    [
                        (0.0, retained_outside),
                        (hold_remaining, retained_outside),
                        (hold_remaining + return_length, 0.0),
                    ],
                )
            else:
                return_progress = clamp(
                    return_elapsed / return_length, 0.0, 1.0
                )
                start_scale = 1.0 - float(
                    _smoothstep(np.asarray([return_progress]))[0]
                )
                return_remaining = max(0.0, return_length - return_elapsed)
                retained_candidate = _profile_from_anchors(
                    s,
                    [
                        (0.0, retained_outside * start_scale),
                        (return_remaining, 0.0),
                    ],
                )
        use = np.abs(retained_candidate) > np.abs(normalized_offset)
        normalized_offset[use] = retained_candidate[use]

    available = np.where(normalized_offset >= 0.0, available_left, available_right)
    offset = normalized_offset * np.minimum(config.maximum_offset_m, available)
    offset *= clamp(config.racing_line_weight, 0.0, 1.0)
    offset = _limit_offset_slope(offset, s, config.maximum_offset_slope)
    offset = np.minimum(offset, available_left)
    offset = np.maximum(offset, -available_right)
    tangent = centered_path.tangent_at_s(s)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))

    # Do not let a racing offset make a feasible base path less feasible. If
    # the handmade boundaries already exceed the configured curvature, retain
    # that base requirement but progressively reduce the optional offset.
    base_peak = float(np.max(np.abs(curvature))) if curvature.size else 0.0
    allowed_peak = max(config.maximum_path_curvature_per_m, base_peak)
    scale = 1.0
    planned_model = centered_path
    applied_offset = np.zeros_like(offset)
    while scale >= 0.10:
        candidate_offset = scale * offset
        candidate_points = points + candidate_offset[:, None] * normal
        candidate_model = _model_from_points(
            candidate_points, centered_path.geometry_window_m
        )
        candidate_peak = float(
            np.max(np.abs(candidate_model.curvature_at_s(candidate_model.arc_length_m)))
        )
        planned_model = candidate_model
        applied_offset = candidate_offset
        if candidate_peak <= 1.05 * allowed_peak:
            break
        scale *= 0.80

    return PlannedPath(
        path=planned_model,
        base_path=centered_path,
        lateral_offset_m=applied_offset,
        center_correction_m=center_correction,
        left_half_width_m=left_width,
        right_half_width_m=right_width,
        detected_corner_count=len(detected_corners),
        detected_corners=tuple(detected_corners),
    )


def spatial_speed_plan(
    path: OrderedPathModel,
    *,
    current_speed_mps: float,
    config: SpeedProfileConfig,
) -> SpeedPlan:
    _, closest_s = closest_path_geometry_ordered(path)
    first_s = min(
        path.total_length_m,
        closest_s + max(0.0, config.lookahead_minimum_s_m),
    )
    last_s = min(
        path.total_length_m,
        closest_s + max(config.lookahead_minimum_s_m, config.lookahead_maximum_s_m),
    )
    if last_s <= first_s + 1.0e-6:
        samples = np.asarray([closest_s])
    else:
        count = max(
            2,
            int(math.ceil((last_s - first_s) / max(config.sample_interval_m, 0.01)))
            + 1,
        )
        samples = np.linspace(first_s, last_s, count)
    curvature = np.abs(np.asarray(path.curvature_at_s(samples), dtype=float))
    finite = curvature[np.isfinite(curvature)]
    representative = (
        float(np.percentile(finite, clamp(config.curvature_percentile, 0.0, 100.0)))
        if finite.size
        else 0.0
    )
    if not config.enabled:
        if representative <= 1.0e-6:
            target = config.maximum_speed_mps
        else:
            target = math.sqrt(config.maximum_lateral_acceleration_mps2 / representative)
        target = clamp(target, config.minimum_speed_mps, config.maximum_speed_mps)
        return SpeedPlan(target, representative, samples, np.full(samples.size, target))

    speed_limit = np.full(samples.size, config.maximum_speed_mps)
    valid_curve = curvature > 1.0e-6
    speed_limit[valid_curve] = np.sqrt(
        config.maximum_lateral_acceleration_mps2 / curvature[valid_curve]
    )
    speed_limit = np.clip(
        speed_limit, config.minimum_speed_mps, config.maximum_speed_mps
    )

    # Backward braking pass: impose future corner speeds on every preceding
    # sample. The friction-circle term progressively releases longitudinal
    # braking as lateral acceleration approaches its configured limit.
    profile = speed_limit.copy()
    for index in range(profile.size - 2, -1, -1):
        distance = float(samples[index + 1] - samples[index])
        lateral_acceleration = (
            profile[index + 1] * profile[index + 1] * curvature[index + 1]
        )
        lateral_ratio = clamp(
            lateral_acceleration / config.maximum_lateral_acceleration_mps2,
            0.0,
            1.0,
        )
        combined_ratio = max(
            config.minimum_combined_braking_ratio,
            math.sqrt(max(0.0, 1.0 - lateral_ratio * lateral_ratio)),
        )
        deceleration = config.maximum_longitudinal_deceleration_mps2 * combined_ratio
        reachable = math.sqrt(
            max(0.0, profile[index + 1] ** 2 + 2.0 * deceleration * distance)
        )
        profile[index] = min(profile[index], reachable)

    target = float(profile[0])
    acceleration_distance = max(config.acceleration_control_distance_m, 1.0e-3)
    acceleration_start = max(abs(current_speed_mps), config.minimum_speed_mps)
    acceleration_limit = math.sqrt(
        acceleration_start * acceleration_start
        + 2.0 * config.maximum_longitudinal_acceleration_mps2 * acceleration_distance
    )
    target = min(target, acceleration_limit)
    target = clamp(target, config.minimum_speed_mps, config.maximum_speed_mps)
    return SpeedPlan(target, representative, samples, profile)
