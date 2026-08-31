#!/usr/bin/env python3
"""Simple front-camera white-lane tracker (no BEV, no colour model)."""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray


class FrontLaneDetector(Node):
    """Threshold lower-half grayscale pixels and follow them with windows."""

    def __init__(self) -> None:
        super().__init__("front_lane_detector")
        for name, value in {
            "image_topic": "/camera/image_rect",
            "mask_topic": "/front_lane/mask",
            "overlay_topic": "/front_lane/overlay",
            "model_topic": "/front_lane/model",
            "preview_enabled": True,
            "preview_window_name": "Front lane detection",
            "preview_window_width_px": 960,
            "preview_window_height_px": 540,
            "preview_fps": 30.0,
            "processing_fps": 30.0,
            "publish_debug_images": False,
            "roi_top_ratio": 0.56,
            "roi_bottom_ratio": 1.00,
            "white_threshold": 180,
            "seed_row_ratio": 0.75,
            "seed_band_height_px": 24,
            "seed_max_component_width_px": 160,
            "seed_max_component_pixels": 2000,
            # Once a boundary is acquired, preserve its identity even when
            # the vehicle yaws enough for it to cross the image midpoint.
            "seed_history_max_shift_px": 180,
            "window_count": 20,
            "window_margin_px": 90,
            "minimum_window_pixels": 20,
            "maximum_missing_windows": 2,
            "maximum_component_pixels": 700,
            "maximum_component_width_px": 70,
            "curve_slope_trigger": 0.65,
            "curve_window_height_px": 10,
            "curve_search_extra_px": 50,
            "wide_component_max_width_px": 280,
            "wide_component_max_pixels": 3000,
            "wide_component_orientation_tolerance_deg": 35.0,
            "show_diagnostic_windows": True,
            "show_center_guidance": True,
            "close_target_y_ratio": 0.70,
            "far_target_y_ratio": 0.60,
            "minimum_control_center_points": 5,
            # Camera-visible reference: centre of the white tape on the front bumper.
            "vehicle_reference_x_ratio": 0.501,
            "vehicle_reference_y_ratio": 0.955,
            "width_profile_required_frames": 40,
            "width_profile_min_pairs_per_frame": 6,
            "width_profile_min_samples_per_bin": 12,
            "width_profile_max_samples_per_bin": 80,
            "width_profile_min_ratio": 0.10,
            "width_profile_max_ratio": 0.98,
            "one_side_min_points": 5,
            "virtual_center_max_age_frames": 60,
            "virtual_offset_match_max_y_px": 80,
            "virtual_tangent_span_windows": 2,
            "virtual_normal_min_horizontal_ratio": 0.30,
            "virtual_normal_max_vertical_shift_px": 120,
            # A briefly reappearing opposite boundary must prove that it
            # agrees with the trusted boundary before it replaces virtual
            # centre guidance.
            "side_reacquire_min_pairs": 8,
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.image_topic = str(value("image_topic"))
        self.mask_topic = str(value("mask_topic"))
        self.overlay_topic = str(value("overlay_topic"))
        self.model_topic = str(value("model_topic"))
        self.preview_enabled = bool(value("preview_enabled"))
        self.preview_window_name = str(value("preview_window_name"))
        self.preview_window_width_px = max(
            1, int(value("preview_window_width_px"))
        )
        self.preview_window_height_px = max(
            1, int(value("preview_window_height_px"))
        )
        self.preview_fps = max(1.0, float(value("preview_fps")))
        self.processing_fps = max(1.0, float(value("processing_fps")))
        self.publish_debug_images = bool(value("publish_debug_images"))
        self.roi_top_ratio = float(value("roi_top_ratio"))
        self.roi_bottom_ratio = float(value("roi_bottom_ratio"))
        self.white_threshold = int(value("white_threshold"))
        self.seed_row_ratio = float(value("seed_row_ratio"))
        self.seed_band_height_px = max(3, int(value("seed_band_height_px")))
        self.seed_max_component_width_px = max(
            10, int(value("seed_max_component_width_px"))
        )
        self.window_count = max(4, int(value("window_count")))
        self.window_margin_px = max(10, int(value("window_margin_px")))
        self.minimum_window_pixels = max(3, int(value("minimum_window_pixels")))
        self.seed_max_component_pixels = max(
            self.minimum_window_pixels, int(value("seed_max_component_pixels"))
        )
        self.seed_history_max_shift_px = max(
            20, int(value("seed_history_max_shift_px"))
        )
        self.maximum_missing_windows = max(0, int(value("maximum_missing_windows")))
        self.maximum_component_pixels = max(
            self.minimum_window_pixels, int(value("maximum_component_pixels"))
        )
        self.maximum_component_width_px = max(
            5, int(value("maximum_component_width_px"))
        )
        self.curve_slope_trigger = max(0.05, float(value("curve_slope_trigger")))
        self.curve_window_height_px = max(
            6, int(value("curve_window_height_px"))
        )
        self.curve_search_extra_px = max(0, int(value("curve_search_extra_px")))
        self.wide_component_max_width_px = max(
            self.maximum_component_width_px, int(value("wide_component_max_width_px"))
        )
        self.wide_component_max_pixels = max(
            self.maximum_component_pixels, int(value("wide_component_max_pixels"))
        )
        self.wide_component_orientation_tolerance_rad = np.deg2rad(
            max(1.0, float(value("wide_component_orientation_tolerance_deg")))
        )
        self.show_diagnostic_windows = bool(value("show_diagnostic_windows"))
        self.show_center_guidance = bool(value("show_center_guidance"))
        self.close_target_y_ratio = float(value("close_target_y_ratio"))
        self.far_target_y_ratio = float(value("far_target_y_ratio"))
        self.minimum_control_center_points = max(
            2, int(value("minimum_control_center_points"))
        )
        self.vehicle_reference_x_ratio = float(value("vehicle_reference_x_ratio"))
        self.vehicle_reference_y_ratio = float(value("vehicle_reference_y_ratio"))
        self.width_profile_required_frames = max(1, int(value("width_profile_required_frames")))
        self.width_profile_min_pairs_per_frame = max(2, int(value("width_profile_min_pairs_per_frame")))
        self.width_profile_min_samples_per_bin = max(1, int(value("width_profile_min_samples_per_bin")))
        self.width_profile_max_samples_per_bin = max(
            self.width_profile_min_samples_per_bin, int(value("width_profile_max_samples_per_bin"))
        )
        self.width_profile_min_ratio = float(value("width_profile_min_ratio"))
        self.width_profile_max_ratio = float(value("width_profile_max_ratio"))
        self.one_side_min_points = max(2, int(value("one_side_min_points")))
        self.virtual_center_max_age_frames = max(
            1, int(value("virtual_center_max_age_frames"))
        )
        self.virtual_offset_match_max_y_px = max(
            1, int(value("virtual_offset_match_max_y_px"))
        )
        self.virtual_tangent_span_windows = max(
            1, int(value("virtual_tangent_span_windows"))
        )
        self.virtual_normal_min_horizontal_ratio = float(
            value("virtual_normal_min_horizontal_ratio")
        )
        self.virtual_normal_max_vertical_shift_px = max(
            1, int(value("virtual_normal_max_vertical_shift_px"))
        )
        self.side_reacquire_min_pairs = max(
            self.one_side_min_points, int(value("side_reacquire_min_pairs"))
        )
        self.width_profile_y: np.ndarray | None = None
        self.width_profile_samples: list[list[float]] = []
        self.width_profile_frames = 0
        # The last reliable image-space translation from each boundary to the
        # measured lane centre.  This is intentionally not a lane-width
        # model: it preserves the currently observed curve shape when one
        # boundary disappears.
        self.last_left_center_offsets = np.empty((0, 3), dtype=np.float32)
        self.last_right_center_offsets = np.empty((0, 3), dtype=np.float32)
        self.center_offset_age_frames = self.virtual_center_max_age_frames + 1
        self.previous_left_seed: int | None = None
        self.previous_right_seed: int | None = None
        self.preferred_boundary: str | None = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.mask_pub = self.create_publisher(Image, self.mask_topic, qos)
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, qos)
        self.model_pub = self.create_publisher(Float32MultiArray, self.model_topic, qos)
        self.stop_pub = self.create_publisher(Bool, "/auto/emergency_stop", qos)
        self.image_sub = self.create_subscription(Image, self.image_topic, self._on_image, qos)
        self.next_process_at = 0.0
        self.next_preview_at = 0.0
        if self.preview_enabled:
            cv2.namedWindow(self.preview_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                self.preview_window_name,
                self.preview_window_width_px,
                self.preview_window_height_px,
            )
        self.get_logger().info(
            "Simple lane tracker started: lower-half grayscale threshold + sliding windows."
        )

    @staticmethod
    def _nv12_to_bgr(message: Image) -> np.ndarray:
        if message.encoding.lower() != "nv12":
            raise ValueError(f"expected nv12 image, received '{message.encoding}'")
        stride = max(message.width, message.step)
        rows = message.height * 3 // 2
        data = np.frombuffer(message.data, dtype=np.uint8, count=stride * rows)
        return cv2.cvtColor(
            data.reshape((rows, stride))[:, : message.width], cv2.COLOR_YUV2BGR_NV12
        )

    def _binary_white_mask(self, gray: np.ndarray, top: int, bottom: int) -> np.ndarray:
        _, mask = cv2.threshold(gray, self.white_threshold, 255, cv2.THRESH_BINARY)
        mask[:top, :] = 0
        mask[bottom:, :] = 0
        # Only bridge one- or two-pixel camera/compression gaps.  This does
        # not create a large colour/edge candidate region.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def _seed_from_band(
        self, mask: np.ndarray, y0: int, y1: int, left: bool,
        previous_seed: int | None,
    ) -> int | None:
        """Choose a narrow white stripe, not the brightest large background area.

        The old histogram-only seed could choose a broad white floor/tile area
        at the reference row.  At the seed row we therefore apply the same
        connected-component idea as the tracking windows and prefer temporal
        continuity when a previous valid seed is available.
        """
        count, _, stats, _ = cv2.connectedComponentsWithStats(
            mask[y0:y1, :], connectivity=8
        )
        width = mask.shape[1]
        midpoint = width // 2
        expected_x = (
            float(previous_seed) if previous_seed is not None
            else width * (0.25 if left else 0.75)
        )
        candidates: list[tuple[float, int, int]] = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            component_width = int(stats[label, cv2.CC_STAT_WIDTH])
            if (
                area < self.minimum_window_pixels
                or area > self.seed_max_component_pixels
                or component_width > self.seed_max_component_width_px
            ):
                continue
            center_x = int(round(
                float(stats[label, cv2.CC_STAT_LEFT]) + component_width / 2.0
            ))
            if previous_seed is None:
                if (left and center_x >= midpoint) or (not left and center_x < midpoint):
                    continue
            elif abs(center_x - previous_seed) > self.seed_history_max_shift_px:
                # Do not swap to a distant white object merely because a
                # boundary passed across the image midpoint.
                continue
            candidates.append((abs(center_x - expected_x), -area, center_x))
        if not candidates:
            # A missing seed is safer than reseeding to a large white tile.
            return None
        return min(candidates)[2]

    def _track_direction(
        self, mask: np.ndarray, seed_x: int | None, seed_y: int,
        top: int, bottom: int, direction: int,
    ) -> tuple[list[tuple[int, int]], list[tuple[int, int, int, int, bool]]]:
        if seed_x is None:
            return [], []
        base_height = max(8, (bottom - top) // self.window_count)
        last_x = float(seed_x)
        last_y = float(seed_y)
        # dx/dy is the local tangent in image coordinates.  It is updated
        # from the last successful windows, so window centres can bend with a
        # curve instead of staying vertically above the previous centre.
        slope_x_per_y = 0.0
        misses = 0
        points: list[tuple[int, int]] = []
        windows: list[tuple[int, int, int, int, bool]] = []
        y = seed_y
        while top <= y < bottom:
            strong_curve = abs(slope_x_per_y) >= self.curve_slope_trigger
            step_height = (
                min(base_height, self.curve_window_height_px)
                if strong_curve else base_height
            )
            y0, y1 = (
                (max(top, y - step_height), y) if direction < 0
                else (y, min(bottom, y + step_height))
            )
            if y1 <= y0:
                break
            candidate_y = 0.5 * (y0 + y1)
            predicted_delta_x = slope_x_per_y * (candidate_y - last_y)
            predicted_x = last_x + predicted_delta_x
            # Keep the normal window narrow.  Only the predicted turn side
            # receives extra room in a strong curve.
            left_margin = self.window_margin_px
            right_margin = self.window_margin_px
            if strong_curve:
                if predicted_delta_x < 0:
                    left_margin += self.curve_search_extra_px
                elif predicted_delta_x > 0:
                    right_margin += self.curve_search_extra_px
            x0 = max(0, int(np.floor(predicted_x - left_margin)))
            x1 = min(mask.shape[1], int(np.ceil(predicted_x + right_margin + 1)))
            if x1 <= x0:
                break
            count, labels, stats, _ = cv2.connectedComponentsWithStats(
                mask[y0:y1, x0:x1], connectivity=8
            )
            candidates: list[tuple[float, float]] = []
            for label in range(1, count):
                area = int(stats[label, cv2.CC_STAT_AREA])
                component_width = int(stats[label, cv2.CC_STAT_WIDTH])
                wide_component = (
                    component_width > self.maximum_component_width_px
                    or area > self.maximum_component_pixels
                )
                if area < self.minimum_window_pixels:
                    continue
                if not wide_component:
                    center_x = x0 + float(stats[label, cv2.CC_STAT_LEFT]) + 0.5 * component_width
                    center_y = float(stats[label, cv2.CC_STAT_TOP] + y0) + 0.5 * float(stats[label, cv2.CC_STAT_HEIGHT])
                    candidates.append((center_x, center_y))
                    continue
                # A nearly horizontal lane becomes wide in a thin horizontal
                # band.  Permit it only during a strong turn and only if its
                # local orientation agrees with the predicted lane tangent.
                if (
                    not strong_curve
                    or component_width > self.wide_component_max_width_px
                    or area > self.wide_component_max_pixels
                ):
                    continue
                component_y, component_x = np.nonzero(labels == label)
                if component_x.size < self.minimum_window_pixels:
                    continue
                line = cv2.fitLine(
                    np.column_stack((component_x + x0, component_y + y0)).astype(np.float32),
                    cv2.DIST_L2, 0, 0.01, 0.01,
                )
                vx, vy = float(line[0]), float(line[1])
                expected_angle = np.arctan2(1.0, slope_x_per_y)
                component_angle = np.arctan2(vy, vx)
                angle_difference = abs((component_angle - expected_angle + np.pi / 2) % np.pi - np.pi / 2)
                if angle_difference > self.wide_component_orientation_tolerance_rad:
                    continue
                global_x = component_x + x0
                nearby = global_x[np.abs(global_x - predicted_x) <= max(12, step_height * 2)]
                if nearby.size < self.minimum_window_pixels:
                    continue
                center_x = float(np.median(nearby))
                center_y = float(np.median(component_y + y0))
                candidates.append((center_x, center_y))
            if candidates:
                # Follow the component closest to the tangent-predicted
                # position, not merely the preceding vertical window.
                next_x, next_y = min(
                    candidates, key=lambda item: abs(item[0] - predicted_x)
                )
                points.append((int(round(next_x)), int(round(next_y))))
                delta_y = next_y - last_y
                if abs(delta_y) > 1e-3:
                    measured_slope = np.clip(
                        (next_x - last_x) / delta_y, -8.0, 8.0
                    )
                    slope_x_per_y = 0.6 * slope_x_per_y + 0.4 * measured_slope
                last_x, last_y = next_x, next_y
                misses = 0
                windows.append((x0, x1, y0, y1, True))
            else:
                windows.append((x0, x1, y0, y1, False))
                misses += 1
                if misses > self.maximum_missing_windows:
                    break
            y += direction * step_height
        return points, windows

    def _track_lane(
        self, mask: np.ndarray, seed_x: int | None, seed_y: int,
        top: int, bottom: int,
    ) -> tuple[np.ndarray, list[tuple[int, int, int, int, bool]]]:
        upward, up_windows = self._track_direction(
            mask, seed_x, seed_y, top, bottom, -1
        )
        downward, down_windows = self._track_direction(
            mask, seed_x, seed_y, top, bottom, 1
        )
        points = upward + downward
        return (
            np.asarray(points, dtype=np.int32)
            if points else np.empty((0, 2), dtype=np.int32),
            up_windows + down_windows,
        )

    @staticmethod
    def _fit(points: np.ndarray) -> np.ndarray | None:
        if points.shape[0] < 5:
            return None
        return np.polyfit(points[:, 1], points[:, 0], 2)

    @staticmethod
    def _matched_lane_pairs(
        left: np.ndarray, right: np.ndarray, max_y_difference: int,
    ) -> np.ndarray:
        """Pair only neighbouring sliding-window points at similar heights.

        This deliberately uses measured midpoints rather than fitting a new
        curve, so the diagnostic path exposes the actual tracking result.
        """
        if left.size == 0 or right.size == 0:
            return np.empty((0, 6), dtype=np.int32)
        right_ordered = right[np.argsort(right[:, 1])]
        pairs: list[tuple[int, int, int, int, int, int]] = []
        for left_x, left_y in left[np.argsort(left[:, 1])]:
            nearest = int(np.argmin(np.abs(right_ordered[:, 1] - left_y)))
            right_x, right_y = right_ordered[nearest]
            if abs(int(right_y) - int(left_y)) > max_y_difference:
                continue
            if int(right_x) <= int(left_x):
                continue
            pairs.append((
                int(left_x), int(left_y), int(right_x), int(right_y),
                int(round((int(left_x) + int(right_x)) / 2.0)),
                int(round((int(left_y) + int(right_y)) / 2.0)),
            ))
        return np.asarray(pairs, dtype=np.int32) if pairs else np.empty((0, 6), dtype=np.int32)

    def _inward_normal(
        self, ordered_points: np.ndarray, index: int, side: str,
    ) -> np.ndarray | None:
        """Return the image-space normal pointing from a boundary into the lane."""
        lower = max(0, index - self.virtual_tangent_span_windows)
        upper = min(ordered_points.shape[0] - 1, index + self.virtual_tangent_span_windows)
        tangent = (
            ordered_points[upper].astype(np.float32)
            - ordered_points[lower].astype(np.float32)
        )
        tangent_norm = float(np.linalg.norm(tangent))
        if tangent_norm < 1e-3:
            return None
        tangent /= tangent_norm
        normal = np.array([-tangent[1], tangent[0]], dtype=np.float32)
        if (side == "left" and normal[0] < 0.0) or (side == "right" and normal[0] > 0.0):
            normal *= -1.0
        return normal

    def _normal_width_samples(self, pairs: np.ndarray, left: np.ndarray) -> np.ndarray:
        """Measure actual two-lane spacing projected onto the left-line normal."""
        if pairs.size == 0 or left.size == 0:
            return np.empty((0, 2), dtype=np.float32)
        ordered_left = left[np.argsort(left[:, 1])]
        samples: list[tuple[float, float]] = []
        for left_x, left_y, right_x, right_y, _, center_y in pairs:
            distances = np.abs(ordered_left[:, 0] - left_x) + np.abs(ordered_left[:, 1] - left_y)
            index = int(np.argmin(distances))
            normal = self._inward_normal(ordered_left, index, "left")
            if normal is None:
                continue
            displacement = np.array(
                [float(right_x - left_x), float(right_y - left_y)], dtype=np.float32
            )
            normal_width = float(np.dot(displacement, normal))
            if normal_width > 0.0:
                samples.append((float(center_y), normal_width))
        return np.asarray(samples, dtype=np.float32) if samples else np.empty((0, 2), dtype=np.float32)

    def _ensure_width_profile(self, top: int, bottom: int) -> None:
        if self.width_profile_y is not None:
            return
        window_height = max(8, (bottom - top) // self.window_count)
        self.width_profile_y = np.arange(
            top + window_height // 2, bottom, window_height, dtype=np.int32
        )
        self.width_profile_samples = [[] for _ in self.width_profile_y]

    def _profile_width_at(self, y: int) -> float | None:
        if self.width_profile_y is None or not self.width_profile_samples:
            return None
        index = int(np.argmin(np.abs(self.width_profile_y - y)))
        samples = self.width_profile_samples[index]
        if len(samples) < self.width_profile_min_samples_per_bin:
            return None
        return float(np.median(samples))

    def _profile_ready(self) -> bool:
        ready_bins = sum(
            len(samples) >= self.width_profile_min_samples_per_bin
            for samples in self.width_profile_samples
        )
        return (
            self.width_profile_y is not None
            and self.width_profile_frames >= self.width_profile_required_frames
            and ready_bins >= max(4, len(self.width_profile_samples) // 2)
        )

    def _learn_width_profile(self, normal_widths: np.ndarray, image_width: int) -> None:
        """Learn a robust normal-direction lane width for each image-height bin."""
        if self.width_profile_y is None or normal_widths.shape[0] < self.width_profile_min_pairs_per_frame:
            return
        valid = [sample for sample in normal_widths if (
            self.width_profile_min_ratio * image_width <= sample[1] <= self.width_profile_max_ratio * image_width
        )]
        if len(valid) < self.width_profile_min_pairs_per_frame:
            return
        self.width_profile_frames = min(
            self.width_profile_required_frames, self.width_profile_frames + 1
        )
        for y, normal_width in valid:
            index = int(np.argmin(np.abs(self.width_profile_y - y)))
            samples = self.width_profile_samples[index]
            samples.append(float(normal_width))
            if len(samples) > self.width_profile_max_samples_per_bin:
                del samples[0]

    def _remember_center_offsets(self, pairs: np.ndarray) -> None:
        """Save the latest measured boundary-to-centre translations.

        Each row is ``[boundary_y, centre_x - boundary_x, centre_y -
        boundary_y]``.  It is an image-space parallel translation captured
        while both boundaries are visible, so it contains the current camera
        perspective and curve geometry without re-estimating a width after a
        boundary vanishes.
        """
        if pairs.shape[0] < self.one_side_min_points:
            self.center_offset_age_frames += 1
            return
        left_offsets = np.column_stack((
            pairs[:, 1], pairs[:, 4] - pairs[:, 0], pairs[:, 5] - pairs[:, 1],
        )).astype(np.float32)
        right_offsets = np.column_stack((
            pairs[:, 3], pairs[:, 4] - pairs[:, 2], pairs[:, 5] - pairs[:, 3],
        )).astype(np.float32)
        self.last_left_center_offsets = left_offsets[np.argsort(left_offsets[:, 0])]
        self.last_right_center_offsets = right_offsets[np.argsort(right_offsets[:, 0])]
        self.center_offset_age_frames = 0

    def _center_offset_at(self, side: str, y: int) -> tuple[float, float] | None:
        """Return the latest valid translation for a source point height."""
        if self.center_offset_age_frames > self.virtual_center_max_age_frames:
            return None
        offsets = (
            self.last_left_center_offsets if side == "left"
            else self.last_right_center_offsets
        )
        if offsets.size == 0:
            return None
        index = int(np.argmin(np.abs(offsets[:, 0] - y)))
        if abs(float(offsets[index, 0]) - y) > self.virtual_offset_match_max_y_px:
            return None
        return float(offsets[index, 1]), float(offsets[index, 2])

    def _virtual_centers(
        self, left: np.ndarray, right: np.ndarray, real_centers: np.ndarray,
        max_y_difference: int, image_width: int, top: int, bottom: int,
    ) -> np.ndarray:
        """Translate the surviving boundary by the last measured centre offset."""
        # A virtual centre is allowed only when one side is clearly present
        # and the other is clearly absent.  Never mix two virtual paths while
        # both real boundaries are visible but imperfectly paired.
        if left.shape[0] >= self.one_side_min_points and right.shape[0] < self.one_side_min_points:
            source, side = left, "left"
        elif right.shape[0] >= self.one_side_min_points and left.shape[0] < self.one_side_min_points:
            source, side = right, "right"
        else:
            return np.empty((0, 2), dtype=np.int32)

        ordered = source[np.argsort(source[:, 1])]
        virtual: list[tuple[int, int]] = []

        def has_real_center(y: int) -> bool:
            return bool(real_centers.size and np.min(np.abs(real_centers[:, 1] - y)) <= max_y_difference)

        for x, y in ordered:
            if has_real_center(int(y)):
                continue
            offset = self._center_offset_at(side, int(y))
            if offset is None:
                continue
            offset_x, offset_y = offset
            center_x = int(round(float(x) + offset_x))
            center_y = int(round(float(y) + offset_y))
            if 0 <= center_x < image_width and top <= center_y < bottom:
                virtual.append((center_x, center_y))
        return np.asarray(virtual, dtype=np.int32) if virtual else np.empty((0, 2), dtype=np.int32)

    @staticmethod
    def _nearest_y_point(points: np.ndarray, target_y: int) -> tuple[int, int] | None:
        if points.size == 0:
            return None
        index = int(np.argmin(np.abs(points[:, 1] - target_y)))
        return tuple(map(int, points[index]))

    @staticmethod
    def _image_message(source: Image, image: np.ndarray, encoding: str) -> Image:
        msg = Image()
        msg.header = source.header
        msg.height, msg.width = image.shape[:2]
        msg.encoding = encoding
        msg.step = image.strides[0]
        msg.data = image.tobytes()
        return msg

    def _draw_tracking_windows(
        self,
        overlay: np.ndarray,
        left_windows: list[tuple[int, int, int, int, bool]],
        right_windows: list[tuple[int, int, int, int, bool]],
    ) -> None:
        """Draw detector-specific search windows on the shared preview."""
        if not self.show_diagnostic_windows:
            return
        height, width = overlay.shape[:2]
        for windows, color in (
            (left_windows, (255, 150, 0)),
            (right_windows, (0, 150, 255)),
        ):
            for x0, x1, y0, y1, found in windows:
                cv2.rectangle(
                    overlay,
                    (x0, y0),
                    (min(width - 1, x1 - 1), min(height - 1, y1)),
                    color if found else (80, 80, 80),
                    1,
                )

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now < self.next_process_at:
            return
        self.next_process_at = now + 1.0 / self.processing_fps
        try:
            bgr = self._nv12_to_bgr(message)
            height, width = bgr.shape[:2]
            top = int(np.clip(self.roi_top_ratio * height, 0, height - 3))
            bottom = int(np.clip(self.roi_bottom_ratio * height, top + 2, height))
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            mask = self._binary_white_mask(gray, top, bottom)
            seed_y = int(np.clip(self.seed_row_ratio * height, top, bottom - 1))
            band = self.seed_band_height_px // 2
            seed_y0 = max(top, seed_y - band)
            seed_y1 = min(bottom, seed_y + band + 1)
            vehicle_point = (
                int(np.clip(self.vehicle_reference_x_ratio * width, 0, width - 1)),
                int(np.clip(self.vehicle_reference_y_ratio * height, 0, height - 1)),
            )
            left_seed = self._seed_from_band(
                mask, seed_y0, seed_y1, True, self.previous_left_seed
            )
            right_seed = self._seed_from_band(
                mask, seed_y0, seed_y1, False, self.previous_right_seed
            )
            # Start with the seed closest to the vehicle reference.  The
            # detector still preserves left/right identity in the result.
            seeds = (("left", left_seed), ("right", right_seed))
            ordered_seeds = sorted(
                seeds,
                key=lambda item: (
                    float("inf") if item[1] is None else
                    (item[1] - vehicle_point[0]) ** 2
                    + (seed_y - vehicle_point[1]) ** 2
                ),
            )
            tracked: dict[
                str,
                tuple[np.ndarray, list[tuple[int, int, int, int, bool]]],
            ] = {}
            for side, seed_x in ordered_seeds:
                tracked[side] = self._track_lane(
                    mask, seed_x, seed_y, top, bottom
                )
            left, left_windows = tracked["left"]
            right, right_windows = tracked["right"]
            if left.shape[0] >= 5:
                self.previous_left_seed = int(
                    self._nearest_y_point(left, seed_y)[0]
                )
            if right.shape[0] >= 5:
                self.previous_right_seed = int(
                    self._nearest_y_point(right, seed_y)[0]
                )
            window_height = max(8, (bottom - top) // self.window_count)
            raw_pairs = self._matched_lane_pairs(left, right, max_y_difference=window_height)

            left_ready = left.shape[0] >= self.one_side_min_points
            right_ready = right.shape[0] >= self.one_side_min_points
            if left_ready and not right_ready:
                self.preferred_boundary = "left"
            elif right_ready and not left_ready:
                self.preferred_boundary = "right"
            elif self.preferred_boundary == "left" and left_ready:
                # Keep following the proven left boundary until the newly
                # seen right boundary yields enough geometrically matched
                # pairs.  This prevents a crossed/image-side-swapped stripe
                # from abruptly becoming the right lane.
                if raw_pairs.shape[0] < self.side_reacquire_min_pairs:
                    right = np.empty((0, 2), dtype=np.int32)
                else:
                    self.preferred_boundary = None
            elif self.preferred_boundary == "right" and right_ready:
                if raw_pairs.shape[0] < self.side_reacquire_min_pairs:
                    left = np.empty((0, 2), dtype=np.int32)
                else:
                    self.preferred_boundary = None
            elif left_ready and right_ready:
                self.preferred_boundary = None

            self._ensure_width_profile(top, bottom)
            pairs = self._matched_lane_pairs(left, right, max_y_difference=window_height)
            normal_widths = self._normal_width_samples(pairs, left)
            self._learn_width_profile(normal_widths, width)
            real_centers = pairs[:, 4:6] if pairs.size else np.empty((0, 2), dtype=np.int32)
            self._remember_center_offsets(pairs)
            virtual_centers = self._virtual_centers(
                left, right, real_centers, max_y_difference=window_height,
                image_width=width, top=top, bottom=bottom,
            )
            centers = np.vstack((real_centers, virtual_centers)) if virtual_centers.size else real_centers
            close_target_y = int(np.clip(self.close_target_y_ratio * height, top, bottom - 1))
            far_target_y = int(np.clip(self.far_target_y_ratio * height, top, bottom - 1))
            close_target = self._nearest_y_point(centers, close_target_y)
            far_target = self._nearest_y_point(centers, far_target_y)
            # CLOSE is the immediate steering target. FAR remains a visual
            # diagnostic/next-stage curve target, so it must not invalidate
            # driving when both selected points happen to coincide.
            control_valid = bool(
                centers.shape[0] >= self.minimum_control_center_points
                and close_target is not None
            )
            # Heading is measured from the vehicle's forward image direction.
            # Positive is right, negative is left.  Unlike a plain x-offset,
            # this naturally commands more steering when the Vehicle->CLOSE
            # line becomes more horizontal.
            close_heading = (
                float(np.arctan2(
                    close_target[0] - vehicle_point[0],
                    max(1, vehicle_point[1] - close_target[1]),
                ))
                if close_target is not None else 0.0
            )
            far_error = (
                (far_target[0] - vehicle_point[0]) / (width / 2.0)
                if far_target is not None else 0.0
            )
            # Compare the current Vehicle->CLOSE direction with the next
            # CLOSE->FAR path direction.  Their angular difference indicates
            # a curve ahead before CLOSE itself reaches the bend.
            curve_ahead = 0.0
            if close_target is not None and far_target is not None:
                path_heading = float(np.arctan2(
                    far_target[0] - close_target[0],
                    max(1, close_target[1] - far_target[1]),
                ))
                curve_ahead = float(np.arctan2(
                    np.sin(path_heading - close_heading),
                    np.cos(path_heading - close_heading),
                ))
            # Any virtual points mean at least one boundary is incomplete;
            # the drive node should reduce duty for that interval.
            virtual_mode = bool(virtual_centers.shape[0] > 0)

            overlay = bgr.copy()
            cv2.line(overlay, (0, top), (width - 1, top), (0, 165, 255), 2)
            # With the lower ROI edge opened to the image bottom there is no
            # artificial bottom boundary to display.
            if bottom < height:
                cv2.line(overlay, (0, bottom - 1), (width - 1, bottom - 1), (0, 165, 255), 2)
            cv2.line(overlay, (0, seed_y), (width - 1, seed_y), (0, 255, 255), 1)
            if left_seed is not None:
                cv2.circle(overlay, (left_seed, seed_y), 8, (255, 0, 0), -1)
            if right_seed is not None:
                cv2.circle(overlay, (right_seed, seed_y), 8, (0, 0, 255), -1)
            self._draw_tracking_windows(overlay, left_windows, right_windows)
            for points, color in ((left, (255, 0, 0)), (right, (0, 0, 255))):
                if points.shape[0] >= 2:
                    ordered = points[np.argsort(points[:, 1])]
                    cv2.polylines(overlay, [ordered], False, color, 5, cv2.LINE_AA)
                    for point in ordered:
                        cv2.circle(overlay, tuple(point), 5, color, -1)

            if self.show_center_guidance:
                # Do not move the lane centreline to compensate for the camera
                # viewpoint.  Use the measured bumper-tape position as the
                # vehicle reference from which the guidance line starts.
                if centers.shape[0] >= 2:
                    ordered_centers = centers[np.argsort(centers[:, 1])]
                    cv2.polylines(overlay, [ordered_centers], False, (255, 255, 0), 3, cv2.LINE_AA)
                    for point in real_centers[np.argsort(real_centers[:, 1])]:
                        cv2.circle(overlay, tuple(point), 4, (255, 255, 0), -1)
                    for point in virtual_centers:
                        cv2.circle(overlay, tuple(point), 5, (255, 0, 255), -1)
                cv2.circle(overlay, vehicle_point, 10, (0, 255, 0), -1)
                cv2.putText(overlay, "vehicle", (vehicle_point[0] + 12, vehicle_point[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
                if close_target is not None:
                    cv2.circle(overlay, close_target, 9, (0, 255, 255), -1)
                    cv2.line(overlay, vehicle_point, close_target, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(overlay, "close", (close_target[0] + 10, close_target[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
                if far_target is not None:
                    cv2.circle(overlay, far_target, 9, (255, 0, 255), -1)
                    if close_target is not None:
                        cv2.line(overlay, close_target, far_target, (0, 255, 0), 2, cv2.LINE_AA)
                    else:
                        cv2.line(overlay, vehicle_point, far_target, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(overlay, "far", (far_target[0] + 10, far_target[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)
                ready_bins = sum(
                    len(samples) >= self.width_profile_min_samples_per_bin
                    for samples in self.width_profile_samples
                )
                total_bins = len(self.width_profile_samples)
                profile_state = (
                    "READY" if self._profile_ready()
                    else f"learning {self.width_profile_frames}/{self.width_profile_required_frames}"
                )
                offset_state = (
                    "fresh" if self.center_offset_age_frames == 0
                    else f"hold {self.center_offset_age_frames}/{self.virtual_center_max_age_frames}"
                )
                cv2.putText(overlay, f"center pairs: {real_centers.shape[0]}  virtual: {virtual_centers.shape[0]}", (20, height - 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(overlay, f"width profile: {profile_state}  bins {ready_bins}/{total_bins}", (20, height - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(overlay, f"centre offset: {offset_state}", (20, height - 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2, cv2.LINE_AA)
                control_text = "control: READY" if control_valid else "control: waiting"
                cv2.putText(overlay, control_text, (20, height - 95),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 0) if control_valid else (0, 165, 255), 2, cv2.LINE_AA)
                heading_text = f"close heading: {np.degrees(close_heading):+.1f} deg"
                cv2.putText(overlay, heading_text, (20, height - 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(overlay, f"curve ahead: {np.degrees(curve_ahead):+.1f} deg", (20, height - 145),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)

            left_fit, right_fit = self._fit(left), self._fit(right)
            confidence = 1.0 if left_fit is not None and right_fit is not None else 0.0
            lateral = lookahead = curvature = 0.0
            if confidence:
                center_fit = 0.5 * (left_fit + right_fit)
                reference_y = 0.75 * height
                lookahead_y = top + 0.25 * (height - top)
                lateral = (np.polyval(center_fit, reference_y) - width / 2) / (width / 2)
                lookahead = (np.polyval(center_fit, lookahead_y) - width / 2) / (width / 2)
                a, b, _ = center_fit
                slope = 2.0 * a * lookahead_y + b
                curvature = float(2.0 * a / ((1.0 + slope * slope) ** 1.5))

            if self.publish_debug_images:
                self.mask_pub.publish(self._image_message(message, mask, "mono8"))
                self.overlay_pub.publish(self._image_message(message, overlay, "bgr8"))
            if self.preview_enabled and now >= self.next_preview_at:
                cv2.imshow(self.preview_window_name, overlay)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord(" "), ord("e"), ord("E")):
                    self.stop_pub.publish(Bool(data=True))
                    self.get_logger().warn("SPACE/E pressed in preview: emergency stop requested.")
                self.next_preview_at = now + 1.0 / self.preview_fps
            # Model layout for lane_assist_drive:
            # valid, close_heading_rad, far_error, virtual_mode, centre_count,
            # real_pair_count, curve_ahead_rad.
            self.model_pub.publish(Float32MultiArray(data=[
                float(control_valid), float(close_heading), float(far_error),
                float(virtual_mode), float(centers.shape[0]), float(real_centers.shape[0]),
                float(curve_ahead),
            ]))
        except Exception as error:
            self.get_logger().error(f"Front lane frame rejected: {error}")

    def destroy_node(self) -> bool:
        if self.preview_enabled:
            cv2.destroyWindow(self.preview_window_name)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontLaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
