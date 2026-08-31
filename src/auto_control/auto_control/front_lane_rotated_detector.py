#!/usr/bin/env python3
"""Front-lane tracker using shrinking rotated search windows from each seed."""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy

from auto_control.front_lane_detector import FrontLaneDetector
from sensor_msgs.msg import Image


class RotatedWindowFrontLaneDetector(FrontLaneDetector):
    """Track away from the vehicle using an oriented, shrinking window."""

    def __init__(self) -> None:
        super().__init__()
        for name, value in {
            # Width is intentionally a little longer than height. Both
            # dimensions shrink toward the far end of the image.
            "rotated_window_initial_width_px": 120,
            "rotated_window_initial_height_px": 90,
            "rotated_window_min_width_px": 54,
            "rotated_window_min_height_px": 40,
            "rotated_window_shrink_ratio": 0.94,
            "rotated_window_step_ratio": 0.60,
            "rotated_max_turn_deg_per_window": 18.0,
            "rotated_max_turn_change_deg_per_window": 10.0,
            "rotated_heading_update_gain": 0.45,
        }.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.rotated_window_initial_width_px = max(
            8, int(value("rotated_window_initial_width_px"))
        )
        self.rotated_window_initial_height_px = max(
            8, int(value("rotated_window_initial_height_px"))
        )
        self.rotated_window_min_width_px = max(
            4, min(
                self.rotated_window_initial_width_px,
                int(value("rotated_window_min_width_px")),
            )
        )
        self.rotated_window_min_height_px = max(
            4, min(
                self.rotated_window_initial_height_px,
                int(value("rotated_window_min_height_px")),
            )
        )
        self.rotated_window_shrink_ratio = float(np.clip(
            float(value("rotated_window_shrink_ratio")), 0.50, 1.0
        ))
        self.rotated_window_step_ratio = float(np.clip(
            float(value("rotated_window_step_ratio")), 0.20, 1.0
        ))
        if (
            self.rotated_window_initial_width_px
            <= self.rotated_window_initial_height_px
        ):
            self.get_logger().warn(
                "rotated window width should be slightly larger than height"
            )
        if self.rotated_window_min_width_px <= self.rotated_window_min_height_px:
            self.get_logger().warn(
                "minimum rotated window width should be larger than height"
            )
        self.rotated_max_turn_rad = math.radians(
            max(1.0, float(value("rotated_max_turn_deg_per_window")))
        )
        self.rotated_max_turn_change_rad = math.radians(
            max(0.5, float(value("rotated_max_turn_change_deg_per_window")))
        )
        self.rotated_heading_update_gain = float(np.clip(
            float(value("rotated_heading_update_gain")), 0.05, 1.0
        ))
        self._rotated_polygons: list[tuple[np.ndarray, bool]] = []
        self.get_logger().info(
            "Rotated tracker ready: nearest seed first, shrinking windows toward far field."
        )

    @staticmethod
    def _unit(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        return vector / norm if norm > 1e-6 else fallback.copy()

    @staticmethod
    def _signed_turn(first: np.ndarray, second: np.ndarray) -> float:
        """Signed image-plane rotation from ``first`` to ``second``."""
        cross = float(first[0] * second[1] - first[1] * second[0])
        dot = float(np.clip(np.dot(first, second), -1.0, 1.0))
        return float(math.atan2(cross, dot))

    @staticmethod
    def _rotate(vector: np.ndarray, angle: float) -> np.ndarray:
        cosine = math.cos(angle)
        sine = math.sin(angle)
        return np.array(
            (
                cosine * vector[0] - sine * vector[1],
                sine * vector[0] + cosine * vector[1],
            ),
            dtype=np.float32,
        )

    def _find_region_candidate(
        self,
        mask: np.ndarray,
        polygon: np.ndarray,
        predicted: np.ndarray,
        tangent: np.ndarray,
        normal: np.ndarray,
    ) -> tuple[float, float] | None:
        """Return the connected white stripe closest to the predicted path."""
        height, width = mask.shape[:2]
        x0 = max(0, int(np.floor(np.min(polygon[:, 0]))))
        x1 = min(width, int(np.ceil(np.max(polygon[:, 0]))) + 1)
        y0 = max(0, int(np.floor(np.min(polygon[:, 1]))))
        y1 = min(height, int(np.ceil(np.max(polygon[:, 1])) + 1))
        if x1 <= x0 or y1 <= y0:
            return None

        local_polygon = np.round(polygon - np.array([x0, y0])).astype(np.int32)
        region = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
        cv2.fillConvexPoly(region, local_polygon, 255)
        search = cv2.bitwise_and(mask[y0:y1, x0:x1], region)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(search, connectivity=8)
        candidates: list[tuple[float, float, float]] = []
        max_area = max(self.maximum_component_pixels * 5, 3000)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.minimum_window_pixels or area > max_area:
                continue
            ys, xs = np.nonzero(labels == label)
            if xs.size < self.minimum_window_pixels:
                continue
            coordinates = np.column_stack((xs + x0, ys + y0)).astype(np.float32)
            # This is the actual centre of the detected white stripe inside
            # the oriented window, equivalent to the response centroid used
            # by the BEV implementation.  The mask is binary, so its mean is
            # the weighted centroid with equal white-pixel weights.
            point = np.mean(coordinates, axis=0)
            offset = point - predicted
            normal_error = abs(float(np.dot(offset, normal)))
            longitudinal_error = abs(float(np.dot(offset, tangent)))
            # Normal closeness keeps us on the predicted boundary; a small
            # longitudinal term prefers the next section, not a remote blob.
            score = normal_error + 0.20 * longitudinal_error
            candidates.append((score, float(point[0]), float(point[1])))
        if not candidates:
            return None
        _, x, y = min(candidates, key=lambda item: item[0])
        return x, y

    def _rotated_polygon(
        self,
        center: np.ndarray,
        tangent: np.ndarray,
        window_width: float,
        window_height: float,
    ) -> np.ndarray:
        normal = np.array((-tangent[1], tangent[0]), dtype=np.float32)
        half_height = 0.5 * window_height
        half_width = 0.5 * window_width
        return np.asarray((
            center - tangent * half_height - normal * half_width,
            center + tangent * half_height - normal * half_width,
            center + tangent * half_height + normal * half_width,
            center - tangent * half_height + normal * half_width,
        ), dtype=np.float32)

    def _track_direction(
        self, mask: np.ndarray, seed_x: int | None, seed_y: int,
        top: int, bottom: int, direction: int,
    ) -> tuple[list[tuple[int, int]], list[tuple[int, int, int, int, bool]]]:
        if seed_x is None:
            return [], []

        height, width = mask.shape[:2]
        last = np.array((float(seed_x), float(seed_y)), dtype=np.float32)
        tangent = np.array((0.0, float(direction)), dtype=np.float32)
        misses = 0
        previous_applied_turn = 0.0
        points: list[tuple[int, int]] = []
        windows: list[tuple[int, int, int, int, bool]] = []

        # Window zero is centred on the seed nearest the vehicle. Every next
        # window advances toward the far field along the measured tangent.
        for window_index in range(self.window_count):
            scale = self.rotated_window_shrink_ratio ** window_index
            window_width = max(
                float(self.rotated_window_min_width_px),
                float(self.rotated_window_initial_width_px) * scale,
            )
            window_height = max(
                float(self.rotated_window_min_height_px),
                float(self.rotated_window_initial_height_px) * scale,
            )
            if window_index == 0:
                predicted = last.copy()
            else:
                step = max(4.0, window_height * self.rotated_window_step_ratio)
                predicted = last + tangent * step
            if not (
                -0.5 * window_width <= predicted[0] < width + 0.5 * window_width
                and top - 0.5 * window_height
                <= predicted[1]
                < bottom + 0.5 * window_height
            ):
                break
            normal = np.array((-tangent[1], tangent[0]), dtype=np.float32)
            polygon = self._rotated_polygon(
                predicted, tangent, window_width, window_height
            )

            candidate = self._find_region_candidate(
                mask, polygon, predicted, tangent, normal
            )
            clipped_polygon = polygon.copy()
            clipped_polygon[:, 0] = np.clip(clipped_polygon[:, 0], 0, width - 1)
            clipped_polygon[:, 1] = np.clip(clipped_polygon[:, 1], top, bottom - 1)
            self._rotated_polygons.append(
                (clipped_polygon, candidate is not None)
            )
            x0 = int(np.floor(np.min(clipped_polygon[:, 0])))
            x1 = int(np.ceil(np.max(clipped_polygon[:, 0])))
            y0 = int(np.floor(np.min(clipped_polygon[:, 1])))
            y1 = int(np.ceil(np.max(clipped_polygon[:, 1])))
            windows.append((x0, x1, y0, y1, candidate is not None))

            if candidate is None:
                misses += 1
                if misses > self.maximum_missing_windows:
                    break
                # Preserve the estimated tangent during a short gap.
                last = predicted
                continue

            next_point = np.array(candidate, dtype=np.float32)
            movement = next_point - last
            measured = (
                self._unit(movement, tangent)
                if window_index > 0 else tangent.copy()
            )
            # Never let a noisy candidate reverse the intended travel axis.
            if measured[1] * direction < -0.05:
                measured = tangent.copy()

            # Rotate gradually toward the measured stripe direction. A noisy
            # component cannot change the heading abruptly.
            desired_turn = float(np.clip(
                self._signed_turn(tangent, measured),
                -self.rotated_max_turn_rad,
                self.rotated_max_turn_rad,
            ))
            desired_turn = float(np.clip(
                desired_turn,
                previous_applied_turn - self.rotated_max_turn_change_rad,
                previous_applied_turn + self.rotated_max_turn_change_rad,
            ))
            applied_turn = previous_applied_turn + self.rotated_heading_update_gain * (
                desired_turn - previous_applied_turn
            )
            tangent = self._unit(self._rotate(tangent, applied_turn), tangent)
            previous_applied_turn = applied_turn
            points.append((int(round(next_point[0])), int(round(next_point[1]))))
            last = next_point
            misses = 0
        return points, windows

    def _track_lane(
        self, mask: np.ndarray, seed_x: int | None, seed_y: int,
        top: int, bottom: int,
    ) -> tuple[np.ndarray, list[tuple[int, int, int, int, bool]]]:
        # Only move away from the vehicle. The former downward arm could make
        # a second set of overlapping windows behind the selected seed.
        points, windows = self._track_direction(
            mask, seed_x, seed_y, top, bottom, -1
        )
        points_array = (
            np.asarray(points, dtype=np.int32)
            if points else np.empty((0, 2), dtype=np.int32)
        )
        return points_array, windows

    def _draw_tracking_windows(
        self,
        overlay: np.ndarray,
        left_windows: list[tuple[int, int, int, int, bool]],
        right_windows: list[tuple[int, int, int, int, bool]],
    ) -> None:
        """Draw true rotated polygons in the detector's only preview."""
        del left_windows, right_windows
        if not self.show_diagnostic_windows:
            return
        for polygon, found in self._rotated_polygons:
            color = (0, 165, 255) if found else (80, 80, 80)
            cv2.polylines(
                overlay,
                [np.round(polygon).astype(np.int32)],
                True,
                color,
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            overlay,
            "rotated windows: orange=lane / gray=miss",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _on_image(self, message: Image) -> None:
        self._rotated_polygons = []
        super()._on_image(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RotatedWindowFrontLaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
