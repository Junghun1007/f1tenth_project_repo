import math
import unittest

import numpy as np

from auto_control.local_path_planner import (
    LocalPathPlannerConfig,
    SpeedProfileConfig,
    build_ordered_path_model,
    plan_local_racing_path,
    spatial_speed_plan,
)


def left_right_angle_path() -> np.ndarray:
    approach = np.column_stack((np.linspace(0.05, 0.55, 26), np.zeros(26)))
    angle = np.linspace(-0.5 * math.pi, 0.0, 50)
    corner = np.column_stack(
        (0.55 + 0.50 * np.cos(angle), 0.50 + 0.50 * np.sin(angle))
    )
    exit_path = np.column_stack(
        (np.full(26, 1.05), np.linspace(0.50, 1.00, 26))
    )
    return np.vstack((approach, corner[1:], exit_path[1:]))


def build(points: np.ndarray):
    return build_ordered_path_model(
        points,
        minimum_points=8,
        minimum_span_m=0.12,
        minimum_x_m=0.01,
        maximum_x_m=3.0,
        local_smoothing_window_m=0.04,
        outlier_threshold_m=0.08,
        geometry_window_m=0.08,
        resample_interval_m=0.02,
    )


class LocalPathPlannerTest(unittest.TestCase):
    def test_ordered_path_preserves_vertical_corner_exit(self) -> None:
        path = build(left_right_angle_path())
        self.assertIsNotNone(path)
        self.assertGreater(path.total_length_m, 1.70)
        vertical_exit = path.points_m[path.points_m[:, 0] > 1.03]
        self.assertGreater(float(np.ptp(vertical_exit[:, 1])), 0.45)

    def test_left_corner_has_outside_inside_outside_offsets(self) -> None:
        path = build(left_right_angle_path())
        self.assertIsNotNone(path)
        plan = plan_local_racing_path(
            path,
            None,
            None,
            LocalPathPlannerConfig(
                corner_curvature_threshold_per_m=0.40,
                corner_minimum_heading_change_deg=15.0,
                maximum_path_curvature_per_m=3.0,
            ),
        )
        self.assertEqual(plan.detected_corner_count, 1)
        self.assertLess(float(np.min(plan.lateral_offset_m)), 0.0)
        self.assertGreater(float(np.max(plan.lateral_offset_m)), 0.0)
        maximum_safe_offset = 0.5 * 0.65 - (0.5 * 0.30 + 0.08)
        self.assertLessEqual(
            float(np.max(np.abs(plan.lateral_offset_m))),
            maximum_safe_offset + 1.0e-6,
        )

    def test_independent_boundaries_recenter_a_biased_centerline(self) -> None:
        path = build(left_right_angle_path())
        self.assertIsNotNone(path)
        tangent = path.tangent_at_s(path.arc_length_m)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        left = path.points_m + 0.25 * normal
        right = path.points_m - 0.42 * normal
        plan = plan_local_racing_path(
            path,
            left,
            right,
            LocalPathPlannerConfig(
                minimum_half_width_m=0.20,
                maximum_half_width_m=0.50,
                corner_curvature_threshold_per_m=0.40,
                corner_minimum_heading_change_deg=15.0,
                maximum_path_curvature_per_m=3.0,
            ),
        )
        self.assertLess(float(np.median(plan.center_correction_m)), -0.05)
        self.assertAlmostEqual(
            float(np.median(plan.left_half_width_m)),
            float(np.median(plan.right_half_width_m)),
            delta=0.02,
        )

    def test_narrow_measured_side_is_not_artificially_widened(self) -> None:
        path = build(left_right_angle_path())
        self.assertIsNotNone(path)
        tangent = path.tangent_at_s(path.arc_length_m)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        left = path.points_m + 0.18 * normal
        plan = plan_local_racing_path(
            path,
            left,
            None,
            LocalPathPlannerConfig(
                minimum_half_width_m=0.24,
                corner_curvature_threshold_per_m=0.40,
                corner_minimum_heading_change_deg=15.0,
                maximum_path_curvature_per_m=3.0,
            ),
        )
        self.assertLess(float(np.median(plan.left_half_width_m)), 0.20)
        self.assertLessEqual(float(np.max(plan.lateral_offset_m)), 1.0e-6)

    def test_right_corner_mirrors_outside_inside_outside_signs(self) -> None:
        points = left_right_angle_path().copy()
        points[:, 1] *= -1.0
        path = build(points)
        self.assertIsNotNone(path)
        plan = plan_local_racing_path(
            path,
            None,
            None,
            LocalPathPlannerConfig(
                corner_curvature_threshold_per_m=0.40,
                corner_minimum_heading_change_deg=15.0,
                maximum_path_curvature_per_m=3.0,
            ),
        )
        self.assertEqual(plan.detected_corner_count, 1)
        self.assertGreater(float(np.max(plan.lateral_offset_m)), 0.0)
        self.assertLess(float(np.min(plan.lateral_offset_m)), 0.0)
        self.assertLess(
            int(np.argmax(plan.lateral_offset_m)),
            int(np.argmin(plan.lateral_offset_m)),
        )

    def test_backward_speed_pass_slows_before_visible_corner(self) -> None:
        corner_path = build(left_right_angle_path())
        straight_path = build(
            np.column_stack((np.linspace(0.05, 1.8, 100), np.zeros(100)))
        )
        self.assertIsNotNone(corner_path)
        self.assertIsNotNone(straight_path)
        config = SpeedProfileConfig(
            minimum_speed_mps=0.8,
            maximum_speed_mps=2.4,
            maximum_lateral_acceleration_mps2=0.9,
            maximum_longitudinal_deceleration_mps2=1.8,
        )
        corner_speed = spatial_speed_plan(
            corner_path, current_speed_mps=2.4, config=config
        ).target_speed_mps
        straight_speed = spatial_speed_plan(
            straight_path, current_speed_mps=2.4, config=config
        ).target_speed_mps
        self.assertLess(corner_speed, straight_speed)
        self.assertAlmostEqual(straight_speed, 2.4, places=3)


if __name__ == "__main__":
    unittest.main()
