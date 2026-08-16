import math
import unittest

import numpy as np

from auto_control.control_core import (
    PathModel,
    SpeedPid,
    centerline_points_from_mono8,
    curvature_target_speed,
    erpm_to_speed_mps,
    fit_path_model,
    representative_curvature,
    speed_feedforward_duty,
    stanley_control,
    steering_angle_to_servo,
)


class ControlCoreTest(unittest.TestCase):
    def test_centerline_image_uses_bev_vehicle_coordinates(self) -> None:
        image = np.zeros((300, 260), dtype=np.uint8)
        image[150:260, 129:131] = 255

        x_m, y_m = centerline_points_from_mono8(
            image,
            x_max_m=3.0,
            y_max_m=0.60,
            lateral_margin_m=0.70,
            meter_per_pixel=0.01,
            threshold=128,
        )

        self.assertAlmostEqual(float(x_m[0]), 0.405)
        self.assertAlmostEqual(float(x_m[-1]), 1.495)
        self.assertTrue(np.allclose(y_m, 0.0))

    def test_path_fit_and_stanley_steer_toward_left_path(self) -> None:
        x_m = np.linspace(0.3, 2.0, 80)
        y_m = 0.10 + 0.08 * np.square(x_m)
        path = fit_path_model(
            x_m,
            y_m,
            polynomial_order=2,
            minimum_points=20,
            minimum_span_m=0.35,
            minimum_x_m=0.25,
            maximum_x_m=2.2,
        )
        self.assertIsNotNone(path)

        result = stanley_control(
            path,
            speed_mps=2.0,
            gain=1.2,
            softening_speed_mps=0.5,
            heading_lookahead_m=0.25,
            maximum_steering_angle_rad=math.radians(30.0),
        )

        self.assertGreater(result.cross_track_error_m, 0.0)
        self.assertGreater(result.heading_error_rad, 0.0)
        self.assertGreater(result.steering_angle_rad, 0.0)
        servo = steering_angle_to_servo(
            result.steering_angle_rad,
            maximum_steering_angle_rad=math.radians(30.0),
            servo_left=0.98,
            servo_center=0.46,
            servo_right=0.02,
        )
        self.assertGreater(servo, 0.46)

    def test_curvature_profile_is_bounded_by_requested_speeds(self) -> None:
        path = PathModel(
            coefficients=np.asarray((0.0, 0.0, 0.25)),
            minimum_x_m=0.25,
            maximum_x_m=2.2,
            point_count=100,
        )
        curvature = representative_curvature(
            path,
            lookahead_minimum_x_m=0.5,
            lookahead_maximum_x_m=1.6,
            percentile=90.0,
        )
        target_speed = curvature_target_speed(
            curvature,
            maximum_lateral_acceleration_mps2=2.0,
            minimum_speed_mps=2.0,
            maximum_speed_mps=2.5,
        )
        self.assertGreaterEqual(target_speed, 2.0)
        self.assertLessEqual(target_speed, 2.5)
        self.assertLess(target_speed, 2.5)

    def test_erpm_conversion_matches_existing_vehicle_geometry(self) -> None:
        speed_mps = erpm_to_speed_mps(
            10000,
            wheel_diameter_m=0.1095,
            motor_pole_pairs=2,
            motor_pinion_teeth=13,
            spur_gear_teeth=54,
            differential_pinion_teeth=13,
            differential_ring_teeth=37,
            direction_sign=1.0,
            scale_correction=1.0,
        )
        self.assertAlmostEqual(speed_mps, 2.425, places=3)

    def test_speed_pid_never_exceeds_duty_limits(self) -> None:
        pid = SpeedPid(
            kp=0.012,
            ki=0.004,
            kd=0.0,
            integral_limit=1.0,
            minimum_duty=0.05,
            maximum_duty=0.07,
        )
        feedforward = speed_feedforward_duty(
            2.25,
            minimum_speed_mps=2.0,
            maximum_speed_mps=2.5,
            minimum_duty=0.05,
            maximum_duty=0.07,
        )
        self.assertAlmostEqual(feedforward, 0.06)
        self.assertEqual(
            pid.update(
                target_speed_mps=2.5,
                current_speed_mps=0.0,
                feedforward_duty=feedforward,
                dt_sec=0.0125,
            ),
            0.07,
        )
        self.assertEqual(
            pid.update(
                target_speed_mps=2.0,
                current_speed_mps=4.0,
                feedforward_duty=feedforward,
                dt_sec=0.0125,
            ),
            0.05,
        )


if __name__ == "__main__":
    unittest.main()
