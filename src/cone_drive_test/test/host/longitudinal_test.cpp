#include "check.hpp"
#include "cone_drive_test/core/longitudinal.hpp"
#include "cone_drive_test/core/steering.hpp"

using namespace cone_drive_test;

TEST(longitudinal_start_duty_once) {
  LongitudinalParams P;
  LongitudinalController lc(P);
  lc.on_speed(0.0, 0.0);
  const auto c1 = lc.update(0.01, 1.0, false, 1e9, false);
  CHECK_NEAR(c1.duty, P.longitudinal_start_duty, 1e-9);
  CHECK(c1.phase == "start_drive");
  lc.on_speed(0.0, 0.01);
  const auto c2 = lc.update(0.01, 1.0, false, 1e9, false);
  CHECK_NEAR(c2.duty, P.longitudinal_start_duty + P.duty_rise_rate_per_sec * 0.01, 1e-9);
}

TEST(longitudinal_brakes_when_overspeed_in_obstacle_zone) {
  LongitudinalParams P;
  LongitudinalController lc(P);
  double t = 0;
  lc.on_speed(1.6, t);
  LongitudinalCommand c;
  for (int k = 0; k < 100; ++k) { t += 0.01; lc.on_speed(1.6, t); c = lc.update(0.01, 1.8, true, 1e9, false); }
  CHECK(c.brake_current > 0.0);
  CHECK_NEAR(c.duty, 0.0, 1e-12);
}

TEST(longitudinal_stop_request_brakes) {
  LongitudinalParams P;
  LongitudinalController lc(P);
  lc.on_speed(0.5, 0.0);
  const auto c = lc.update(0.02, 1.0, false, 1e9, true);
  CHECK(c.brake_current > 0.0);
  CHECK_NEAR(c.duty, 0.0, 1e-12);
  CHECK_NEAR(c.target_speed, 0.0, 1e-12);
}

TEST(longitudinal_speed_cap_lowers_target_immediately) {
  LongitudinalParams P;
  LongitudinalController lc(P);
  lc.on_speed(1.0, 0.0);
  const auto c = lc.update(0.01, 1.8, false, 0.5, false);
  CHECK_NEAR(c.target_speed, 0.5, 1e-9);
}

TEST(steering_filter_and_rate_limit) {
  SteeringParams P;
  SteeringFilter f(P);
  const double a = f.update(0.3, 0.01, 0.0, 1.0);
  const double cw = 1.0 - std::pow(1.0 - P.steering_current_weight, 0.01 * P.control_rate_hz);
  CHECK_NEAR(a, std::min(cw * 0.3, P.rate_limit_rad_s * 0.01), 1e-9);
  CHECK(f.servo(0.0) == P.servo_center);
}

TEST(steering_holds_at_standstill) {
  SteeringParams P;
  SteeringFilter f(P);
  for (int k = 0; k < 50; ++k) f.update(0.2, 0.01, 0.0, 1.0);
  const double held = f.angle();
  const double a = f.update(-0.3, 0.01, 0.0, 0.0, 0.0);   // 목표 0, 정지 → 유지
  CHECK_NEAR(a, held, 1e-12);
}
