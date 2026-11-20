#include "check.hpp"
#include "cone_drive_test/core/vehicle_accel.hpp"
#include <cmath>

using namespace cone_drive_test;

TEST(steady_turn_gives_centripetal_and_lever_terms) {   // 1 m/s, 1.6 rad/s 일정 회전: ay = v·ω, ax = −ω²·r
  VehicleAccelModel m(0.24);
  for (int i = 0; i < 2000; ++i) {
    const double t = i * 0.0025;
    m.on_yaw_rate(1.6, t);
    if (i % 8 == 0) m.on_speed(1.0, t);
  }
  const auto a = m.accel(5.0);
  CHECK(a.has_value());
  CHECK_NEAR(a->y, 1.6, 0.02);
  CHECK_NEAR(a->x, -1.6 * 1.6 * 0.24, 0.02);
}

TEST(acceleration_ramp_and_stale_speed) {   // 직선 가속 0.5 m/s²: ax ≈ 0.5, 속도가 끊기면 보정하지 않음
  VehicleAccelModel m(0.24);
  for (int i = 0; i < 400; ++i) {
    const double t = i * 0.0025;
    m.on_yaw_rate(0.0, t);
    if (i % 8 == 0) m.on_speed(0.5 * t, t);
  }
  const auto a = m.accel(1.0);
  CHECK(a.has_value());
  CHECK_NEAR(a->x, 0.5, 0.05);
  CHECK_NEAR(a->y, 0.0, 1e-6);
  CHECK(!m.accel(1.5).has_value());
}
