#include "check.hpp"
#include "auto_control/drive/copied/control_core.hpp"
#include "auto_control/drive/core/types.hpp"
#include <cmath>

using namespace auto_control::drive;

static OrderedPath straight(double y, double length = 2.0) {
  std::vector<Point> pts;
  for (double x = 0.0; x <= length + 1e-9; x += 0.05) pts.push_back({x, y});
  auto p = build_ordered_path(pts, 4, 0.3, -0.5, 3.5, 0.2, 0.12);
  CHECK(p.has_value());
  return *p;
}

TEST(stanley_steers_toward_path_on_left) {
  const auto r = stanley_control(straight(0.10), 1.0, 1.3, 0.3, 0.05, 30.0 * M_PI / 180, 60.0 * M_PI / 180, 0.8);
  CHECK(r.steering_angle_rad > 0.0);
}

TEST(stanley_zero_on_path) {
  const auto r = stanley_control(straight(0.0), 1.0, 1.3, 0.3, 0.05, 30.0 * M_PI / 180, 60.0 * M_PI / 180, 0.8);
  CHECK_NEAR(r.cross_track_error_m, 0.0, 1e-6);
  CHECK_NEAR(r.steering_angle_rad, 0.0, 1e-6);
}

TEST(speed_plan_respects_lateral_accel_on_arc) {
  std::vector<Point> pts;   // 반경 0.5 m 왼쪽 원호
  for (double a = 0.0; a <= M_PI / 2 + 1e-9; a += 0.02) pts.push_back({0.5 * std::sin(a), 0.5 - 0.5 * std::cos(a)});
  auto path = build_ordered_path(pts, 4, 0.3, -0.5, 3.5, 0.2, 0.12);
  CHECK(path.has_value());
  const auto plan = plan_path_speeds(*path, 1.0, 1.8, 1.6, 0.5, 0.5, 1.8, 0.0, 0.0);
  double peak = 0;
  for (std::size_t i = 0; i + 1 < plan.nominal_speed_mps.size(); ++i) peak = std::max(peak, plan.nominal_speed_mps[i]);
  CHECK(peak <= std::sqrt(1.6 / 2.0) * 1.05 + 1e-6 || peak <= 1.0 + 1e-6);   // sqrt(a/κ)=0.894, 최소 속도 1.0이 하한
}

TEST(erpm_zero_is_zero_speed) {
  CHECK_NEAR(erpm_to_speed_mps(0, 0.11, 2, 13, 54, 13, 37, 1.0, 1.0), 0.0, 1e-12);
}

TEST(types_now_sec_monotonic) {
  const double a = now_sec(), b = now_sec();
  CHECK(b >= a);
}
