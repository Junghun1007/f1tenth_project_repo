#include "check.hpp"
#include "semi_slam/core/duty_only.hpp"
#include "semi_slam/core/longitudinal.hpp"

using namespace semi_slam;

static LongitudinalParams duty_mode() { LongitudinalParams p; p.mode = 2; return p; }

TEST(duty_only_starts_and_tracks_without_brake) {
  LongitudinalController lc(duty_mode());
  double t = 0;
  lc.on_speed(0.0, t);
  auto c = lc.update(0.01, 1.0, false, 1e9, false);
  CHECK(c.phase == "duty_only_start");
  CHECK_NEAR(c.duty, 0.053, 1e-9);
  for (int k = 0; k < 200; ++k) { t += 0.01; lc.on_speed(0.8, t); c = lc.update(0.01, 1.0, false, 1e9, false); }
  CHECK(!c.brake_mode);
  CHECK(c.duty > 0.053);
}

TEST(duty_only_zero_target_coasts_and_overspeed_brakes) {
  LongitudinalController slow(duty_mode());
  slow.on_speed(0.3, 0.0);
  auto c = slow.update(0.01, 0.0, false, 1e9, true);   // 느릴 때 정지 요청 → 모터 풀고 굴러서 정지
  CHECK(!c.brake_mode);
  CHECK_NEAR(c.duty, 0.0, 1e-12);
  CHECK(c.phase == "duty_only_coast");
  LongitudinalController fast(duty_mode());
  fast.on_speed(1.2, 0.0);
  c = fast.update(0.01, 0.0, false, 1e9, true);        // 목표보다 0.5 m/s 이상 빠름 → 비상 제동
  CHECK(c.brake_mode);
  CHECK(c.phase == "duty_only_emergency_brake");
}

TEST(mode_switch_waits_for_standstill) {
  LongitudinalController lc(LongitudinalParams{});      // staged로 시작
  lc.on_speed(1.0, 0.0);
  lc.set_params(duty_mode());
  lc.update(0.01, 1.0, false, 1e9, false);
  CHECK(lc.active_mode() == 1);                         // 움직이는 중엔 그대로
  lc.on_speed(0.0, 1.0);
  lc.update(0.01, 0.0, false, 1e9, false);
  CHECK(lc.active_mode() == 2);
}

TEST(stop_speed_limit_releases_to_coast_near_aim) {
  CHECK_NEAR(duty_only_stop_speed_limit(0.0, 0.4, 0.5, 0.35, 0.1), 0.0, 1e-12);
  CHECK(duty_only_stop_speed_limit(1.0, 0.4, 0.5, 0.35, 0.1) > 0.35);
  CHECK(duty_only_coast_release(0.12, 0.35, 0.5, 0.03, 0.40));    // 0.35²/(2·0.5)+0.01 ≈ 0.133 ≥ 0.12
  CHECK(!duty_only_coast_release(0.12, 0.8, 0.5, 0.03, 0.40));    // release 속도보다 빠르면 풀지 않음
}

