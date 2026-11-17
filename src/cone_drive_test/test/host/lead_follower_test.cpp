#include "check.hpp"
#include "cone_drive_test/core/lead_follower.hpp"

using namespace cone_drive_test;

static std::vector<Vec2> straight() { std::vector<Vec2> p; for (double x = 0; x <= 3.0001; x += 0.05) p.push_back({x, 0}); return p; }
static ConeObservation car(double near_x, double y) {   // 가까운 면이 near_x인 세단 (50×30 cm)
  ConeObservation c; c.class_id = 2; c.length = 0.50; c.width = 0.30; c.center = {near_x + 0.25, y}; return c;
}

TEST(gap_is_along_path_from_front_bumper_and_off_path_is_ignored) {
  LeadParams p;
  CHECK_NEAR(lead_gap_along_path(straight(), car(1.0, 0.0), p), 0.95, 0.03);    // 앞범퍼 = 앞차축 + 5 cm
  CHECK(lead_gap_along_path(straight(), car(1.0, 0.0 + 0.15 + 0.05 + 0.15 + 0.02), p) < 0);   // 옆으로 비켜 있음
  CHECK(lead_gap_along_path(straight(), car(1.0, 0.30), p) >= 0);               // 경로에 살짝 걸침 → 앞차 (Q1 b)
}

TEST(stops_at_stop_gap_and_restarts_after_resume_gap) {
  LeadFollower f;
  LeadParams p;
  f.set_params(p);
  double t = 0, d = 0;
  // 서 있는 앞차에 다가감
  auto s = f.update(straight(), {car(1.0, 0.0)}, true, t, d, 1.0);
  CHECK(s.active);
  CHECK(s.cap > 0.5 && s.cap < 1.5);                     // sqrt(2·0.8·(0.95-0.25)) ≈ 1.06
  d = 0.70; t = 1.0;                                      // 0.70 m 전진 → 거리 0.25
  s = f.update(straight(), {car(0.30, 0.0)}, true, t, d, 0.0);
  CHECK(s.holding);
  CHECK_NEAR(s.cap, 0.0, 1e-9);
  // 앞차가 천천히(0.2 m/s < 0.3) 움직여 0.35 m → 아직 정지 유지
  t = 1.5; s = f.update(straight(), {car(0.40, 0.0)}, true, t, d, 0.0);
  CHECK(s.holding);
  // 0.45 m로 벌어지면 재출발
  t = 2.0; s = f.update(straight(), {car(0.50, 0.0)}, true, t, d, 0.0);
  CHECK(!s.holding);
  CHECK(s.cap > 0.3);
}

TEST(moving_lead_keeps_speed_dependent_gap) {
  LeadFollower f;
  f.set_params(LeadParams{});
  double t = 0, d = 0;
  // 앞차·내 차 모두 1.0 m/s, 거리 1.0 m 유지 → 앞차 속도 ≈ 1.0
  for (int k = 0; k < 40; ++k) { f.update(straight(), {car(1.05, 0.0)}, true, t, d, 1.0); t += 0.025; d += 0.025; }
  const auto s = f.state();
  CHECK_NEAR(s.lead_speed, 1.0, 0.05);
  CHECK(s.cap > 1.0);                                    // 목표 거리 0.40+0.3·1.0=0.70 < 1.0 → 앞차보다 조금 빨라도 됨
  // 거리 0.5 (< 0.70) → 앞차보다 느리게
  LeadFollower g; g.set_params(LeadParams{}); t = 0; d = 0;
  for (int k = 0; k < 40; ++k) { g.update(straight(), {car(0.55, 0.0)}, true, t, d, 1.0); t += 0.025; d += 0.025; }
  CHECK(g.state().cap < 1.0);
}

TEST(lost_close_lead_is_held_until_seen_off_path) {
  LeadFollower f;
  f.set_params(LeadParams{});
  f.update(straight(), {car(0.30, 0.0)}, true, 0.0, 0.0, 0.0);   // 정지 거리(0.25)에 서 있음
  auto s = f.update(straight(), {}, true, 2.0, 0.0, 0.0);   // 가까이서 2초간 안 보임 → 유지
  CHECK(s.active);
  CHECK(!s.seen);
  CHECK_NEAR(s.cap, 0.0, 1e-9);
  s = f.update(straight(), {car(0.35, 0.6)}, true, 2.1, 0.0, 0.0);   // 경로 밖에서 보임 → 해제
  CHECK(!s.active);
}

TEST(disabled_follower_never_caps) {
  LeadFollower f;
  LeadParams p; p.enabled = false;
  f.set_params(p);
  CHECK(!f.update(straight(), {car(0.3, 0.0)}, true, 0, 0, 1.0).active);
}
