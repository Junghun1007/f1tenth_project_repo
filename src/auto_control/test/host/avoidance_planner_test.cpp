#include "check.hpp"
#include "auto_control/drive/core/avoidance_planner.hpp"

using namespace auto_control::drive;

static LaneObservation straight_lane() {
  LaneObservation lane;
  for (double x = 0.0; x <= 3.0 + 1e-9; x += 0.05) { lane.path.push_back({x, 0.0}); lane.center.push_back({x, 0.0}); }
  lane.valid = true;
  return lane;
}

TEST(planner_center_mode_without_cones) {
  AvoidancePlanner pl{PlannerParams{}, ControlParams{}};
  const auto r = pl.plan(straight_lane(), {}, {0, 0, 0}, 1.0, 0.0);
  CHECK(r.mode == PlanMode::Center);
  CHECK(!r.obstacle_active);
  CHECK(r.path_local.size() >= 50);
  CHECK_NEAR(r.path_local.back().y, 0.0, 1e-9);
}

TEST(planner_avoids_cone_inside_lane) {
  AvoidancePlanner pl{PlannerParams{}, ControlParams{}};
  // 차량 좌표 = odom (자세 0): 왼쪽에서 21 cm 들어온 콘
  const auto r = pl.plan(straight_lane(), {{1.5, 0.21}}, {0, 0, 0}, 1.0, 0.0);
  CHECK(r.mode == PlanMode::Lane || r.mode == PlanMode::LaneRelaxed);
  CHECK(r.obstacle_active);
  double y_at = 0, best = 1e9;
  for (const auto & p : r.path_local) if (std::abs(p.x - 1.5) < best) { best = std::abs(p.x - 1.5); y_at = p.y; }
  CHECK(y_at < -0.02);
}

TEST(planner_uses_pose_for_world_cones) {
  AvoidancePlanner pl{PlannerParams{}, ControlParams{}};
  // 차가 odom (10, 5)에서 +y를 향함 → 차량 앞 1.5 m, 왼쪽 0.21 m 콘의 월드 좌표는 (10-0.21, 6.5)
  const auto r = pl.plan(straight_lane(), {{10.0 - 0.21, 6.5}}, {10.0, 5.0, M_PI / 2}, 1.0, 0.0);
  CHECK(r.obstacle_active);
  CHECK(r.mode != PlanMode::Center);
}

#include "auto_control/drive/core/path_lock.hpp"

static double y_near(const std::vector<Vec2> & path, double x) {
  double y = 0, best = 1e9;
  for (const auto & p : path) if (std::abs(p.x - x) < best) { best = std::abs(p.x - x); y = p.y; }
  return y;
}

TEST(pinned_path_is_kept_even_without_visible_cones) {   // 모드 2: 화면 밖 콘 옆에서는 중앙선으로 돌아가지 않는다
  AvoidancePlanner pl{PlannerParams{}, ControlParams{}};
  LanePin pin;
  for (double s = 0.0; s <= 1.2; s += 0.1) { pin.s.push_back(s); pin.l.push_back(-0.12); }
  pin.end_s = 0.9;
  const auto r = pl.plan(straight_lane(), {}, {0, 0, 0}, 1.0, 0.0, &pin);
  CHECK(r.mode != PlanMode::Center);
  CHECK(r.pinned);
  CHECK(r.obstacle_active);
  CHECK_NEAR(y_near(r.path_local, 0.6), -0.12, 0.035);
}

TEST(pin_gives_way_when_infeasible) {   // 고정 경로 위에 새 콘이 보이면 고정 없이 다시 계획
  AvoidancePlanner pl{PlannerParams{}, ControlParams{}};
  LanePin pin;
  for (double s = 0.0; s <= 1.5; s += 0.1) { pin.s.push_back(s); pin.l.push_back(0.0); }
  pin.end_s = 1.4;
  const auto r = pl.plan(straight_lane(), {{0.9, 0.12}}, {0, 0, 0}, 1.0, 0.0, &pin);   // 고정 경로(l=0) 위를 막는 콘
  CHECK(!r.pinned);
  CHECK(y_near(r.path_local, 0.9) < -0.05);
}

TEST(free_mode_cannot_leave_lane_far) {   // 차선 밖 회피도 free_max_excess까지만
  PlannerParams p;
  p.free_max_excess = 0.05;
  // 차선 한가운데를 막는 콘 두 개: 차선 안에는 틈이 없음
  const auto ls = frenet_dp(p, 0.0, {-0.33, 0.0}, {{1.5, 0.08}, {1.5, -0.08}, {1.5, 0.21}, {1.5, -0.21}}, 3.0, 0.0, false, false, DesiredOffset{});
  if (ls) {
    for (std::size_t i = 0; i < ls->s.size(); ++i) CHECK(std::abs(ls->l[i]) + p.width / 2 <= p.lane_half + p.free_max_excess + 0.06);
  }
}

TEST(path_lock_refresh_and_release) {
  PathLock lock;
  std::vector<Vec2> plan;
  for (double s = 0.0; s <= 3.0; s += 0.1) plan.push_back({s, s < 1.0 ? -0.1 : 0.0});
  lock.set_last_plan(plan);
  CHECK(!lock.active());
  lock.refresh(0.9);
  CHECK(lock.active());
  CHECK_NEAR(lock.remaining(), 0.9, 1e-9);
  lock.refresh(0.92);   // 같은 콘(조금 다른 끝)은 다시 고정하지 않음
  CHECK_NEAR(lock.remaining(), 0.9, 1e-9);
  lock.advance(0.5);
  CHECK_NEAR(lock.remaining(), 0.4, 1e-9);
  CHECK_NEAR(lock.pin()->l.front(), -0.1, 1e-9);
  CHECK_NEAR(lock.pin()->end_s, 0.4, 1e-9);
  lock.advance(0.5);
  CHECK(!lock.active());
  CHECK(lock.pin() == nullptr);
}

TEST(cones_outside_the_road_do_not_trigger_avoidance) {   // 도로 밖(차선선에서 30 cm 바깥) 콘 → 중앙선 그대로
  AvoidancePlanner pl{PlannerParams{}, ControlParams{}};
  const auto r = pl.plan(straight_lane(), {{0.8, 0.65}, {1.2, -0.70}}, {0, 0, 0}, 1.0, 0.0);
  CHECK(r.mode == PlanMode::Center);
  CHECK(!r.obstacle_active);
  // 차선선 바로 옆(0.40 m) 콘도 차체와 18 cm 떨어져 권장 여유 밖이면 계획을 켜지 않음 (아래 테스트에서 띠 안·밖을 확인)
}

TEST(avoidance_turns_on_only_for_cones_in_the_driving_corridor) {   // 차가 지나갈 폭(차체 반폭 + 권장 여유 + 콘 반폭 + 여유) 밖 콘은 계획을 켜지 않는다
  PlannerParams p;   // 반폭 0.15 + 권장 0.06 + 콘 0.065 + 여유 0.05 = 0.325 m
  AvoidancePlanner pl{p, ControlParams{}};
  CHECK(pl.plan(straight_lane(), {{1.2, 0.40}}, {0, 0, 0}, 1.0, 0.0).mode == PlanMode::Center);   // 차선 안이지만 띠 밖
  CHECK(pl.plan(straight_lane(), {{1.2, 0.25}}, {0, 0, 0}, 1.0, 0.0).mode != PlanMode::Center);   // 띠 안
}

TEST(cones_beyond_reference_end_do_not_block_the_lane) {   // 짧게 잘린 중앙선(0.5 m) 끝 너머 콘은 회피 계획에서 빠진다
  LaneObservation lane;
  for (double x = -0.1; x <= 0.5 + 1e-9; x += 0.05) { lane.center.push_back({x, 0}); lane.path.push_back({x, 0}); }
  lane.valid = true;
  AvoidancePlanner pl{PlannerParams{}, ControlParams{}};
  // 끝 방향 직선 위 0.75 m: 예전에는 차선 한가운데 콘으로 계산돼 회피가 켜졌다
  CHECK(pl.plan(lane, {{0.75, 0.0}}, {0, 0, 0}, 1.0, 0.0).mode == PlanMode::Center);
  PlannerParams off;
  off.beyond_end_margin = 100;
  AvoidancePlanner old{off, ControlParams{}};
  CHECK(old.plan(lane, {{0.75, 0.0}}, {0, 0, 0}, 1.0, 0.0).mode != PlanMode::Center);
}
