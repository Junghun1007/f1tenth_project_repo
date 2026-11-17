#include "check.hpp"
#include "cone_drive_test/core/avoidance_planner.hpp"

using namespace cone_drive_test;

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
