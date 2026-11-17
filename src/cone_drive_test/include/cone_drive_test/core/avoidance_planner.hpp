// 제안 회피 계획 (planners.py ProposedPlanner.update의 계획 부분 이식). 기억 갱신은 포함하지 않는다.
#pragma once
#include <limits>
#include <optional>
#include <vector>
#include "cone_drive_test/core/frenet_planner.hpp"

namespace cone_drive_test {

enum class PlanMode { Center, Lane, LaneRelaxed, Free, Fallback };
const char * plan_mode_name(PlanMode m);   // "center" | "lane" | "lane-tight" | "free" | "fallback"

struct PlanResult {
  std::vector<Vec2> path_local;    // 앞차축 기준 차량 좌표 (계획에 쓴 자세 기준)
  PlanMode mode{PlanMode::Center};
  bool rollout_ok{true};
  bool obstacle_active{false};     // 근처(s −0.6~3.0, |l|<1)에 기억 콘이 있음
  double speed_cap{std::numeric_limits<double>::infinity()};   // 롤아웃 실패 시 rollout_fail_speed
  double compute_ms{0};
  bool pinned{false};             // 고정 구간을 지킨 경로
};

class AvoidancePlanner {
public:
  AvoidancePlanner(const PlannerParams & p, const ControlParams & c) : p_(p), c_(c) {}
  void set_params(const PlannerParams & p, const ControlParams & c) { p_ = p; c_ = c; }
  // lane: 그 자세에서의 차선 관측, cones_world: 확정 기억 콘(odom), pose: lane 촬영 시점 odom 자세
  // pin: 고정 구간 (기억 방식 2). 고정을 지키는 경로가 없으면 고정 없이, 화면 밖 콘(blind_world)까지 넣고 다시 계획한다
  PlanResult plan(const LaneObservation & lane, const std::vector<Vec2> & cones_world, const Pose2 & pose,
                  double speed, double steer, const LanePin * pin = nullptr, const std::vector<Vec2> * blind_world = nullptr);
  void reset() { last_path_world_.clear(); }

private:
  PlannerParams p_;
  ControlParams c_;
  std::vector<Vec2> last_path_world_;
};

}  // namespace cone_drive_test
