// 코너 경로 계획 (실험): 속도 반영 내륜차 통로 + 최소 곡률 스플라인.
// tools/lane_path_lab/planner.js와 같은 알고리즘. 좌표는 앞차축 기준 차량 좌표 (x 앞, y 왼쪽).
#pragma once
#include "auto_control/drive/core/types.hpp"
#include <vector>

namespace auto_control::drive {

struct CornerPlanParams {
  bool enabled{false};
  double lane_width{0.65};          // 한쪽 차선이 안 보일 때 반대쪽 추정에 씀
  double vehicle_width{0.30};
  double wheelbase{0.33};
  double front_overhang{0.05};      // 앞차축 → 앞 범퍼
  double lateral_accel{1.4};        // 곡률 속도 상한 (drive.maximum_lateral_accel)
  double deceleration{0.45};        // 감속 계획 (drive.planning_decel)
  double acceleration{0.8};         // 가속 계획 (lon.planning_acceleration_mps2)
  double v0{1.3};                   // 내륜차가 0이 되는 속도 (슬립). 실측으로 맞출 것
  double inner_margin{0.03};        // 회전 안쪽 차선과 차체 사이 여유
  double outer_margin{0.03};
};

struct CornerPlanResult {
  bool ok{false};
  std::vector<Vec2> path;           // 0.05 m 간격, 첫 점 = 차량 위치 근처
  double min_radius{0};             // ±0.24 m 세 점 반경
  double min_clearance{0};          // 관측한 차선과 차체 네 모서리의 최소 여유 (관측 없으면 +inf)
  double apex_speed{0};
  double estimated_fraction{0};     // 통로 중 한쪽 이상을 차선 폭으로 추정한 비율
  bool infeasible{false};           // 통로가 차폭+여유보다 좁은 구간 있음
  bool decel_short{false};          // 현재 속도에서 코너 전에 충분히 못 줄임
};

// reference: 코너 바깥 이동을 끈 차선 중심선. left/right: 관측한 차선 점 (순서 무관).
CornerPlanResult plan_corner_path(const std::vector<Vec2> & reference, const std::vector<Vec2> & left,
                                  const std::vector<Vec2> & right, double speed, const CornerPlanParams & p);

}  // namespace auto_control::drive
