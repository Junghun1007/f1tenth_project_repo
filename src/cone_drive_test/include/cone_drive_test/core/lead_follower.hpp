// 차량 추종 (크루즈 컨트롤, acc.enabled). 회피 경로는 콘만으로 짜고, 그 경로를 따라 지나갈 폭
// (차 폭/2 + lateral_margin) 안에 차량이 조금이라도 걸치면 앞차로 보고 속도 상한을 건다.
//  - 거리(gap): 계획 경로를 따라 잰 앞범퍼 → 앞차 가장 가까운 면 거리
//  - 앞차 속도: 측정 간 거리 변화 + 내 이동 거리(바퀴), 시정수 speed_tau로 평활
//  - 앞차가 min_follow_speed보다 느리면 서 있는 것으로 보고 stop_gap에서 정지, 거리가 resume_gap 이상이 되면 다시 출발
//  - 움직이면 목표 거리 = resume_gap + time_gap × 내 속도 (빠를수록 멀리)
//  - hold_near 안에서 놓치면(카메라 아래 사각) 다시 보일 때까지 그 자리에 서 있다고 가정
#pragma once
#include <limits>
#include <vector>
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

struct LeadParams {
  bool enabled{true};
  double stop_gap{0.25}, resume_gap{0.40}, time_gap{0.3};
  double decel{0.8};
  double lateral_margin{0.05};
  double min_follow_speed{0.3};
  double hold_near{0.6}, lost_timeout{0.3};
  double speed_tau{0.3};
  double ego_half_width{0.15}, front_overhang{0.05};
};

struct LeadState {
  bool active{false};      // 앞차 있음
  bool seen{false};        // 이번에 실제로 보임 (false면 예측·유지 중)
  bool holding{false};     // 정지 유지 중 (resume_gap까지 벌어지길 기다림)
  double gap{0}, lead_speed{0};
  double cap{std::numeric_limits<double>::infinity()};   // 속도 상한
};

// 경로(앞차축 기준 차량 좌표)를 따라 지나갈 폭 안에 차량 사각형이 걸치면 앞범퍼 기준 거리, 아니면 음수
double lead_gap_along_path(const std::vector<Vec2> & path, const ConeObservation & vehicle, const LeadParams & p);

class LeadFollower {
public:
  void set_params(const LeadParams & p) { p_ = p; }
  void reset() { s_ = LeadState{}; have_meas_ = false; }
  // path: 이번 계획 경로, vehicles: 새 검출이 있으면 그 차량들(경로와 같은 좌표·시각으로 옮김), new_det: 새 깊이 검출 여부
  // t: 경로 시각, dist: 그 시각 바퀴 누적 거리, v_ego: 내 속도
  const LeadState & update(const std::vector<Vec2> & path, const std::vector<ConeObservation> & vehicles, bool new_det,
                           double t, double dist, double v_ego);
  const LeadState & state() const { return s_; }

private:
  double cap_for(double gap, double v_lead, double v_ego);
  LeadParams p_;
  LeadState s_;
  bool have_meas_{false};
  double meas_gap_{0}, meas_t_{0}, meas_dist_{0};
};

}  // namespace cone_drive_test
