// Frenet DP 경로 탐색과 롤아웃 충돌 검사 (tools/avoidance_sim planners.py _dp / _rollout_ok 이식).
// 좌표: s = 기준선(차선 중심) 호 길이, l = 왼쪽 횡오프셋. 롤아웃은 앞차축 기준 차량 좌표.
#pragma once
#include <optional>
#include <vector>
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

struct PlannerParams {
  double s_max{3.0};
  double ds{0.15}, l_max{1.0}, dl{0.04};                 // 차선 밖 허용 모드 격자
  double ds_lane{0.10}, l_max_lane{0.36}, dl_lane{0.02}; // 차선 안 모드 격자
  double lane_half{0.325};
  double hard_clearance{0.015}, soft_clearance{0.06};    // 차선 안 (지그재그 탐색 결과)
  double relaxed_hard_clearance{0.0}, relaxed_soft_clearance{0.04};
  double free_hard_clearance{0.005}, free_soft_clearance{0.10};
  double w_obstacle{4000}, w_lane{400}, w_lane_linear{40}, lane_quad_cap{0.06}, w_lane_linear_free{4};
  double lane_tolerance{0.03}, lane_return_distance{0.8};
  double free_max_excess{1.0};
  double activation_margin{0.05};   // 회피 계획을 켜는 띠 여유 (차체 반폭 + 권장 여유 + 콘 반폭에 더함)    // 차선 밖 회피에서 차체가 차선 밖으로 나갈 최대 거리 (m). 1.0 = 시뮬레이터와 같음(사실상 제한 없음), 실차 기본은 settings 0.10
  double w_offset{20}, w_slope{1}, w_curvature{0.08}, max_curvature{3.5};
  double rollout_sec{1.5}, rollout_retry_margin{0.03}, backward_extension{0.8};
  // 기준선(중앙선) 끝보다 beyond_end_margin 넘게 앞인 콘은 차선 좌표를 믿을 수 없으므로 회피 계획(비용·단계 판정)에서 뺀다.
  // 롤아웃 충돌 검사에는 그대로 넣는다. (코너에서 BEV 옆 끝에 잘린 중앙선을 끝 방향 직선으로 늘려 계산하면
  // 계속 꺾이는 길의 바깥 콘이 차선 한가운데로 나와 차선 밖으로 밀려났다. 녹화 재생: 코너 끝 fallback 41→31, 롤아웃 실패 68→44)
  // forward_extension: 중앙선이 계획 거리보다 짧으면 끝 곡률을 0까지 줄여 가며 원호로 늘린다 (0 = 끔, 기본 끔).
  // 녹화 재생에서는 늘린 길로 콘을 계산하면 오히려 막힘이 늘었다 (코너 끝 fallback 41→69~75) → 실차 시험용
  double forward_extension{0.0}, beyond_end_margin{0.1};
  double cone_half{0.065};
  // 차량
  double wheelbase{0.33}, length{0.50}, width{0.30}, rear_overhang{0.12};
  double max_steer_rad{30.0 * 3.14159265358979323846 / 180.0};
  double steer_rate_rad_s{240.0 * 3.14159265358979323846 / 180.0};
};

struct ControlParams {   // 롤아웃이 흉내 내는 실제 제어기 설정
  double stanley_gain{1.3}, softening_speed{0.3}, heading_lookahead{0.05}, geometry_window{0.16};
  double corner_heading_threshold_rad{60.0 * 3.14159265358979323846 / 180.0}, corner_opposing_ratio{0.8};
  double steering_current_weight{0.4};
  double max_speed{1.8};
  double latency{0.05};           // 인지→제어 지연 (롤아웃 시작 지연)
  double obstacle_speed{1.0};     // 롤아웃 속도 하한 (곧 도달할 장애물 구간 속도)
  double rollout_fail_speed{0.5}; // 충돌 예상 시 속도 상한
};

struct DesiredOffset { std::vector<double> s, l; };   // s 오름차순. 비어 있으면 0
struct FrenetPath { std::vector<double> s, l; };
// 고정 구간: s ≤ end_s에서는 l이 이 경로(±tol)를 따라야 한다 (화면을 벗어난 콘 옆을 지나는 동안 재계획 금지)
struct LanePin { std::vector<double> s, l; double end_s{0}; double tol{0.03}; };

std::optional<FrenetPath> frenet_dp(const PlannerParams & p, double l0, Vec2 rear_sl, const std::vector<Vec2> & cones_sl,
                                    double s_end, double margin_extra, bool lane_hard, bool relaxed,
                                    const DesiredOffset & l_des, const LanePin * pin = nullptr);

struct RolloutResult { bool ok{true}; double min_gap{1e9}; };
RolloutResult rollout_ok(const PlannerParams & p, const ControlParams & c, const std::vector<Vec2> & path_local,
                         const std::vector<Vec2> & cones_local, double speed, double steer);

// 회전 사각형(차체)과 축정렬 정사각형들의 SAT 최소 간격 (음수=겹침)
double rect_squares_min_gap(double cx, double cy, double yaw, double half_len, double half_wid,
                            const std::vector<Vec2> & centers, double half_sq);

}  // namespace cone_drive_test
