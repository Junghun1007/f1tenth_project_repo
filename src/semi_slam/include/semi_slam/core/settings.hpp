// semi_slam 실시간 파라미터 (웹 파라미터 창 탭: "slam" = 매핑·보정, "driving" = 주행)
#pragma once
#include "semi_slam/core/lane_post.hpp"
#include "semi_slam/core/longitudinal.hpp"
#include "semi_slam/core/params.hpp"
#include "semi_slam/core/slam.hpp"
#include "semi_slam/core/steering.hpp"

namespace semi_slam {

struct Settings {
  // 주행 계획
  double run_speed{1.0};          // 일정 속도 (코너 감속 없음)
  int run_laps{10};
  bool stabilization{true};       // IMU 영상 안정화 (전체 / 끄기)
  bool auto_calibrate{true};      // 목표 바퀴 완료 시 보정 계산·적용
  bool clockwise{false};          // 주행 방향 (출발할 때 적용)
  double camera_yaw_deg{0.0};     // 카메라 yaw (BEV 생성에 반영)
  double start_offset_m{0.0};     // 출발: 앞범퍼가 아래 직선 시작점에서 진행 방향으로 이만큼 앞 (+)
  double start_lateral_m{0.0};    // 출발: 도로 가운데에서 왼쪽(+)으로
  SlamConfig slam;
  // 조향·차선
  double stanley_gain{1.2}, stanley_softening{0.3}, heading_lookahead_m{0.12};
  double steering_current_weight{0.4}, steering_rate_rad_s{240.0 * 3.14159265358979323846 / 180.0};
  bool lane_side_tracking{true};
  double corner_outward_offset_m{0.08}, corner_entry_distance_m{2.2}, corner_entry_full_offset_m{0.6};
  // 종방향
  LongitudinalParams lon;
  // 안전
  double lane_timeout_sec{1.0};   // 0 = 제한 없음
  double link_loss_stop_sec{1.0};
};

void register_params(ParamRegistry & registry);
Settings settings_from(const ParamRegistry & registry);

}  // namespace semi_slam
