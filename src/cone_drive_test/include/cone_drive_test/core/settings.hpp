// 실시간 조절 파라미터 정의(웹 파라미터 창 목록)와 각 모듈 설정으로의 변환
#pragma once
#include <array>
#include "cone_drive_test/core/avoidance_planner.hpp"
#include "cone_drive_test/core/box_depth.hpp"
#include "cone_drive_test/core/cone_memory.hpp"
#include "cone_drive_test/core/lane_memory.hpp"
#include "cone_drive_test/core/lane_post.hpp"
#include "cone_drive_test/core/longitudinal.hpp"
#include "cone_drive_test/core/params.hpp"
#include "cone_drive_test/core/steering.hpp"
#include "cone_drive_test/core/visibility.hpp"

namespace cone_drive_test {

struct Settings {
  // 장애물
  std::array<float, 4> yolo_thresholds{0.7f, 0.7f, 0.7f, 0.7f};   // con, truck, sedan, traffic_light
  double nms_threshold{0.65}, roi_height_percent{65}, yolo_max_fps{40};
  double cone_size_m{0.13};
  BoxDepthOptions box;
  MemoryParams memory;
  int memory_mode{0};             // 0 = 월드 좌표 기억(오도메트리 x·y·방향), 1 = 차선 기준 단기 기억(이동 거리만)
  LaneMemoryParams lane_memory;
  PlannerParams planner;
  // 주행
  ControlParams control;           // 롤아웃용 Stanley·조향 필터 + 장애물/실패 속도
  LongitudinalParams lon;
  double maximum_lateral_accel{1.6}, planning_decel{0.5};
  double stanley_gain{1.3}, stanley_softening{0.3}, heading_lookahead_m{0.05};
  double steering_current_weight{0.4}, steering_rate_rad_s{240.0 * 3.14159265358979323846 / 180.0};
  double corner_outward_offset_m{0.08}, corner_entry_distance_m{2.2}, corner_entry_full_offset_m{0.6};
  double link_loss_stop_sec{1.0};
};

void register_params(ParamRegistry & registry);       // 기본값 = 시뮬레이터 config.py / auto_control.yaml / multiclass_yolo
Settings settings_from(const ParamRegistry & registry);

}  // namespace cone_drive_test
