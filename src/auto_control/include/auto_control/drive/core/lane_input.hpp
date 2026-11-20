// line_detactor LaneResult(BEV 패딩 픽셀 좌표) → 주행부 LaneObservation(앞차축 기준 차량 좌표, m).
// 픽셀 → 차량 변환은 cone_drive_test lane_post와 같다: x = x_max − (v + 0.5)·mpp, y = y_max − (u − pad + 0.5)·mpp
#pragma once
#include <vector>
#include "auto_control/drive/core/types.hpp"

namespace auto_control::drive {

struct LanePixels {
  struct P { float x, y; };
  std::vector<P> centerline, reference, left, right;
  int source_width{120}, padding_left{0};
  double bev_width_m{1.2}, bev_height_m{3.0};
  double stop_line_m{std::numeric_limits<double>::quiet_NaN()};   // 앞차축 → 정지선 (NaN = 없음)
  bool centerline_valid{false};
};

LaneObservation lane_from_pixels(const LanePixels & in, double capture_time);

}  // namespace auto_control::drive
