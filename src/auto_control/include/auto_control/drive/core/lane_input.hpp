// line_detactor LaneResult(BEV 패딩 픽셀 좌표) → 주행부 LaneObservation(앞차축 기준 차량 좌표, m).
// 픽셀 → 차량 변환은 cone_drive_test lane_post와 같다: x = x_max − (v + 0.5)·mpp, y = y_max − (u − pad + 0.5)·mpp
#pragma once
#include <limits>
#include <optional>
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

enum class LaneSource { RGB, IR };

class LaneContinuity {
public:
  // Keep the last observed geometry as a fixed reference during avoidance.
  // Matching frames may extend the control path without moving that reference.
  bool accept(LaneObservation & lane, const std::optional<Pose2> & pose,
              bool avoiding, double lane_half_width, double vehicle_half_width, double wheelbase);

private:
  struct Reference {
    LaneObservation lane;
    Pose2 pose;
  };
  std::optional<Reference> reference_;
  double last_capture_{-std::numeric_limits<double>::infinity()};
  bool holding_{false};
  int departure_side_{0};  // +1 left, -1 right
  int matching_frames_{0};
};

}  // namespace auto_control::drive
