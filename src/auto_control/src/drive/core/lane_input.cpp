#include "auto_control/drive/core/lane_input.hpp"
#include <algorithm>
#include <cmath>

namespace auto_control::drive {

LaneObservation lane_from_pixels(const LanePixels & in, double t) {
  LaneObservation out;
  out.capture_time = t;
  if (in.source_width <= 0 || !(in.bev_width_m > 0) || !(in.bev_height_m > 0)) return out;
  const double mpp = in.bev_width_m / in.source_width;
  const double x_max = in.bev_height_m, y_max = in.bev_width_m / 2.0;
  auto conv = [&](const std::vector<LanePixels::P> & src, std::vector<Vec2> & dst) {
    dst.reserve(src.size());
    for (const auto & p : src) dst.push_back({x_max - (p.y + 0.5) * mpp, y_max - (p.x - in.padding_left + 0.5) * mpp});
  };
  if (in.centerline_valid) conv(in.centerline, out.path);
  conv(in.reference, out.center);
  conv(in.left, out.left);
  conv(in.right, out.right);
  for (auto * side : {&out.left, &out.right})
    std::sort(side->begin(), side->end(), [](const Vec2 & a, const Vec2 & b) { return a.x < b.x; });
  out.stop_line_m = std::isfinite(in.stop_line_m) ? in.stop_line_m : std::numeric_limits<double>::quiet_NaN();
  out.valid = out.path.size() >= 4;
  return out;
}

}  // namespace auto_control::drive
