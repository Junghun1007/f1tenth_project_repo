#include "semi_slam/core/lane_fit.hpp"
#include <cmath>

namespace semi_slam {

LineFit fit_lane_line(const std::vector<Vec2> & pts, double x0, double x1, int min_n) {
  LineFit f;
  double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (const auto & q : pts) if (q.x >= x0 && q.x <= x1) { ++n; sx += q.x; sy += q.y; sxx += q.x * q.x; sxy += q.x * q.y; }
  const double den = n * sxx - sx * sx;
  if (n < min_n || std::abs(den) < 1e-9) return f;
  f.a = (n * sxy - sx * sy) / den;
  f.b = (sy - f.a * sx) / n;
  f.ok = true;
  return f;
}

LaneFrameFit fit_lane_frame(const std::vector<Vec2> & left, const std::vector<Vec2> & right, double x0, double x1, double at_x) {
  LaneFrameFit r;
  r.left = fit_lane_line(left, x0, x1);
  r.right = fit_lane_line(right, x0, x1);
  if (!r.left.ok || !r.right.ok) return r;
  const double gap = r.left.y_at(at_x) - r.right.y_at(at_x);
  r.both = std::abs(r.left.a - r.right.a) < 0.1 && gap > 0.3 && gap < 1.0;
  if (!r.both) return r;
  const double a = 0.5 * (r.left.a + r.right.a);
  r.angle = std::atan(a);
  r.center_y = 0.5 * (r.left.y_at(at_x) + r.right.y_at(at_x));
  r.width = gap * std::cos(r.angle);
  return r;
}

}  // namespace semi_slam
