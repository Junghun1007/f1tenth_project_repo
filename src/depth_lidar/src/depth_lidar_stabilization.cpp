#include "depth_lidar/depth_lidar_stabilization.hpp"
#include <algorithm>
#include <cmath>
namespace depth_lidar {
void ScanHold::clear() { ranges_.clear(); times_.clear(); next_expiry_ = std::numeric_limits<double>::infinity(); }
ScanResult ScanHold::render(double now, double hold) {
  ScanResult result;
  result.ranges = ranges_;
  result.ages.assign(ranges_.size(), std::numeric_limits<float>::quiet_NaN());
  next_expiry_ = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < ranges_.size(); ++i) {
    if (!std::isfinite(ranges_[i])) { continue; }
    if (now - times_[i] > hold || now < times_[i]) { ranges_[i] = result.ranges[i] = std::numeric_limits<float>::quiet_NaN(); continue; }
    result.ages[i] = static_cast<float>(now - times_[i]);
    ++result.valid_bins;
    next_expiry_ = std::min(next_expiry_, times_[i] + hold);
  }
  return result;
}
ScanResult ScanHold::update(const ScanResult & raw, double now, double hold) {
  if (hold <= 0.0) { clear(); return raw; } // Latest frame only; no idle expiry between frames.
  if (ranges_.size() != raw.ranges.size()) {
    ranges_.assign(raw.ranges.size(), std::numeric_limits<float>::quiet_NaN());
    times_.assign(raw.ranges.size(), 0.0);
  }
  for (std::size_t i = 0; i < ranges_.size(); ++i) {
    if (std::isfinite(raw.ranges[i])) { ranges_[i] = raw.ranges[i]; times_[i] = now; }
  }
  auto result = render(now, hold);
  result.accepted_points = raw.accepted_points;
  return result;
}
ScanResult ScanHold::snapshot(double now, double hold) { return render(now, hold); }
}
