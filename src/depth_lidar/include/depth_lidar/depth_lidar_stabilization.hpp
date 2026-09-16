#pragma once
#include "depth_lidar/depth_lidar_geometry.hpp"
#include <limits>
namespace depth_lidar {
// Optional short missing-return hold, no shape extraction, association or motion model.
class ScanHold {
public:
  void clear();
  ScanResult update(const ScanResult & raw, double host_time_sec, double hold_sec);
  ScanResult snapshot(double host_time_sec, double hold_sec);
  double nextExpiryTime() const { return next_expiry_; }
private:
  ScanResult render(double host_time_sec, double hold_sec);
  std::vector<float> ranges_;
  std::vector<double> times_;
  double next_expiry_{std::numeric_limits<double>::infinity()};
};
}
