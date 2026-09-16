#pragma once
#include "depth_lidar/depth_lidar_geometry.hpp"
#include <limits>

namespace depth_lidar
{
struct StabilizationConfig
{
  bool enabled{true};
  int confirm_hits{2};
  int window_frames{3};
  double hold_sec{0.08};
  double max_frame_gap_sec{0.20};
};
bool validateStabilizationConfig(const StabilizationConfig & config, std::string & reason);

// Cell confirmation avoids pairwise cluster association. Times are host monotonic
// times for both update and snapshot. No odometry or accumulation beyond hold_sec.
class GridStabilizer
{
public:
  void clear();
  DetectionResult snapshot(double time_sec, const StabilizationConfig & config);
  DetectionResult update(const DetectionResult & raw, double time_sec, const StabilizationConfig & config);
  double nextExpiryTime() const { return next_expiry_sec_; }
private:
  struct CellState {
    std::uint32_t history{0U}, support_points{0U};
    double last_seen_sec{0.0};
    bool confirmed{false};
  };
  DetectionResult render(double time_sec, const StabilizationConfig & config);
  std::vector<CellState> states_;
  GridConfig grid_;
  ClusterConfig cluster_;
  RoiRect roi_;
  bool have_time_{false};
  double previous_time_sec_{0.0};
  double next_expiry_sec_{std::numeric_limits<double>::infinity()};
};
} // namespace depth_lidar
