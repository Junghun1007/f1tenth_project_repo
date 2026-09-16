#pragma once

#include "depth_lidar/depth_lidar_geometry.hpp"

namespace depth_lidar
{
struct StabilizationConfig
{
  bool enabled{true};
  int confirm_hits{2};
  int window_frames{3};
  double hold_sec{0.08};
  double match_distance_m{0.15};
  double max_frame_gap_sec{0.20};
};

bool validateStabilizationConfig(const StabilizationConfig & config, std::string & reason);

// Caller resets both the pixel mask and this history on invalid/jumping planes,
// detection-setting changes and camera restarts. Times are monotonic frame times.
class ClusterStabilizer
{
public:
  void clear();
  DetectionResult snapshot(double time_sec, const StabilizationConfig & config) const;
  DetectionResult update(const DetectionResult & raw, double time_sec,
    const StabilizationConfig & config);

private:
  struct Track
  {
    ObstacleCircle circle;
    std::uint32_t history{0U};
    double last_seen_sec{0.0};
    bool confirmed{false};
  };
  std::vector<Track> tracks_;
  double previous_time_sec_{0.0};
  bool have_time_{false};
};
} // namespace depth_lidar
