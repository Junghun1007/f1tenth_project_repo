#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace depth_lidar
{
struct RoiRect { int x{0}; int y{0}; int width{0}; int height{0}; };

struct CameraGeometry
{
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

struct ProjectionConfig
{
  double roi_width_ratio{1.0};
  double roi_height_ratio{0.30};
  double roi_bottom_offset_ratio{0.25};
  double min_range_m{0.10};
  double max_range_m{3.0};
  double range_offset_m{0.0};
  int pixel_stride{1};
};

struct ClusterConfig
{
  int min_points{20};
  double neighbor_distance_m{0.08};
  double radius_margin_m{0.04};
  double min_radius_m{0.05};
};

struct GroundConfig
{
  double roi_width_ratio{0.90};
  double roi_height_ratio{0.55};
  double roi_bottom_offset_ratio{0.0};
  int pixel_stride{4};
  int max_samples{3000};
  int max_iterations{200};
  double min_depth_m{0.10};
  double max_depth_m{4.0};
  double inlier_distance_m{0.02};
  int min_inlier_points{100};
  double min_inlier_ratio{0.35};
  double min_spread_m{0.05};
  double max_rmse_m{0.015};
  double reference_up_x{0.0};
  double reference_up_y{0.0};
  double reference_up_z{1.0};
  double max_tilt_deg{60.0};
  double min_camera_height_m{0.05};
  double max_camera_height_m{1.0};
  double min_height_m{0.05};
  double max_height_m{1.0};
  double noise_scale{3.0};
  double release_ratio{0.60};
  double reset_history_angle_deg{3.0};
  double reset_history_height_m{0.03};
};

// Unit normal points toward the expected up direction in camera forward/left/up.
// No calibration file or previous-frame plane is used to produce this estimate.
struct GroundPlane
{
  bool valid{false};
  std::array<double, 3> normal{{0.0, 0.0, 1.0}};
  double offset_m{0.0};
  double rmse_m{0.0};
  std::size_t sample_points{0};
  std::size_t inlier_points{0};
  std::string reason{"not estimated"};
  double height(double forward, double left, double up) const;
  bool changedFrom(const GroundPlane & previous, const GroundConfig & config) const;
};

struct ForegroundPoint
{
  double forward_m{0.0};
  double left_m{0.0};
  double up_m{0.0};
  int u{0};
  int v{0};
};

struct ObstacleCircle
{
  double forward_m{0.0};
  double left_m{0.0};
  double radius_m{0.0};
  std::size_t support_points{0};
  double observation_age_sec{0.0};
};

struct DetectionResult
{
  RoiRect roi;
  std::vector<ForegroundPoint> points;
  std::vector<ObstacleCircle> obstacles;
};

bool validateProjectionConfig(const ProjectionConfig & config, std::string & reason);
bool validateClusterConfig(const ClusterConfig & config, std::string & reason);
bool validateGroundConfig(const GroundConfig & config, std::string & reason);
RoiRect computeRoi(int image_width, int image_height,
  double width_ratio, double height_ratio, double bottom_offset_ratio);

GroundPlane estimateGroundPlane(const std::uint16_t * depth_mm,
  std::size_t row_stride_elements, const CameraGeometry & camera, const GroundConfig & config);

// Pixel labels are optional, transient hysteresis state. Caller clears them
// after invalid/jumping planes and changes to camera, ROI or detection settings.
DetectionResult detectForeground(const std::uint16_t * depth_mm,
  std::size_t row_stride_elements, const CameraGeometry & camera,
  const GroundPlane & ground, const GroundConfig & ground_config,
  const ProjectionConfig & projection, const ClusterConfig & cluster,
  std::vector<std::uint8_t> * foreground_mask = nullptr);
} // namespace depth_lidar
