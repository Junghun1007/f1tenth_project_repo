#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace depth_lidar
{

struct RoiRect
{
  int x{0};
  int y{0};
  int width{0};
  int height{0};
};

struct ProjectionConfig
{
  double roi_width_ratio{1.0};
  double roi_height_ratio{0.10};
  double roi_bottom_offset_ratio{0.35};
  double min_range_m{0.20};
  double max_range_m{8.0};
  double range_offset_m{0.0};
  int scan_bins{360};
  int pixel_stride{1};
  int min_points_per_bin{1};
};

struct ScanProjection
{
  RoiRect roi;
  float angle_min{0.0F};
  float angle_max{0.0F};
  float angle_increment{0.0F};
  std::vector<float> ranges;
  std::size_t valid_input_points{0};
  std::size_t valid_bins{0};
};

struct ClusterConfig
{
  int min_bins{3};
  int max_missing_bins{1};
  double base_neighbor_distance_m{0.04};
  double angular_neighbor_scale{1.5};
  double radius_margin_m{0.04};
  double min_radius_m{0.05};
  double max_radius_m{0.40};
};

struct ObstacleCircle
{
  double forward_m{0.0};
  double left_m{0.0};
  double radius_m{0.0};
  double representative_range_m{0.0};
  std::size_t support_bins{0};
  std::size_t first_bin{0};
  std::size_t last_bin{0};
};

bool validateProjectionConfig(const ProjectionConfig & config, std::string & reason);

bool validateClusterConfig(const ClusterConfig & config, std::string & reason);

RoiRect computeRoi(int image_width,
  int image_height,
  double width_ratio,
  double height_ratio,
  double bottom_offset_ratio);

ScanProjection projectDepthToScan(const std::uint16_t * depth_mm,
  int image_width,
  int image_height,
  std::size_t row_stride_elements,
  double fx,
  double cx,
  const ProjectionConfig & config);

std::vector<ObstacleCircle> clusterScan(const ScanProjection & projection,
  const ClusterConfig & config);

} // namespace depth_lidar
