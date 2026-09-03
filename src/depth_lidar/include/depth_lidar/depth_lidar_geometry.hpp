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

bool validateProjectionConfig(const ProjectionConfig & config, std::string & reason);

RoiRect computeRoi(
  int image_width,
  int image_height,
  double width_ratio,
  double height_ratio,
  double bottom_offset_ratio);

ScanProjection projectDepthToScan(
  const std::uint16_t * depth_mm,
  int image_width,
  int image_height,
  std::size_t row_stride_elements,
  double fx,
  double cx,
  const ProjectionConfig & config);

}  // namespace depth_lidar
