#include "depth_lidar/depth_lidar_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace depth_lidar
{

bool validateProjectionConfig(const ProjectionConfig & config, std::string & reason)
{
  if (config.roi_width_ratio <= 0.0 || config.roi_width_ratio > 1.0) {
    reason = "roi.width_ratio must be in (0.0, 1.0]";
    return false;
  }
  if (config.roi_height_ratio <= 0.0 || config.roi_height_ratio > 1.0) {
    reason = "roi.height_ratio must be in (0.0, 1.0]";
    return false;
  }
  if (config.roi_bottom_offset_ratio < 0.0 || config.roi_bottom_offset_ratio >= 1.0) {
    reason = "roi.bottom_offset_ratio must be in [0.0, 1.0)";
    return false;
  }
  if (config.roi_height_ratio + config.roi_bottom_offset_ratio > 1.0) {
    reason = "roi.height_ratio + roi.bottom_offset_ratio must be <= 1.0";
    return false;
  }
  if (config.min_range_m <= 0.0 || config.max_range_m <= config.min_range_m) {
    reason = "range.max_m must be greater than range.min_m > 0.0";
    return false;
  }
  if (!std::isfinite(config.range_offset_m)) {
    reason = "range.offset_m must be finite";
    return false;
  }
  if (config.scan_bins < 2 || config.scan_bins > 4096) {
    reason = "scan.bins must be in [2, 4096]";
    return false;
  }
  if (config.pixel_stride < 1 || config.pixel_stride > 32) {
    reason = "scan.pixel_stride must be in [1, 32]";
    return false;
  }
  if (config.min_points_per_bin < 1 || config.min_points_per_bin > 10000) {
    reason = "scan.min_points_per_bin must be in [1, 10000]";
    return false;
  }
  reason.clear();
  return true;
}

RoiRect computeRoi(
  const int image_width,
  const int image_height,
  const double width_ratio,
  const double height_ratio,
  const double bottom_offset_ratio)
{
  if (image_width <= 0 || image_height <= 0) {
    throw std::invalid_argument("image dimensions must be positive");
  }

  const int roi_width = std::clamp(
    static_cast<int>(std::lround(static_cast<double>(image_width) * width_ratio)),
    1, image_width);
  const int roi_height = std::clamp(
    static_cast<int>(std::lround(static_cast<double>(image_height) * height_ratio)),
    1, image_height);
  const int bottom_offset = std::clamp(
    static_cast<int>(std::lround(static_cast<double>(image_height) * bottom_offset_ratio)),
    0, image_height - roi_height);

  return RoiRect{
    (image_width - roi_width) / 2,
    image_height - bottom_offset - roi_height,
    roi_width,
    roi_height};
}

ScanProjection projectDepthToScan(
  const std::uint16_t * const depth_mm,
  const int image_width,
  const int image_height,
  const std::size_t row_stride_elements,
  const double fx,
  const double cx,
  const ProjectionConfig & config)
{
  std::string reason;
  if (!validateProjectionConfig(config, reason)) {
    throw std::invalid_argument(reason);
  }
  if (depth_mm == nullptr || image_width <= 0 || image_height <= 0) {
    throw std::invalid_argument("depth buffer and image dimensions must be valid");
  }
  if (row_stride_elements < static_cast<std::size_t>(image_width)) {
    throw std::invalid_argument("row stride is smaller than image width");
  }
  if (!std::isfinite(fx) || !std::isfinite(cx) || fx <= 0.0) {
    throw std::invalid_argument("camera intrinsics must be finite and fx must be positive");
  }

  ScanProjection output;
  output.roi = computeRoi(
    image_width, image_height, config.roi_width_ratio, config.roi_height_ratio,
    config.roi_bottom_offset_ratio);

  // ROS LaserScan convention: x is forward, y is left, and positive angles turn left.
  // Pixel-cell edges are used so even a one-column ROI has a non-zero angular span.
  const double left_edge = static_cast<double>(output.roi.x) - 0.5;
  const double right_edge = static_cast<double>(output.roi.x + output.roi.width) - 0.5;
  output.angle_min = static_cast<float>(std::atan2(cx - right_edge, fx));
  output.angle_max = static_cast<float>(std::atan2(cx - left_edge, fx));
  output.angle_increment =
    (output.angle_max - output.angle_min) / static_cast<float>(config.scan_bins - 1);
  output.ranges.assign(
    static_cast<std::size_t>(config.scan_bins), std::numeric_limits<float>::infinity());
  std::vector<int> point_counts(static_cast<std::size_t>(config.scan_bins), 0);

  for (int v = output.roi.y; v < output.roi.y + output.roi.height;
    v += config.pixel_stride)
  {
    const std::uint16_t * const row =
      depth_mm + static_cast<std::size_t>(v) * row_stride_elements;
    for (int u = output.roi.x; u < output.roi.x + output.roi.width;
      u += config.pixel_stride)
    {
      const std::uint16_t raw_depth_mm = row[u];
      if (raw_depth_mm == 0U) {
        continue;
      }

      const double forward_m = static_cast<double>(raw_depth_mm) * 0.001;
      const double left_m = -(static_cast<double>(u) - cx) * forward_m / fx;
      const double measured_range_m = std::hypot(forward_m, left_m);
      const double corrected_range_m = measured_range_m + config.range_offset_m;
      if (!std::isfinite(corrected_range_m) || corrected_range_m < config.min_range_m ||
        corrected_range_m > config.max_range_m)
      {
        continue;
      }

      const double angle = std::atan2(left_m, forward_m);
      const auto bin = static_cast<int>(std::lround(
          (angle - static_cast<double>(output.angle_min)) /
          static_cast<double>(output.angle_increment)));
      if (bin < 0 || bin >= config.scan_bins) {
        continue;
      }

      const std::size_t index = static_cast<std::size_t>(bin);
      output.ranges[index] =
        std::min(output.ranges[index], static_cast<float>(corrected_range_m));
      ++point_counts[index];
      ++output.valid_input_points;
    }
  }

  for (std::size_t i = 0; i < output.ranges.size(); ++i) {
    if (point_counts[i] < config.min_points_per_bin) {
      output.ranges[i] = std::numeric_limits<float>::infinity();
    } else {
      ++output.valid_bins;
    }
  }
  return output;
}

}  // namespace depth_lidar
