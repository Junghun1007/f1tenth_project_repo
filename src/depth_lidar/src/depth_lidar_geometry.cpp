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

bool validateClusterConfig(const ClusterConfig & config, std::string & reason)
{
  if (config.min_bins < 1 || config.min_bins > 4096) {
    reason = "cluster.min_bins must be in [1, 4096]";
    return false;
  }
  if (config.max_missing_bins < 0 || config.max_missing_bins > 64) {
    reason = "cluster.max_missing_bins must be in [0, 64]";
    return false;
  }
  if (!std::isfinite(config.base_neighbor_distance_m) || config.base_neighbor_distance_m < 0.0) {
    reason = "cluster.base_neighbor_distance_m must be finite and non-negative";
    return false;
  }
  if (!std::isfinite(config.angular_neighbor_scale) || config.angular_neighbor_scale < 0.0) {
    reason = "cluster.angular_neighbor_scale must be finite and non-negative";
    return false;
  }
  if (!std::isfinite(config.radius_margin_m) || config.radius_margin_m < 0.0) {
    reason = "cluster.radius_margin_m must be finite and non-negative";
    return false;
  }
  if (!std::isfinite(config.min_radius_m) || !std::isfinite(config.max_radius_m)
      || config.min_radius_m <= 0.0 || config.max_radius_m < config.min_radius_m)
  {
    reason = "cluster radii must satisfy max_radius_m >= min_radius_m > 0";
    return false;
  }
  reason.clear();
  return true;
}

RoiRect computeRoi(const int image_width,
  const int image_height,
  const double width_ratio,
  const double height_ratio,
  const double bottom_offset_ratio)
{
  if (image_width <= 0 || image_height <= 0) {
    throw std::invalid_argument("image dimensions must be positive");
  }

  const int roi_width =
    std::clamp(static_cast<int>(std::lround(static_cast<double>(image_width) * width_ratio)),
      1,
      image_width);
  const int roi_height =
    std::clamp(static_cast<int>(std::lround(static_cast<double>(image_height) * height_ratio)),
      1,
      image_height);
  const int bottom_offset = std::clamp(
    static_cast<int>(std::lround(static_cast<double>(image_height) * bottom_offset_ratio)),
    0,
    image_height - roi_height);

  return RoiRect{(image_width - roi_width) / 2,
    image_height - bottom_offset - roi_height,
    roi_width,
    roi_height};
}

ScanProjection projectDepthToScan(const std::uint16_t * const depth_mm,
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
  output.roi = computeRoi(image_width,
    image_height,
    config.roi_width_ratio,
    config.roi_height_ratio,
    config.roi_bottom_offset_ratio);

  // ROS LaserScan convention: x is forward, y is left, and positive angles turn
  // left. Pixel-cell edges are used so even a one-column ROI has a non-zero
  // angular span.
  const double left_edge = static_cast<double>(output.roi.x) - 0.5;
  const double right_edge = static_cast<double>(output.roi.x + output.roi.width) - 0.5;
  output.angle_min = static_cast<float>(std::atan2(cx - right_edge, fx));
  output.angle_max = static_cast<float>(std::atan2(cx - left_edge, fx));
  output.angle_increment =
    (output.angle_max - output.angle_min) / static_cast<float>(config.scan_bins - 1);
  output.ranges.assign(static_cast<std::size_t>(config.scan_bins),
    std::numeric_limits<float>::infinity());
  std::vector<int> point_counts(static_cast<std::size_t>(config.scan_bins), 0);

  for (int v = output.roi.y; v < output.roi.y + output.roi.height; v += config.pixel_stride) {
    const std::uint16_t * const row = depth_mm + static_cast<std::size_t>(v) * row_stride_elements;
    for (int u = output.roi.x; u < output.roi.x + output.roi.width; u += config.pixel_stride) {
      const std::uint16_t raw_depth_mm = row[u];
      if (raw_depth_mm == 0U) {
        continue;
      }

      const double forward_m = static_cast<double>(raw_depth_mm) * 0.001;
      const double left_m = -(static_cast<double>(u) - cx) * forward_m / fx;
      const double measured_range_m = std::hypot(forward_m, left_m);
      const double corrected_range_m = measured_range_m + config.range_offset_m;
      if (!std::isfinite(corrected_range_m) || corrected_range_m < config.min_range_m
          || corrected_range_m > config.max_range_m)
      {
        continue;
      }

      const double angle = std::atan2(left_m, forward_m);
      const auto bin = static_cast<int>(std::lround((angle - static_cast<double>(output.angle_min))
                                                    / static_cast<double>(output.angle_increment)));
      if (bin < 0 || bin >= config.scan_bins) {
        continue;
      }

      const std::size_t index = static_cast<std::size_t>(bin);
      output.ranges[index] = std::min(output.ranges[index], static_cast<float>(corrected_range_m));
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

std::vector<ObstacleCircle> clusterScan(const ScanProjection & projection,
  const ClusterConfig & config)
{
  std::string reason;
  if (!validateClusterConfig(config, reason)) {
    throw std::invalid_argument(reason);
  }
  if (projection.ranges.empty() || !std::isfinite(projection.angle_min)
      || !std::isfinite(projection.angle_increment) || projection.angle_increment <= 0.0F)
  {
    throw std::invalid_argument("scan angles and ranges must describe a non-empty ordered scan");
  }

  struct PolarPoint
  {
    std::size_t bin;
    double range_m;
    double forward_m;
    double left_m;
  };

  std::vector<ObstacleCircle> obstacles;
  std::vector<PolarPoint> cluster;
  cluster.reserve(projection.ranges.size());

  const auto finish_cluster = [&]() {
    if (cluster.size() < static_cast<std::size_t>(config.min_bins)) {
      cluster.clear();
      return;
    }

    std::vector<double> ranges;
    ranges.reserve(cluster.size());
    double forward_sum = 0.0;
    double left_sum = 0.0;
    for (const auto & point : cluster) {
      ranges.push_back(point.range_m);
      forward_sum += point.forward_m;
      left_sum += point.left_m;
    }
    const auto middle = ranges.begin() + static_cast<std::ptrdiff_t>(ranges.size() / 2U);
    std::nth_element(ranges.begin(), middle, ranges.end());
    const double representative_range_m = *middle;
    const std::size_t span_bins = cluster.back().bin - cluster.front().bin + 1U;
    const double angular_width =
      static_cast<double>(span_bins) * static_cast<double>(projection.angle_increment);
    const double physical_width_m = 2.0 * representative_range_m * std::sin(0.5 * angular_width);
    const double radius_m = std::clamp(0.5 * physical_width_m + config.radius_margin_m,
      config.min_radius_m,
      config.max_radius_m);

    obstacles.push_back(ObstacleCircle{forward_sum / static_cast<double>(cluster.size()),
      left_sum / static_cast<double>(cluster.size()),
      radius_m,
      representative_range_m,
      cluster.size(),
      cluster.front().bin,
      cluster.back().bin});
    cluster.clear();
  };

  for (std::size_t bin = 0; bin < projection.ranges.size(); ++bin) {
    const double range_m = static_cast<double>(projection.ranges[bin]);
    if (!std::isfinite(range_m)) {
      continue;
    }
    const double angle =
      static_cast<double>(projection.angle_min)
      + static_cast<double>(bin) * static_cast<double>(projection.angle_increment);
    const PolarPoint point{bin, range_m, range_m * std::cos(angle), range_m * std::sin(angle)};

    if (!cluster.empty()) {
      const PolarPoint & previous = cluster.back();
      const std::size_t missing_bins = point.bin - previous.bin - 1U;
      const double point_gap_m =
        std::hypot(point.forward_m - previous.forward_m, point.left_m - previous.left_m);
      const double angular_gap = static_cast<double>(point.bin - previous.bin)
                                 * static_cast<double>(projection.angle_increment);
      const double mean_range_m = 0.5 * (point.range_m + previous.range_m);
      const double allowed_gap_m = config.base_neighbor_distance_m
                                   + config.angular_neighbor_scale * mean_range_m * angular_gap;
      if (missing_bins > static_cast<std::size_t>(config.max_missing_bins)
          || point_gap_m > allowed_gap_m)
      {
        finish_cluster();
      }
    }
    cluster.push_back(point);
  }
  finish_cluster();
  return obstacles;
}

} // namespace depth_lidar
