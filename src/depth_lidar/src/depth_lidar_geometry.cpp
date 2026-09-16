#include "depth_lidar/depth_lidar_geometry.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

namespace depth_lidar
{
namespace
{
bool validCamera(const CameraGeometry & camera)
{
  return camera.width > 0 && camera.height > 0 && camera.width <= 4096 && camera.height <= 4096
    && std::isfinite(camera.fx) && camera.fx > 0.0 && std::isfinite(camera.fy) && camera.fy > 0.0
    && std::isfinite(camera.cx) && std::isfinite(camera.cy);
}

} // namespace

bool validateProjectionConfig(const ProjectionConfig & c, std::string & reason)
{
  if (!std::isfinite(c.roi_width_ratio) || c.roi_width_ratio <= 0.0 || c.roi_width_ratio > 1.0
      || !std::isfinite(c.roi_height_ratio) || c.roi_height_ratio <= 0.0 || c.roi_height_ratio > 1.0
      || !std::isfinite(c.roi_bottom_offset_ratio) || c.roi_bottom_offset_ratio < 0.0
      || c.roi_height_ratio + c.roi_bottom_offset_ratio > 1.0) {
    reason = "ROI ratios must be finite, width/height in (0,1], height + bottom_offset <= 1";
    return false;
  }
  if (!std::isfinite(c.min_range_m) || !std::isfinite(c.max_range_m)
      || c.min_range_m <= 0.0 || c.max_range_m <= c.min_range_m
      || !std::isfinite(c.range_offset_m)) {
    reason = "range values must be finite with max_m > min_m > 0";
    return false;
  }
  if (c.pixel_stride < 1 || c.pixel_stride > 32) {
    reason = "points.pixel_stride must be in [1,32]";
    return false;
  }
  reason.clear();
  return true;
}

bool validateClusterConfig(const ClusterConfig & c, std::string & reason)
{
  if (c.min_points < 1 || c.min_points > 1000000
      || !std::isfinite(c.neighbor_distance_m) || c.neighbor_distance_m <= 0.0
      || !std::isfinite(c.radius_margin_m) || c.radius_margin_m < 0.0
      || !std::isfinite(c.min_radius_m) || c.min_radius_m <= 0.0) {
    reason = "cluster requires positive min_points/neighbor_distance/min_radius and nonnegative margin";
    return false;
  }
  reason.clear();
  return true;
}

bool validateGroundConfig(const GroundConfig & c, std::string & reason)
{
  ProjectionConfig roi;
  roi.roi_width_ratio = c.roi_width_ratio;
  roi.roi_height_ratio = c.roi_height_ratio;
  roi.roi_bottom_offset_ratio = c.roi_bottom_offset_ratio;
  roi.pixel_stride = c.pixel_stride;
  if (!validateProjectionConfig(roi, reason)) { reason = "ground " + reason; return false; }
  const double up_length = std::hypot(c.reference_up_x, c.reference_up_y, c.reference_up_z);
  if (c.max_samples < 100 || c.max_samples > 20000 || c.max_iterations < 1 || c.max_iterations > 2000
      || !std::isfinite(c.min_depth_m) || c.min_depth_m <= 0.0
      || !std::isfinite(c.max_depth_m) || c.max_depth_m <= c.min_depth_m || c.max_depth_m > 65.535
      || !std::isfinite(c.inlier_distance_m) || c.inlier_distance_m <= 0.0
      || c.min_inlier_points < 3 || c.min_inlier_points > c.max_samples
      || !std::isfinite(c.min_inlier_ratio) || c.min_inlier_ratio <= 0.0 || c.min_inlier_ratio > 1.0
      || !std::isfinite(c.min_spread_m) || c.min_spread_m <= 0.0
      || !std::isfinite(c.max_rmse_m) || c.max_rmse_m <= 0.0 || c.max_rmse_m > c.inlier_distance_m
      || !std::isfinite(up_length) || up_length < 1e-6 || c.reference_up_z <= 0.0
      || !std::isfinite(c.max_tilt_deg) || c.max_tilt_deg <= 0.0 || c.max_tilt_deg >= 85.0
      || !std::isfinite(c.min_camera_height_m) || c.min_camera_height_m <= 0.0
      || !std::isfinite(c.max_camera_height_m) || c.max_camera_height_m <= c.min_camera_height_m
      || !std::isfinite(c.min_height_m) || c.min_height_m <= c.inlier_distance_m
      || !std::isfinite(c.max_height_m) || c.max_height_m <= c.min_height_m
      || !std::isfinite(c.noise_scale) || c.noise_scale < 0.0
      || !std::isfinite(c.release_ratio) || c.release_ratio <= 0.0 || c.release_ratio > 1.0
      || !std::isfinite(c.reset_history_angle_deg) || c.reset_history_angle_deg <= 0.0 || c.reset_history_angle_deg > 90.0
      || !std::isfinite(c.reset_history_height_m) || c.reset_history_height_m <= 0.0) {
    reason = "invalid ground MSAC sampling, fit limits, direction/height constraints or obstacle height band";
    return false;
  }
  reason.clear();
  return true;
}

RoiRect computeRoi(int image_width, int image_height, double width_ratio,
  double height_ratio, double bottom_offset_ratio)
{
  if (image_width <= 0 || image_height <= 0) {
    throw std::invalid_argument("image dimensions must be positive");
  }
  const int width = std::clamp(static_cast<int>(std::lround(image_width * width_ratio)), 1, image_width);
  const int height = std::clamp(static_cast<int>(std::lround(image_height * height_ratio)), 1, image_height);
  const int offset = std::clamp(static_cast<int>(std::lround(image_height * bottom_offset_ratio)),
    0, image_height - height);
  return {(image_width - width) / 2, image_height - offset - height, width, height};
}

double GroundPlane::height(double forward, double left, double up) const
{
  return normal[0] * forward + normal[1] * left + normal[2] * up + offset_m;
}

bool GroundPlane::changedFrom(const GroundPlane & previous, const GroundConfig & c) const
{
  if (!valid || !previous.valid) { return true; }
  double dot = 0.0;
  for (int i = 0; i < 3; ++i) { dot += normal[i] * previous.normal[i]; }
  const double angle = std::acos(std::clamp(dot, -1.0, 1.0)) * 180.0 / CV_PI;
  return angle > c.reset_history_angle_deg || std::abs(offset_m - previous.offset_m) > c.reset_history_height_m;
}

GroundPlane estimateGroundPlane(const std::uint16_t * depth, std::size_t stride,
  const CameraGeometry & camera, const GroundConfig & c)
{
  std::string reason;
  if (!depth || !validCamera(camera) || stride < static_cast<std::size_t>(camera.width)
      || !validateGroundConfig(c, reason)) {
    throw std::invalid_argument("invalid ground input: " + reason);
  }
  GroundPlane result;
  const auto roi = computeRoi(camera.width, camera.height, c.roi_width_ratio,
    c.roi_height_ratio, c.roi_bottom_offset_ratio);
  int step = c.pixel_stride;
  const auto gridSize = [&](int s) {
    return static_cast<std::size_t>((roi.width + s - 1) / s) * ((roi.height + s - 1) / s);
  };
  while (gridSize(step) > static_cast<std::size_t>(c.max_samples)) { ++step; }
  std::vector<cv::Vec3d> points;
  points.reserve(gridSize(step));
  for (int v = roi.y; v < roi.y + roi.height; v += step) {
    for (int u = roi.x; u < roi.x + roi.width; u += step) {
      const double z = depth[static_cast<std::size_t>(v) * stride + u] * 0.001;
      if (z < c.min_depth_m || z > c.max_depth_m) { continue; }
      points.emplace_back(z, (camera.cx - u) * z / camera.fx, (camera.cy - v) * z / camera.fy);
    }
  }
  result.sample_points = points.size();
  const auto required = std::max(static_cast<std::size_t>(c.min_inlier_points),
    static_cast<std::size_t>(std::ceil(c.min_inlier_ratio * points.size())));
  if (points.size() < required) {
    result.reason = "TOO FEW VALID GROUND SAMPLES";
    return result;
  }
  cv::Vec3d expected_up(c.reference_up_x, c.reference_up_y, c.reference_up_z);
  expected_up *= 1.0 / cv::norm(expected_up);
  const double cos_tilt = std::cos(c.max_tilt_deg * CV_PI / 180.0);
  const auto allowed = [&](const cv::Vec3d & n, double d) {
    return n.dot(expected_up) >= cos_tilt && n[2] > std::cos(85.0 * CV_PI / 180.0)
      && d >= c.min_camera_height_m && d <= c.max_camera_height_m;
  };
  const double truncation = c.inlier_distance_m * c.inlier_distance_m;
  // MSAC score: sum of truncated squared point-to-plane distances. Outliers
  // contribute a bounded penalty; inlier fit accuracy also affects the score.
  const auto evaluate = [&](const cv::Vec3d & n, double d, double & score, double & error) {
    std::vector<std::size_t> inliers;
    score = 0.0;
    error = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const double distance = n.dot(points[i]) + d;
      const double squared = distance * distance;
      score += std::min(squared, truncation);
      if (squared <= truncation) { inliers.push_back(i); error += squared; }
    }
    return inliers;
  };
  std::mt19937 random(0x4d534143U);
  std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);
  double best_score = std::numeric_limits<double>::infinity();
  std::vector<std::size_t> best;
  for (int iteration = 0; iteration < c.max_iterations; ++iteration) {
    const auto a = pick(random), b = pick(random), e = pick(random);
    if (a == b || a == e || b == e) { continue; }
    cv::Vec3d n = (points[b] - points[a]).cross(points[e] - points[a]);
    const double length = cv::norm(n);
    if (length < 1e-8) { continue; }
    n *= 1.0 / length;
    if (n.dot(expected_up) < 0.0) { n *= -1.0; }
    const double d = -n.dot(points[a]);
    if (!allowed(n, d)) { continue; }
    double score, error;
    auto inliers = evaluate(n, d, score, error);
    if (inliers.size() >= required && (score < best_score
        || (score == best_score && inliers.size() > best.size()))) {
      best = std::move(inliers);
      best_score = score;
    }
  }
  if (best.empty()) {
    result.reason = "NO PLANE WITH REQUIRED DIRECTION / HEIGHT / SUPPORT";
    return result;
  }
  cv::Vec3d normal(0.0, 0.0, 1.0);
  double offset = 0.0, squared_error = 0.0;
  // Refine the winning model using orthogonal least squares, then recheck its
  // support. Covariance's second eigenvalue rejects line-like support.
  for (int pass = 0; pass < 2; ++pass) {
    cv::Vec3d center(0.0, 0.0, 0.0);
    for (const auto i : best) { center += points[i]; }
    center *= 1.0 / best.size();
    cv::Matx33d covariance = cv::Matx33d::zeros();
    for (const auto i : best) {
      const auto delta = points[i] - center;
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) { covariance(row, col) += delta[row] * delta[col]; }
      }
    }
    covariance *= 1.0 / best.size();
    cv::Mat values, vectors;
    if (!cv::eigen(cv::Mat(covariance), values, vectors)
        || values.at<double>(1) < c.min_spread_m * c.min_spread_m) {
      result.reason = "INSUFFICIENT GROUND SPREAD";
      return result;
    }
    normal = cv::Vec3d(vectors.at<double>(2, 0), vectors.at<double>(2, 1), vectors.at<double>(2, 2));
    normal *= 1.0 / cv::norm(normal);
    if (normal.dot(expected_up) < 0.0) { normal *= -1.0; }
    offset = -normal.dot(center);
    if (!allowed(normal, offset)) {
      result.reason = "REFINED PLANE OUTSIDE DIRECTION / HEIGHT LIMITS";
      return result;
    }
    double score;
    best = evaluate(normal, offset, score, squared_error);
    if (best.size() < required) {
      result.reason = "INSUFFICIENT REFINED GROUND SUPPORT";
      return result;
    }
  }
  result.inlier_points = best.size();
  result.rmse_m = std::sqrt(squared_error / best.size());
  if (result.rmse_m > c.max_rmse_m) {
    result.reason = "GROUND FIT ERROR TOO LARGE";
    return result;
  }
  if (std::max(c.min_height_m, c.noise_scale * result.rmse_m) >= c.max_height_m) {
    result.reason = "GROUND NOISE EXCEEDS OBSTACLE HEIGHT BAND";
    return result;
  }
  result.normal = {{normal[0], normal[1], normal[2]}};
  result.offset_m = offset;
  result.valid = true;
  result.reason.clear();
  return result;
}

DetectionResult detectForeground(const std::uint16_t * depth, std::size_t stride,
  const CameraGeometry & camera, const GroundPlane & ground, const GroundConfig & ground_config,
  const ProjectionConfig & projection, const ClusterConfig & cluster,
  std::vector<std::uint8_t> * foreground_mask)
{
  std::string reason;
  if (!validCamera(camera) || !depth || stride < static_cast<std::size_t>(camera.width)
      || !validateProjectionConfig(projection, reason) || !validateClusterConfig(cluster, reason)
      || !validateGroundConfig(ground_config, reason)) {
    throw std::invalid_argument("invalid foreground input: " + reason);
  }
  DetectionResult result;
  result.roi = computeRoi(camera.width, camera.height, projection.roi_width_ratio,
    projection.roi_height_ratio, projection.roi_bottom_offset_ratio);
  if (!ground.valid) {
    if (foreground_mask) { foreground_mask->clear(); }
    return result;
  }
  const auto pixels = static_cast<std::size_t>(camera.width) * camera.height;
  if (foreground_mask && foreground_mask->size() != pixels) { foreground_mask->assign(pixels, 0U); }
  const int step = projection.pixel_stride;
  const int cols = (result.roi.width + step - 1) / step;
  const int rows = (result.roi.height + step - 1) / step;
  std::vector<int> grid(static_cast<std::size_t>(cols) * rows, -1);
  std::vector<ForegroundPoint> candidates;
  candidates.reserve(grid.size());
  const double threshold = std::max(ground_config.min_height_m, ground_config.noise_scale * ground.rmse_m);
  // Even retained pixels must remain outside the ground inlier band.
  const double release = std::max(ground_config.inlier_distance_m, threshold * ground_config.release_ratio);
  for (int row = 0; row < rows; ++row) {
    const int v = result.roi.y + row * step;
    for (int col = 0; col < cols; ++col) {
      const int u = result.roi.x + col * step;
      const auto raw = depth[static_cast<std::size_t>(v) * stride + u];
      const auto pixel = static_cast<std::size_t>(v) * camera.width + u;
      const bool previous = foreground_mask && (*foreground_mask)[pixel] != 0U;
      const double forward = raw * 0.001;
      const double left = (camera.cx - u) * forward / camera.fx;
      const double up = (camera.cy - v) * forward / camera.fy;
      const double height = ground.height(forward, left, up);
      const bool foreground = raw && height > (previous ? release : threshold)
        && height <= ground_config.max_height_m;
      if (foreground_mask) { (*foreground_mask)[pixel] = foreground ? 1U : 0U; }
      if (!foreground) { continue; }
      const double range = std::hypot(forward, left);
      const double corrected = range + projection.range_offset_m;
      if (corrected < projection.min_range_m || corrected > projection.max_range_m) {
        if (foreground_mask) { (*foreground_mask)[pixel] = 0U; }
        continue;
      }
      const double scale = corrected / range;
      grid[static_cast<std::size_t>(row) * cols + col] = static_cast<int>(candidates.size());
      candidates.push_back({forward * scale, left * scale, up * scale, u, v});
    }
  }
  std::vector<bool> visited(grid.size(), false);
  std::vector<std::size_t> component;
  const double max_gap_squared = cluster.neighbor_distance_m * cluster.neighbor_distance_m;
  for (std::size_t seed = 0; seed < grid.size(); ++seed) {
    if (visited[seed] || grid[seed] < 0) { continue; }
    component.clear();
    component.push_back(seed);
    visited[seed] = true;
    double forward_sum = 0.0, left_sum = 0.0;
    for (std::size_t head = 0; head < component.size(); ++head) {
      const auto cell = component[head];
      const auto & point = candidates[grid[cell]];
      forward_sum += point.forward_m;
      left_sum += point.left_m;
      const int row = static_cast<int>(cell / cols), col = static_cast<int>(cell % cols);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nr = row + dy, nc = col + dx;
          if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) { continue; }
          const auto next = static_cast<std::size_t>(nr) * cols + nc;
          if (visited[next] || grid[next] < 0) { continue; }
          const auto & neighbor = candidates[grid[next]];
          const double df = point.forward_m - neighbor.forward_m;
          const double dl = point.left_m - neighbor.left_m;
          const double du = point.up_m - neighbor.up_m;
          if (df * df + dl * dl + du * du <= max_gap_squared) {
            visited[next] = true;
            component.push_back(next);
          }
        }
      }
    }
    if (component.size() < static_cast<std::size_t>(cluster.min_points)) { continue; }
    const double x = forward_sum / component.size(), y = left_sum / component.size();
    double radius = 0.0;
    for (const auto cell : component) {
      const auto & point = candidates[grid[cell]];
      radius = std::max(radius, std::hypot(point.forward_m - x, point.left_m - y));
      result.points.push_back(point);
    }
    // Do not cap the radius: a cap would under-represent a large obstacle.
    result.obstacles.push_back({x, y,
      std::max(cluster.min_radius_m, radius + cluster.radius_margin_m), component.size()});
  }
  return result;
}
} // namespace depth_lidar
