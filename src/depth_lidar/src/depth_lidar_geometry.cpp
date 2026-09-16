#include "depth_lidar/depth_lidar_geometry.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
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

template<typename T> void writeValue(std::ostream & stream, const T & value)
{
  stream.write(reinterpret_cast<const char *>(&value), sizeof(value));
}
template<typename T> void readValue(std::istream & stream, T & value)
{
  stream.read(reinterpret_cast<char *>(&value), sizeof(value));
  if (!stream) { throw std::runtime_error("truncated floor reference"); }
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

bool validateFloorConfig(const FloorConfig & c, std::string & reason)
{
  ProjectionConfig roi;
  roi.roi_width_ratio = c.measure_roi_width_ratio;
  roi.roi_height_ratio = c.measure_roi_height_ratio;
  roi.roi_bottom_offset_ratio = c.measure_roi_bottom_offset_ratio;
  roi.pixel_stride = c.fit_pixel_stride;
  if (!validateProjectionConfig(roi, reason)) {
    reason = "floor measurement " + reason;
    return false;
  }
  if (c.measure_frames < 2 || c.measure_frames > 3600
      || !std::isfinite(c.min_valid_ratio) || c.min_valid_ratio <= 0.0 || c.min_valid_ratio > 1.0
      || !std::isfinite(c.fit_max_depth_m) || c.fit_max_depth_m <= 0.0 || c.fit_max_depth_m > 65.535
      || c.ransac_iterations < 1 || c.ransac_iterations > 5000
      || !std::isfinite(c.inlier_distance_m) || c.inlier_distance_m <= 0.0
      || c.min_inlier_points < 3 || c.min_inlier_points > 1000000
      || !std::isfinite(c.min_inlier_ratio) || c.min_inlier_ratio <= 0.0 || c.min_inlier_ratio > 1.0
      || !std::isfinite(c.max_tilt_deg) || c.max_tilt_deg <= 0.0 || c.max_tilt_deg >= 85.0
      || !std::isfinite(c.min_camera_height_m) || c.min_camera_height_m <= 0.0
      || !std::isfinite(c.max_camera_height_m) || c.max_camera_height_m <= c.min_camera_height_m
      || !std::isfinite(c.min_height_m) || c.min_height_m <= 0.0
      || !std::isfinite(c.max_height_m) || c.max_height_m <= c.min_height_m
      || !std::isfinite(c.noise_scale) || c.noise_scale < 0.0) {
    reason = "invalid floor sampling, RANSAC, tilt/camera-height limits or obstacle height band";
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

void FloorReference::clear() { *this = FloorReference{}; }

void FloorReference::begin(const CameraGeometry & camera, const FloorConfig & config)
{
  clear();
  std::string reason;
  if (!validCamera(camera) || !validateFloorConfig(config, reason)) {
    throw std::invalid_argument("invalid floor measurement geometry/config: " + reason);
  }
  camera_ = camera;
  measurement_config_ = config;
  target_frames_ = config.measure_frames;
  min_samples_ = std::max(2, static_cast<int>(std::ceil(target_frames_ * config.min_valid_ratio)));
  const auto size = static_cast<std::size_t>(camera.width) * camera.height;
  mean_m_.assign(size, 0.0);
  counts_.assign(size, 0U);
  measuring_ = true;
}

bool FloorReference::compatible(const CameraGeometry & c) const
{
  const auto close = [](double a, double b) { return std::abs(a - b) <= 1e-5 * std::max(1.0, std::abs(a)); };
  return validCamera(c) && camera_.width == c.width && camera_.height == c.height
    && camera_.signature == c.signature && close(camera_.fx, c.fx) && close(camera_.fy, c.fy)
    && close(camera_.cx, c.cx) && close(camera_.cy, c.cy);
}

bool FloorReference::accumulate(const std::uint16_t * depth, std::size_t stride)
{
  if (!measuring_ || depth == nullptr || stride < static_cast<std::size_t>(camera_.width)) {
    throw std::invalid_argument("floor measurement requires a valid depth frame and active session");
  }
  const auto & c = measurement_config_;
  const auto roi = computeRoi(camera_.width, camera_.height, c.measure_roi_width_ratio,
    c.measure_roi_height_ratio, c.measure_roi_bottom_offset_ratio);
  for (int v = roi.y; v < roi.y + roi.height; v += c.fit_pixel_stride) {
    for (int u = roi.x; u < roi.x + roi.width; u += c.fit_pixel_stride) {
      const double value = depth[static_cast<std::size_t>(v) * stride + u] * 0.001;
      if (value <= 0.0 || value > c.fit_max_depth_m) { continue; }
      const auto i = static_cast<std::size_t>(v) * camera_.width + u;
      const auto count = ++counts_[i];
      mean_m_[i] += (value - mean_m_[i]) / count;
    }
  }
  if (++frames_ < target_frames_) { return false; }
  measuring_ = false;
  fitPlane();
  // Only the fitted plane is retained and saved, not a background depth image.
  std::vector<double>().swap(mean_m_);
  std::vector<std::uint32_t>().swap(counts_);
  return true;
}

bool FloorReference::fitPlane()
{
  const auto & c = measurement_config_;
  std::vector<cv::Vec3d> points;
  for (int v = 0; v < camera_.height; ++v) {
    for (int u = 0; u < camera_.width; ++u) {
      const auto i = static_cast<std::size_t>(v) * camera_.width + u;
      if (counts_[i] < static_cast<std::uint32_t>(min_samples_)) { continue; }
      const double z = mean_m_[i];
      points.emplace_back(z, (camera_.cx - u) * z / camera_.fx, (camera_.cy - v) * z / camera_.fy);
    }
  }
  sample_points_ = points.size();
  const auto required = std::max(static_cast<std::size_t>(c.min_inlier_points),
    static_cast<std::size_t>(std::ceil(c.min_inlier_ratio * points.size())));
  if (points.size() < required) {
    failure_reason_ = "too few stable depth samples in floor measurement ROI";
    return false;
  }
  const double min_up = std::cos(c.max_tilt_deg * CV_PI / 180.0);
  const auto allowed = [&](const cv::Vec3d & n, double d) {
    return n[2] >= min_up && d >= c.min_camera_height_m && d <= c.max_camera_height_m;
  };
  const auto collect = [&](const cv::Vec3d & n, double d, double & squared_error) {
    std::vector<std::size_t> indices;
    squared_error = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      const double distance = n.dot(points[i]) + d;
      if (std::abs(distance) <= c.inlier_distance_m) {
        indices.push_back(i);
        squared_error += distance * distance;
      }
    }
    return indices;
  };
  std::mt19937 random(0x464c4f52U);
  std::uniform_int_distribution<std::size_t> pick(0, points.size() - 1);
  std::vector<std::size_t> best;
  double best_error = std::numeric_limits<double>::infinity();
  cv::Vec3d normal(0.0, 0.0, 1.0);
  double offset = 0.0;
  for (int iteration = 0; iteration < c.ransac_iterations; ++iteration) {
    const auto a = pick(random), b = pick(random), e = pick(random);
    if (a == b || a == e || b == e) { continue; }
    cv::Vec3d n = (points[b] - points[a]).cross(points[e] - points[a]);
    const double length = cv::norm(n);
    if (length < 1e-8) { continue; }
    n *= 1.0 / length;
    if (n[2] < 0.0) { n *= -1.0; }
    const double d = -n.dot(points[a]);
    if (!allowed(n, d)) { continue; }
    double error;
    auto indices = collect(n, d, error);
    if (indices.size() > best.size() || (indices.size() == best.size() && error < best_error)) {
      best = std::move(indices);
      best_error = error;
      normal = n;
      offset = d;
    }
  }
  if (best.size() < required) {
    failure_reason_ = "no floor plane satisfies inlier, tilt and camera-height limits";
    return false;
  }
  // Least-squares refinement of RANSAC inliers. Reject nearly collinear support.
  for (int pass = 0; pass < 2; ++pass) {
    cv::Vec3d center(0.0, 0.0, 0.0);
    for (const auto i : best) { center += points[i]; }
    center *= 1.0 / best.size();
    cv::Matx33d covariance = cv::Matx33d::zeros();
    for (const auto i : best) {
      const cv::Vec3d delta = points[i] - center;
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) { covariance(row, col) += delta[row] * delta[col]; }
      }
    }
    covariance *= 1.0 / best.size();
    cv::Mat eigenvalues, eigenvectors;
    if (!cv::eigen(cv::Mat(covariance), eigenvalues, eigenvectors)
        || eigenvalues.at<double>(1) < 0.0001) {
      failure_reason_ = "floor samples do not span a sufficient planar area";
      return false;
    }
    normal = cv::Vec3d(eigenvectors.at<double>(2, 0), eigenvectors.at<double>(2, 1),
      eigenvectors.at<double>(2, 2));
    normal *= 1.0 / cv::norm(normal);
    if (normal[2] < 0.0) { normal *= -1.0; }
    offset = -normal.dot(center);
    if (!allowed(normal, offset)) {
      failure_reason_ = "refined floor plane violates tilt or camera-height limits";
      return false;
    }
    best = collect(normal, offset, best_error);
    if (best.size() < required) {
      failure_reason_ = "refined floor plane has insufficient inliers";
      return false;
    }
  }
  rmse_m_ = std::sqrt(best_error / best.size());
  if (std::max(c.min_height_m, c.noise_scale * rmse_m_) >= c.max_height_m) {
    failure_reason_ = "floor fit noise exceeds the obstacle height band";
    return false;
  }
  plane_ = {{normal[0], normal[1], normal[2], offset}};
  inlier_points_ = best.size();
  updateGroundAxes();
  failure_reason_.clear();
  return true;
}

void FloorReference::updateGroundAxes()
{
  const cv::Vec3d n(plane_[0], plane_[1], plane_[2]);
  cv::Vec3d forward = cv::Vec3d(1.0, 0.0, 0.0) - n * n[0];
  forward *= 1.0 / cv::norm(forward);
  const cv::Vec3d left = n.cross(forward);
  for (int i = 0; i < 3; ++i) { forward_axis_[i] = forward[i]; left_axis_[i] = left[i]; }
}

std::array<double, 3> FloorReference::groundCoordinates(double forward, double left, double up) const
{
  const std::array<double, 3> point{{forward, left, up}};
  std::array<double, 3> result{{0.0, 0.0, plane_[3]}};
  // Origin is the perpendicular foot of the camera on the floor. Its dot
  // product with either tangential axis is zero, so no XY translation is needed.
  for (int i = 0; i < 3; ++i) {
    result[0] += forward_axis_[i] * point[i];
    result[1] += left_axis_[i] * point[i];
    result[2] += plane_[i] * point[i];
  }
  return result;
}

bool FloorReference::isObstacleHeight(double height_m, const FloorConfig & c) const
{
  return ready() && std::isfinite(height_m)
    && height_m >= std::max(c.min_height_m, c.noise_scale * rmse_m_) && height_m <= c.max_height_m;
}

void FloorReference::save(const std::string & path) const
{
  if (!ready()) { throw std::runtime_error("no valid floor plane to save"); }
  const std::filesystem::path target(path);
  if (!target.parent_path().empty()) { std::filesystem::create_directories(target.parent_path()); }
  const auto temporary = path + ".tmp";
  try {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) { throw std::runtime_error("cannot write floor plane: " + temporary); }
    stream.write("DLPLANE2", 8);
    writeValue(stream, std::uint32_t{0x01020304});
    writeValue(stream, static_cast<std::uint32_t>(camera_.width));
    writeValue(stream, static_cast<std::uint32_t>(camera_.height));
    for (double value : {camera_.fx, camera_.fy, camera_.cx, camera_.cy}) { writeValue(stream, value); }
    writeValue(stream, static_cast<std::uint32_t>(camera_.signature.size()));
    stream.write(camera_.signature.data(), static_cast<std::streamsize>(camera_.signature.size()));
    writeValue(stream, static_cast<std::uint32_t>(frames_));
    writeValue(stream, static_cast<std::uint32_t>(min_samples_));
    for (const auto value : plane_) { writeValue(stream, value); }
    writeValue(stream, rmse_m_);
    writeValue(stream, static_cast<std::uint32_t>(sample_points_));
    writeValue(stream, static_cast<std::uint32_t>(inlier_points_));
    stream.flush();
    if (!stream) { throw std::runtime_error("failed to write floor plane: " + path); }
    stream.close();
    if (!stream) { throw std::runtime_error("failed to close floor plane: " + path); }
    std::filesystem::rename(temporary, target);
  } catch (...) {
    std::error_code error;
    std::filesystem::remove(temporary, error);
    throw;
  }
}

bool FloorReference::load(const std::string & path, const CameraGeometry & camera)
{
  clear();
  if (!std::filesystem::exists(path)) { return false; }
  std::ifstream stream(path, std::ios::binary);
  char magic[8]{};
  stream.read(magic, 8);
  if (!stream || std::string(magic, 8) != "DLPLANE2") {
    throw std::runtime_error("unsupported floor file; press B to measure and save a ground plane");
  }
  std::uint32_t endian, width, height, length, frames, minimum;
  readValue(stream, endian);
  readValue(stream, width);
  readValue(stream, height);
  if (endian != 0x01020304 || width != static_cast<std::uint32_t>(camera.width)
      || height != static_cast<std::uint32_t>(camera.height) || !validCamera(camera)) { return false; }
  FloorReference candidate;
  candidate.camera_.width = static_cast<int>(width);
  candidate.camera_.height = static_cast<int>(height);
  readValue(stream, candidate.camera_.fx);
  readValue(stream, candidate.camera_.fy);
  readValue(stream, candidate.camera_.cx);
  readValue(stream, candidate.camera_.cy);
  readValue(stream, length);
  if (length > 4096U) { throw std::runtime_error("invalid floor camera signature length"); }
  candidate.camera_.signature.resize(length);
  stream.read(candidate.camera_.signature.data(), length);
  if (!stream || !candidate.compatible(camera)) { return false; }
  readValue(stream, frames);
  readValue(stream, minimum);
  if (frames < 2 || frames > 3600 || minimum < 2 || minimum > frames) {
    throw std::runtime_error("invalid floor measurement counts");
  }
  candidate.frames_ = candidate.target_frames_ = static_cast<int>(frames);
  candidate.min_samples_ = static_cast<int>(minimum);
  for (auto & value : candidate.plane_) { readValue(stream, value); }
  readValue(stream, candidate.rmse_m_);
  std::uint32_t samples, inliers;
  readValue(stream, samples);
  readValue(stream, inliers);
  const auto & p = candidate.plane_;
  const double norm = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-6
      || p[2] <= std::cos(85.0 * CV_PI / 180.0) || !std::isfinite(p[3]) || p[3] <= 0.0
      || !std::isfinite(candidate.rmse_m_) || candidate.rmse_m_ < 0.0
      || samples > static_cast<std::size_t>(width) * height || inliers < 3 || inliers > samples) {
    throw std::runtime_error("invalid saved floor plane or fit statistics");
  }
  candidate.sample_points_ = samples;
  candidate.inlier_points_ = inliers;
  candidate.updateGroundAxes();
  *this = std::move(candidate);
  return true;
}

DetectionResult detectForeground(const std::uint16_t * depth, std::size_t stride,
  const CameraGeometry & camera, const FloorReference & floor, const FloorConfig & floor_config,
  const ProjectionConfig & projection, const ClusterConfig & cluster)
{
  std::string reason;
  if (!validCamera(camera) || !depth || stride < static_cast<std::size_t>(camera.width)
      || !validateProjectionConfig(projection, reason) || !validateClusterConfig(cluster, reason)
      || !validateFloorConfig(floor_config, reason)) {
    throw std::invalid_argument("invalid foreground input: " + reason);
  }
  DetectionResult result;
  result.roi = computeRoi(camera.width, camera.height, projection.roi_width_ratio,
    projection.roi_height_ratio, projection.roi_bottom_offset_ratio);
  if (!floor.ready() || !floor.compatible(camera)) { return result; }
  const int step = projection.pixel_stride;
  const int cols = (result.roi.width + step - 1) / step;
  const int rows = (result.roi.height + step - 1) / step;
  std::vector<int> grid(static_cast<std::size_t>(cols) * rows, -1);
  std::vector<ForegroundPoint> candidates;
  candidates.reserve(grid.size());
  for (int row = 0; row < rows; ++row) {
    const int v = result.roi.y + row * step;
    for (int col = 0; col < cols; ++col) {
      const int u = result.roi.x + col * step;
      const auto raw = depth[static_cast<std::size_t>(v) * stride + u];
      if (!raw) { continue; }
      const double z = raw * 0.001;
      const auto ground = floor.groundCoordinates(z, (camera.cx - u) * z / camera.fx,
        (camera.cy - v) * z / camera.fy);
      if (ground[0] <= 0.0 || !floor.isObstacleHeight(ground[2], floor_config)) { continue; }
      const double range = std::hypot(ground[0], ground[1]);
      const double corrected = range + projection.range_offset_m;
      if (corrected < projection.min_range_m || corrected > projection.max_range_m) { continue; }
      const double scale = corrected / range;
      grid[static_cast<std::size_t>(row) * cols + col] = static_cast<int>(candidates.size());
      candidates.push_back({ground[0] * scale, ground[1] * scale, ground[2], u, v});
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
