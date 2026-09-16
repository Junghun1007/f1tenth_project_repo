#include "depth_lidar/depth_lidar_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
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
  if (c.measure_frames < 2 || c.measure_frames > 3600
      || !std::isfinite(c.min_valid_ratio) || c.min_valid_ratio <= 0.0 || c.min_valid_ratio > 1.0
      || !std::isfinite(c.min_delta_m) || c.min_delta_m <= 0.0
      || !std::isfinite(c.noise_scale) || c.noise_scale < 0.0) {
    reason = "floor requires frames [2,3600], valid_ratio (0,1], delta > 0, noise_scale >= 0";
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
  target_frames_ = config.measure_frames;
  min_samples_ = std::max(2, static_cast<int>(std::ceil(target_frames_ * config.min_valid_ratio)));
  const auto size = static_cast<std::size_t>(camera.width) * camera.height;
  mean_m_.assign(size, 0.0);
  m2_.assign(size, 0.0);
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
  // Learn the entire frame, independent of detection ROI/range limits. This
  // allows ROI tuning and detecting an object in front of a more distant floor.
  for (int v = 0; v < camera_.height; ++v) {
    for (int u = 0; u < camera_.width; ++u) {
      const auto raw = depth[static_cast<std::size_t>(v) * stride + u];
      if (raw == 0U) { continue; }
      const auto i = static_cast<std::size_t>(v) * camera_.width + u;
      const double value = raw * 0.001;
      const auto count = ++counts_[i];
      const double delta = value - mean_m_[i];
      mean_m_[i] += delta / count;
      m2_[i] += delta * (value - mean_m_[i]);
    }
  }
  if (++frames_ < target_frames_) { return false; }
  measuring_ = false;
  for (const auto count : counts_) {
    if (count >= static_cast<std::uint32_t>(min_samples_)) { ++valid_pixels_; }
  }
  return true;
}

bool FloorReference::isForeground(std::size_t i, double depth_m, const FloorConfig & c) const
{
  if (!ready() || i >= counts_.size() || counts_[i] < static_cast<std::uint32_t>(min_samples_)) {
    return false;
  }
  const double sigma = std::sqrt(std::max(0.0, m2_[i] / (counts_[i] - 1U)));
  return mean_m_[i] - depth_m > std::max(c.min_delta_m, c.noise_scale * sigma);
}

void FloorReference::save(const std::string & path) const
{
  if (!ready()) { throw std::runtime_error("no valid floor reference to save"); }
  const std::filesystem::path target(path);
  if (!target.parent_path().empty()) { std::filesystem::create_directories(target.parent_path()); }
  const auto temporary = path + ".tmp";
  try {
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream) { throw std::runtime_error("cannot write floor reference: " + temporary); }
    stream.write("DLFLOOR1", 8);
    writeValue(stream, std::uint32_t{0x01020304});
    writeValue(stream, static_cast<std::uint32_t>(camera_.width));
    writeValue(stream, static_cast<std::uint32_t>(camera_.height));
    for (double value : {camera_.fx, camera_.fy, camera_.cx, camera_.cy}) { writeValue(stream, value); }
    writeValue(stream, static_cast<std::uint32_t>(camera_.signature.size()));
    stream.write(camera_.signature.data(), static_cast<std::streamsize>(camera_.signature.size()));
    writeValue(stream, static_cast<std::uint32_t>(frames_));
    writeValue(stream, static_cast<std::uint32_t>(min_samples_));
    for (std::size_t i = 0; i < counts_.size(); ++i) {
      writeValue(stream, mean_m_[i]);
      writeValue(stream, m2_[i]);
      writeValue(stream, counts_[i]);
    }
    stream.flush();
    if (!stream) { throw std::runtime_error("failed to write floor reference: " + path); }
    stream.close();
    if (!stream) { throw std::runtime_error("failed to close floor reference: " + path); }
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
  if (!stream || std::string(magic, 8) != "DLFLOOR1") { throw std::runtime_error("invalid floor file header"); }
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
  const auto size = static_cast<std::size_t>(width) * height;
  candidate.mean_m_.resize(size);
  candidate.m2_.resize(size);
  candidate.counts_.resize(size);
  for (std::size_t i = 0; i < size; ++i) {
    readValue(stream, candidate.mean_m_[i]);
    readValue(stream, candidate.m2_[i]);
    readValue(stream, candidate.counts_[i]);
    if (!std::isfinite(candidate.mean_m_[i]) || candidate.mean_m_[i] < 0.0
        || candidate.mean_m_[i] > 65.535 || !std::isfinite(candidate.m2_[i]) || candidate.m2_[i] < -1e-9
        || candidate.counts_[i] > frames
        || (candidate.counts_[i] > 0U && candidate.mean_m_[i] <= 0.0)) {
      throw std::runtime_error("invalid floor pixel statistics");
    }
    if (candidate.counts_[i] >= minimum) { ++candidate.valid_pixels_; }
  }
  if (!candidate.ready()) { return false; }
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
      if (!raw || !floor.isForeground(static_cast<std::size_t>(v) * camera.width + u,
          raw * 0.001, floor_config)) { continue; }
      const double forward = raw * 0.001;
      const double left = (camera.cx - u) * forward / camera.fx;
      const double up = (camera.cy - v) * forward / camera.fy;
      const double range = std::hypot(forward, left);
      const double corrected = range + projection.range_offset_m;
      if (corrected < projection.min_range_m || corrected > projection.max_range_m) { continue; }
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
