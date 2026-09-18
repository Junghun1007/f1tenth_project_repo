#pragma once

#include "point_cloud/depth_projection.hpp"

#include <array>
#include <deque>
#include <unordered_map>

namespace point_cloud
{
struct TemporalOptions
{
  bool enabled{false};
  double voxel_size_m{0.04};
  double match_distance_m{0.04};
  int window_frames{5};
  int min_hits{4};
  double max_age_sec{0.25};
};
inline void validateTemporal(const TemporalOptions & o)
{
  if (!std::isfinite(o.voxel_size_m) || o.voxel_size_m < 0.005 || o.voxel_size_m > 0.2 ||
    !std::isfinite(o.match_distance_m) || o.match_distance_m <= 0 ||
    o.match_distance_m > 2 * o.voxel_size_m || o.window_frames < 1 || o.window_frames > 30 ||
    o.min_hits < 1 || o.min_hits > o.window_frames ||
    !std::isfinite(o.max_age_sec) || o.max_age_sec < 0.02 || o.max_age_sec > 2.0)
  {
    throw std::invalid_argument("temporal: voxel_size_m=0.005..0.2, match_distance_m=(0,2*voxel_size], "
      "window_frames=1..30, min_hits=1..window_frames, max_age_sec=0.02..2");
  }
}

// Frame history contains occupied 3D voxel centroids, not accumulated output
// points. Only points observed NOW can survive. Coordinates must share a fixed
// reference across frames: currently this assumes a stationary vehicle/mount.
class TemporalFilter
{
  struct Key
  {
    int x, y, z;
    bool operator==(const Key & other) const {return x == other.x && y == other.y && z == other.z;}
  };
  struct Hash
  {
    std::size_t operator()(const Key & k) const
    {
      std::size_t h = std::hash<int>{}(k.x);
      for (int v : {k.y, k.z}) {h ^= std::hash<int>{}(v) + 0x9e3779b9U + (h << 6) + (h >> 2);}
      return h;
    }
  };
  struct Voxel
  {
    std::array<double, 3> center{0,0,0};
    std::size_t count{0};
    bool keep{false};
  };
  using Voxels = std::unordered_map<Key, Voxel, Hash>;
  struct Frame {double stamp; Voxels voxels;};

public:
  void clear()
  {
    history_.clear();
    last_stamp_ = -std::numeric_limits<double>::infinity();
  }

  // Return surviving points; invalid/out-of-height points are masked as well.
  // Each previous frame contributes at most one hit regardless of point density.
  std::size_t apply(Cloud & cloud, double stamp, const TemporalOptions & o,
    double min_height, double max_height)
  {
    validateTemporal(o);
    if (!std::isfinite(stamp) || !std::isfinite(min_height) || !std::isfinite(max_height) ||
      min_height > max_height) {throw std::invalid_argument("Invalid temporal timestamp/height bounds");}
    const bool changed = o.enabled != options_.enabled || o.voxel_size_m != options_.voxel_size_m ||
      o.match_distance_m != options_.match_distance_m || o.window_frames != options_.window_frames ||
      o.min_hits != options_.min_hits || o.max_age_sec != options_.max_age_sec ||
      min_height != min_height_ || max_height != max_height_;
    if (changed) {clear();}
    options_ = o; min_height_ = min_height; max_height_ = max_height;
    if (!o.enabled) {clear(); return cloud.valid_points;}
    if (stamp < last_stamp_) {clear();}
    if (stamp == last_stamp_) {
      // Replayed frames must not supply additional evidence or output.
      for (auto & value : cloud.xyz) {value = std::numeric_limits<float>::quiet_NaN();}
      cloud.valid_points = 0;
      return 0;
    }
    last_stamp_ = stamp;
    while (!history_.empty() && (stamp - history_.front().stamp > o.max_age_sec ||
      history_.size() >= static_cast<std::size_t>(o.window_frames))) {history_.pop_front();}
    Voxels current;
    auto keyFor = [&](std::size_t j, Key & key) {
        std::array<int,3> values{};
        for (int axis = 0; axis < 3; ++axis) {
          const double value = cloud.xyz[j + axis];
          if (!std::isfinite(value)) {return false;}
          const double grid = std::floor(value / o.voxel_size_m);
          if (std::abs(grid) > 1e9) {return false;}
          values[axis] = static_cast<int>(grid);
        }
        if (cloud.xyz[j+2] < min_height || cloud.xyz[j+2] > max_height) {return false;}
        key = {values[0],values[1],values[2]};
        return true;
      };
    for (std::size_t j = 0; j < cloud.xyz.size(); j += 3) {
      Key key{};
      if (!keyFor(j,key)) {continue;}
      auto & voxel = current[key];
      for (int a = 0; a < 3; ++a) {voxel.center[a] += cloud.xyz[j+a];}
      ++voxel.count;
    }
    for (auto & entry : current) {
      for (auto & value : entry.second.center) {value /= entry.second.count;}
    }
    const int radius = static_cast<int>(std::ceil(o.match_distance_m / o.voxel_size_m));
    const double distance2 = o.match_distance_m * o.match_distance_m;
    auto matches = [&](const Voxels & previous, const Key & key, const Voxel & voxel) {
        auto close = [&](const Key & neighbor) {
            const auto found = previous.find(neighbor);
            if (found == previous.end()) {return false;}
            double squared = 0;
            for (int a = 0; a < 3; ++a) {
              const double d = voxel.center[a] - found->second.center[a]; squared += d*d;
            }
            return squared <= distance2;
          };
        if (close(key)) {return true;}
        for (int x = -radius; x <= radius; ++x) {
          for (int y = -radius; y <= radius; ++y) {
            for (int z = -radius; z <= radius; ++z) {
              if (x == 0 && y == 0 && z == 0) {continue;}
              if (close({key.x+x,key.y+y,key.z+z})) {return true;}
            }
          }
        }
        return false;
      };
    for (auto & entry : current) {
      int hits = 1;  // Current frame counts once, never once per point.
      if (o.min_hits <= static_cast<int>(history_.size()) + 1) {
        for (auto it = history_.rbegin(); it != history_.rend() && hits < o.min_hits; ++it) {
          if (matches(it->voxels,entry.first,entry.second)) {++hits;}
        }
      }
      entry.second.keep = hits >= o.min_hits;
    }
    cloud.valid_points = 0;
    for (std::size_t j = 0; j < cloud.xyz.size(); j += 3) {
      Key key{};
      if (keyFor(j,key) && current.at(key).keep) {++cloud.valid_points;}
      else {for (int a = 0; a < 3; ++a) {cloud.xyz[j+a] = std::numeric_limits<float>::quiet_NaN();}}
    }
    // Store observations before masking, including unconfirmed voxels, so they
    // can become confirmed later. History is bounded by both frame count and age.
    history_.push_back(Frame{stamp,std::move(current)});
    return cloud.valid_points;
  }

private:
  TemporalOptions options_;
  double min_height_{0}, max_height_{0};
  double last_stamp_{-std::numeric_limits<double>::infinity()};
  std::deque<Frame> history_;
};
}  // namespace point_cloud
