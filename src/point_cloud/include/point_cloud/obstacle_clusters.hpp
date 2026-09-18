#pragma once

#include "point_cloud/depth_projection.hpp"

#include <algorithm>
#include <array>
#include <unordered_map>

namespace point_cloud
{
struct ClusterOptions
{
  double cell_size_m{0.02};
  double tolerance_m{0.08};
  int min_points{20};
  double min_height_m{0.03};
  double max_height_m{2.0};
  double support_height_m{0.06};
  int min_support_points{5};
  double min_support_ratio{0.30};
  double min_extent_m{0.03};
};

inline void validateClusters(const ClusterOptions & o)
{
  for (const double value : {o.cell_size_m, o.tolerance_m, o.min_height_m,
    o.max_height_m, o.support_height_m, o.min_support_ratio, o.min_extent_m})
  {
    if (!std::isfinite(value)) {throw std::invalid_argument("cluster: all values must be finite");}
  }
  if (o.cell_size_m < 0.005 || o.cell_size_m > 0.2 ||
    o.tolerance_m < o.cell_size_m || o.tolerance_m > 0.5 ||
    o.tolerance_m / o.cell_size_m > 10.0 || o.min_points < 1 || o.min_points > 1000000 ||
    o.min_height_m < 0 || o.max_height_m <= o.min_height_m ||
    o.support_height_m < o.min_height_m || o.support_height_m > o.max_height_m ||
    o.min_support_points < 1 || o.min_support_points > 1000000 ||
    o.min_support_ratio <= 0 || o.min_support_ratio > 1 || o.min_extent_m < 0)
  {
    throw std::invalid_argument("cluster: cell_size_m=0.005..0.2, tolerance_m=cell_size..min(0.5,10*cell_size), "
      "point counts=1..1000000, 0<=min_height<max_height, support_height within height range, "
      "support_ratio=(0,1], min_extent>=0");
  }
}

struct ClusterResult
{
  Cloud points;  // Compact original points, never voxel centroids.
  std::vector<std::uint32_t> ids;  // Frame-local labels, starting at 1. Not track IDs.
  std::size_t candidates{0}, accepted{0};
};

// XY cell aggregation bounds work for dense stereo clouds. Connect neighboring
// occupied cells by Euclidean distance between their XY centroids, then classify
// each connected component using original-point counts, height support and size.
// Connectivity is approximate at cell_size_m resolution; there is no time state.
inline ClusterResult obstacleClusters(const Cloud & cloud, const ClusterOptions & o)
{
  validateClusters(o);
  struct Cell
  {
    int gx, gy;
    double sum_x{0}, sum_y{0};
    double min_x{std::numeric_limits<double>::infinity()}, max_x{-min_x};
    double min_y{std::numeric_limits<double>::infinity()}, max_y{-min_y};
    std::size_t count{0}, support{0};
    bool visited{false};
    std::uint32_t label{0};
  };
  auto key = [](int x, int y) {
      return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
             static_cast<std::uint32_t>(y);
    };
  std::unordered_map<std::uint64_t, std::size_t> grid;
  std::vector<Cell> cells;
  const auto count = cloud.xyz.size() / 3;
  const auto none = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> membership(count, none);
  for (std::size_t i = 0; i < count; ++i) {
    const double x = cloud.xyz[i*3], y = cloud.xyz[i*3+1], z = cloud.xyz[i*3+2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      z < o.min_height_m || z > o.max_height_m) {continue;}
    const double sx = std::floor(x / o.cell_size_m), sy = std::floor(y / o.cell_size_m);
    // Reject nonsensical coordinates before integer conversion/neighbor offsets.
    if (std::abs(sx) > 1e9 || std::abs(sy) > 1e9) {continue;}
    const int gx = static_cast<int>(sx), gy = static_cast<int>(sy);
    const auto entry = grid.emplace(key(gx, gy), cells.size());
    if (entry.second) {cells.push_back(Cell{gx, gy});}
    const auto index = entry.first->second;
    membership[i] = index;
    auto & cell = cells[index];
    cell.sum_x += x; cell.sum_y += y; ++cell.count;
    if (z >= o.support_height_m) {++cell.support;}
    cell.min_x = std::min(cell.min_x, x); cell.max_x = std::max(cell.max_x, x);
    cell.min_y = std::min(cell.min_y, y); cell.max_y = std::max(cell.max_y, y);
  }
  ClusterResult result;
  result.points.height = 1;
  // +1 handles centroids close across cell boundaries, including exact multiples.
  const int radius = static_cast<int>(std::ceil(o.tolerance_m / o.cell_size_m)) + 1;
  const double threshold2 = o.tolerance_m * o.tolerance_m;
  std::vector<std::size_t> queue;
  for (std::size_t seed = 0; seed < cells.size(); ++seed) {
    if (cells[seed].visited) {continue;}
    ++result.candidates;
    queue.clear(); queue.push_back(seed); cells[seed].visited = true;
    std::size_t points = 0, support = 0;
    double min_x = std::numeric_limits<double>::infinity(), max_x = -min_x;
    double min_y = min_x, max_y = -min_x;
    for (std::size_t head = 0; head < queue.size(); ++head) {
      const auto & cell = cells[queue[head]];
      points += cell.count; support += cell.support;
      min_x = std::min(min_x, cell.min_x); max_x = std::max(max_x, cell.max_x);
      min_y = std::min(min_y, cell.min_y); max_y = std::max(max_y, cell.max_y);
      const double cx = cell.sum_x / cell.count, cy = cell.sum_y / cell.count;
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          const auto found = grid.find(key(cell.gx + dx, cell.gy + dy));
          if (found == grid.end()) {continue;}
          auto & neighbor = cells[found->second];
          if (neighbor.visited) {continue;}
          const double nx = neighbor.sum_x / neighbor.count - cx;
          const double ny = neighbor.sum_y / neighbor.count - cy;
          if (nx*nx + ny*ny <= threshold2) {
            neighbor.visited = true;
            queue.push_back(found->second);
          }
        }
      }
    }
    if (points < static_cast<std::size_t>(o.min_points) ||
      support < static_cast<std::size_t>(o.min_support_points) ||
      static_cast<double>(support) / points < o.min_support_ratio ||
      std::max(max_x - min_x, max_y - min_y) < o.min_extent_m) {continue;}
    ++result.accepted;
    for (const auto index : queue) {cells[index].label = static_cast<std::uint32_t>(result.accepted);}
  }
  for (std::size_t i = 0; i < count; ++i) {
    if (membership[i] == none) {continue;}
    const auto label = cells[membership[i]].label;
    if (label == 0) {continue;}
    result.points.xyz.insert(result.points.xyz.end(), cloud.xyz.begin() + i*3, cloud.xyz.begin() + i*3 + 3);
    result.ids.push_back(label);
  }
  result.points.width = static_cast<std::uint32_t>(result.ids.size());
  result.points.valid_points = result.ids.size();
  return result;
}
}  // namespace point_cloud
