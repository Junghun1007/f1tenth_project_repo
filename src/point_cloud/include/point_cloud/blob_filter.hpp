#pragma once

#include "point_cloud/obstacle_clusters.hpp"

namespace point_cloud
{
struct BlobOptions
{
  bool enabled{true};
  double cell_size_m{0.01};
  int closing_radius_cells{1};
  int opening_radius_cells{1};
  double min_area_m2{0.0009};
  double min_thickness_m{0.03};
  double min_fill_ratio{0.35};
  double max_aspect_ratio{6.0};
};
inline void validateBlobs(const BlobOptions & o)
{
  for (const double value : {o.cell_size_m, o.min_area_m2, o.min_thickness_m,
    o.min_fill_ratio, o.max_aspect_ratio})
  {
    if (!std::isfinite(value)) {throw std::invalid_argument("blob: all values must be finite");}
  }
  if (o.cell_size_m < 0.005 || o.cell_size_m > 0.05 || o.closing_radius_cells < 0 ||
    o.closing_radius_cells > 5 || o.opening_radius_cells < 0 || o.opening_radius_cells > 5 ||
    o.min_area_m2 <= 0 || o.min_thickness_m <= 0 ||
    o.min_fill_ratio <= 0 || o.min_fill_ratio > 1 || o.max_aspect_ratio < 1)
  {
    throw std::invalid_argument("blob: cell_size_m=0.005..0.05, morphology radii=0..5 cells, "
      "min_area/thickness>0, fill_ratio=(0,1], max_aspect_ratio>=1");
  }
}
struct BlobResult
{
  ClusterResult clusters;
  std::size_t grid_limit_rejections{0};
};

// Clean each cluster's BEV occupancy, split at removed thin connections, and
// validate each resulting component. Output contains only original 3D points.
BlobResult filterBlobs(const ClusterResult & input, const BlobOptions & options,
  const ClusterOptions & cluster_options);
}  // namespace point_cloud
