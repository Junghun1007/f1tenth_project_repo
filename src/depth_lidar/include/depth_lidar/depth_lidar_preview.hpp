#pragma once

#include "depth_lidar/depth_lidar_geometry.hpp"

#include <opencv2/core.hpp>

namespace depth_lidar
{

// Camera-origin polar scan; no vehicle pose, BEV bounds, or obstacle inflation.
// Leaves the bottom 40 pixels available for receiver/processing metrics.
cv::Mat makeRadarPreview(const ScanProjection & projection, int width_px, double max_range_m);

} // namespace depth_lidar
