#pragma once

#include "depth_lidar/depth_lidar_geometry.hpp"

#include <opencv2/core.hpp>

namespace depth_lidar
{

// Ground-aligned foreground points and cluster centers/footprints.
// Leaves the bottom 40 pixels available for receiver/processing metrics.
cv::Mat makeRadarPreview(const DetectionResult & detection, int width_px, double max_range_m);

// Draw the actual depth ROI scaled into rectified left/right images. Right is
// the depth reference; the left rectangle is a guide, not a correspondence.
cv::Mat makeStereoPreview(const cv::Mat & left, const cv::Mat & right,
  const RoiRect & depth_roi, int depth_width, int depth_height, const RoiRect * floor_roi = nullptr);

} // namespace depth_lidar
