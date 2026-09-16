#pragma once

#include "depth_lidar/depth_lidar_geometry.hpp"

#include <opencv2/core.hpp>

namespace depth_lidar
{

// Fixed vehicle-frame scan points; orange=current, gray=briefly held.
// Leaves the bottom 40 pixels available for receiver/processing metrics.
cv::Mat makeRadarPreview(const ScanResult & scan, const ProjectionConfig & config, int width_px, bool ready = true);

// Draw the actual depth ROI scaled into rectified left/right images. Right is
// the depth reference; the left rectangle is a guide, not a correspondence.
cv::Mat makeStereoPreview(const cv::Mat & left, const cv::Mat & right,
  const RoiRect & depth_roi, int depth_width, int depth_height);

} // namespace depth_lidar
