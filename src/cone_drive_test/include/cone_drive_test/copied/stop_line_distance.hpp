// Copied from src/line_detactor/include/line_detactor/stop_line_distance.hpp (origin/1113ML)
#ifndef CONE_DRIVE_TEST__STOP_LINE_DISTANCE_HPP_
#define CONE_DRIVE_TEST__STOP_LINE_DISTANCE_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace cone_drive_test
{

struct StopLineDistanceConfig
{
  bool enabled{true};
  int min_pixels{6};
  int max_fit_samples{256};
  double support_trim_quantile{0.02};
  double support_margin_px{5.0};
  double near_edge_quantile{0.05};
  double min_crossing_alignment{0.5};  // Absolute path dot line-normal: sin(crossing angle).
  double fit_distance_tolerance_px{0.01};
  double fit_angle_tolerance_rad{0.01};
};
void validate_stop_line_distance(const StopLineDistanceConfig & config);

// Returns front-axle-to-near-edge distance along centerline, or NaN when the
// fitted stop line and generated path do not form a reliable intersection.
double estimate_stop_line_distance_m(
  const cv::Mat & stop_mask,
  const std::vector<cv::Point2f> & centerline,
  int padding_left,
  double bev_width_m,
  double bev_height_m,
  const StopLineDistanceConfig & config = StopLineDistanceConfig{});

}  // namespace cone_drive_test

#endif  // CONE_DRIVE_TEST__STOP_LINE_DISTANCE_HPP_
