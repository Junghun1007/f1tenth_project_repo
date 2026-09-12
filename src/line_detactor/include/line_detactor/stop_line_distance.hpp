#ifndef LINE_DETACTOR__STOP_LINE_DISTANCE_HPP_
#define LINE_DETACTOR__STOP_LINE_DISTANCE_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace line_detactor
{

// Returns front-axle-to-near-edge distance along centerline, or NaN when the
// fitted stop line and generated path do not form a reliable intersection.
double estimate_stop_line_distance_m(
  const cv::Mat & stop_mask,
  const std::vector<cv::Point2f> & centerline,
  int padding_left,
  double bev_width_m,
  double bev_height_m);

}  // namespace line_detactor

#endif  // LINE_DETACTOR__STOP_LINE_DISTANCE_HPP_
