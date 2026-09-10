#ifndef LINE_DETACTOR__LANE_CONNECTOR_HPP_
#define LINE_DETACTOR__LANE_CONNECTOR_HPP_

#include <array>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace line_detactor
{

struct LaneConnectionConfig
{
  bool enabled{true};
  int padding_px{30};  // Per side; does not change model input or BEV scale.
  int min_component_area_px{8};
  double min_fragment_length_px{8.0};
  int max_fragments{24};
  double tangent_window_px{8.0};
  double max_gap_px{80.0};
  double corridor_half_width_px{4.0};
  double direction_tolerance_deg{20.0};
  double max_turn_deg{180.0};
  double max_curvature_per_px{0.12};
  double max_arc_ratio{1.8};
  double border_endpoint_distance_px{6.0};
  int line_width_px{2};
};

struct ConnectedLane
{
  std::vector<cv::Point2f> points;  // Extended-image diagnostic segments; no global ordering.
  std::vector<std::uint32_t> segment_starts;  // Never join across these offsets.
  std::vector<std::uint8_t> interpolated;  // 0=model-supported, 1=bridge.
  double observed_length_px{0.0};
};

struct LaneConnectionResult
{
  std::array<ConnectedLane, 2> lanes;
  cv::Mat labels;  // mono8: 0=background, 1/2=left/right model, 3/4=left/right bridge.
  cv::Mat image;   // bgr8: lanes/bridges; the node composites stop lines in green afterward.
  cv::Mat stop_line_mask;  // Independent mono8 0/255; original ROI only, no lane cleanup/bridges.
  std::uint8_t state{0U};  // 0=NONE, 1=LEFT_ONLY, 2=RIGHT_ONLY, 3=BOTH.
};

void validate_lane_connection(const LaneConnectionConfig & config);
LaneConnectionResult connect_lane_fragments(
  const cv::Mat & labels, const LaneConnectionConfig & config);

}  // namespace line_detactor

#endif
