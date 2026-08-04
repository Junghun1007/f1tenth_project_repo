#ifndef BEV_PROCESSOR__BEV_LANE_RECONSTRUCTOR_HPP_
#define BEV_PROCESSOR__BEV_LANE_RECONSTRUCTOR_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace bev_processor
{

struct BevLaneReconstructorConfig
{
  double x_min_m{0.0};
  double x_max_m{3.0};
  double y_min_m{-0.6};
  double y_max_m{0.6};
  double meter_per_pixel{0.01};
  int image_width{120};
  int image_height{300};

  int minimum_brightness{160};
  int far_minimum_brightness{110};
  int maximum_saturation{100};
  int brightness_blur_kernel{1};
  double vertical_close_m{0.05};
  double minimum_lane_mark_width_m{0.01};
  double maximum_lane_mark_width_m{0.10};
  int minimum_local_contrast{25};
  int maximum_local_background_brightness{170};
  double local_background_band_m{0.05};
  double tracked_lane_mark_width_near_m{0.11};
  double tracked_lane_mark_width_far_m{0.20};
  double measurement_lateral_gate_near_m{0.08};
  double measurement_lateral_gate_far_m{0.18};
  int row_step_px{2};

  double observation_minimum_x_m{0.20};
  // This is a confidence boundary, not a hard observation cut. Pixels past
  // it are searched with a wider window and the far brightness threshold.
  double observation_maximum_x_m{1.80};
  double reconstruction_minimum_x_m{0.20};
  double reconstruction_maximum_x_m{2.70};
  double maximum_extrapolation_m{0.20};

  double sliding_window_step_m{0.06};
  double sliding_window_length_m{0.18};
  double sliding_window_half_width_near_m{0.12};
  double sliding_window_half_width_far_m{0.22};
  double sliding_window_measurement_weight{0.90};
  double sliding_window_heading_weight{0.80};
  double maximum_tracking_arc_length_m{3.20};
  double maximum_gap_fill_m{0.26};
  double measured_point_smoothing_weight{0.85};
  int minimum_window_pixel_count{6};

  double expected_lane_width_m{0.625};
  double lane_width_tolerance_m{0.20};
  double initial_center_tolerance_m{0.30};
  double single_lane_initial_tolerance_m{0.20};
  double maximum_tracking_gap_m{0.20};
  int minimum_points{5};
  bool allow_single_lane{true};

  double output_line_thickness_m{0.02};
};

struct BevLaneReconstruction
{
  cv::Mat candidate_mask;
  cv::Mat reconstructed_mask;
  std::vector<cv::Point2d> left_measured_points;
  std::vector<cv::Point2d> right_measured_points;
  bool valid{false};
  int measured_point_count{0};
  double measured_lane_width_m{0.0};
  double reconstructed_maximum_x_m{0.0};
};

class BevLaneReconstructor
{
public:
  explicit BevLaneReconstructor(BevLaneReconstructorConfig config);

  BevLaneReconstruction reconstruct(const cv::Mat & bev_bgr);
  void reset();

private:
  BevLaneReconstructorConfig config_;
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__BEV_LANE_RECONSTRUCTOR_HPP_
