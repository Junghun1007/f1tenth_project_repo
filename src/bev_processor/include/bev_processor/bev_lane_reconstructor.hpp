#ifndef BEV_PROCESSOR__BEV_LANE_RECONSTRUCTOR_HPP_
#define BEV_PROCESSOR__BEV_LANE_RECONSTRUCTOR_HPP_

#include <array>
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
  double output_lateral_margin_m{0.70};
  double meter_per_pixel{0.01};
  int image_width{120};
  int image_height{300};

  int minimum_brightness{160};
  int far_minimum_brightness{125};
  int maximum_saturation{80};
  int brightness_blur_kernel{1};
  double vertical_close_m{0.05};
  double minimum_lane_mark_width_m{0.01};
  double maximum_lane_mark_width_m{0.08};
  int minimum_local_contrast{35};
  int maximum_local_background_brightness{140};
  double local_background_band_m{0.05};
  double tracked_lane_mark_width_near_m{0.11};
  double tracked_lane_mark_width_far_m{0.20};
  double measurement_lateral_gate_near_m{0.05};
  double measurement_lateral_gate_far_m{0.07};
  int row_step_px{2};

  double observation_minimum_x_m{0.20};
  // This is a confidence boundary, not a hard observation cut. Pixels past
  // it are searched with a wider window and the far brightness threshold.
  double observation_maximum_x_m{1.80};
  double reconstruction_minimum_x_m{0.20};
  double reconstruction_maximum_x_m{2.70};
  double maximum_extrapolation_m{0.0};

  double sliding_window_step_m{0.06};
  double sliding_window_length_m{0.18};
  double sliding_window_half_width_near_m{0.08};
  double sliding_window_half_width_far_m{0.10};
  double sliding_window_measurement_weight{0.88};
  double sliding_window_heading_weight{0.50};
  double maximum_tracking_arc_length_m{3.20};
  double maximum_gap_fill_m{0.26};
  double measured_point_smoothing_weight{0.50};
  int minimum_window_pixel_count{6};

  double expected_lane_width_m{0.80};
  double lane_width_tolerance_m{0.10};
  double initial_center_tolerance_m{0.30};
  double single_lane_initial_tolerance_m{0.20};
  double maximum_tracking_gap_m{0.08};
  int minimum_points{6};
  bool allow_single_lane{true};

  bool infer_partially_missing_lane{true};
  bool inference_preserve_reference_shape{true};
  double inference_tangent_window_m{0.20};
  double inference_maximum_curvature_per_m{1.25};
  double inference_maximum_heading_step_deg{8.0};

  bool temporal_tracking_enabled{true};
  double temporal_maximum_lateral_jump_near_m{0.06};
  double temporal_maximum_lateral_jump_far_m{0.12};
  double temporal_maximum_heading_jump_deg{15.0};
  int temporal_confirmation_frames{4};
  int temporal_hold_frames{5};

  double output_line_thickness_m{0.02};
};

struct BevLaneReconstruction
{
  cv::Mat candidate_mask;
  cv::Mat reconstructed_mask;
  cv::Mat left_reconstructed_mask;
  cv::Mat right_reconstructed_mask;
  std::vector<cv::Point2d> left_measured_points;
  std::vector<cv::Point2d> right_measured_points;
  bool valid{false};
  int measured_point_count{0};
  int inferred_point_count{0};
  bool temporal_hold_used{false};
  double measured_lane_width_m{0.0};
  double reconstructed_maximum_x_m{0.0};

  struct SlidingWindow
  {
    std::array<cv::Point2d, 4> corners;
    bool left_lane{false};
    bool measurement_found{false};
  };
  std::vector<SlidingWindow> sliding_windows;
};

class BevLaneReconstructor
{
public:
  explicit BevLaneReconstructor(BevLaneReconstructorConfig config);

  BevLaneReconstruction reconstruct(const cv::Mat & bev_bgr);
  void reset();

private:
  BevLaneReconstructorConfig config_;
  std::vector<cv::Point2d> accepted_left_points_;
  std::vector<cv::Point2d> accepted_right_points_;
  std::vector<cv::Point2d> pending_left_points_;
  std::vector<cv::Point2d> pending_right_points_;
  int pending_left_frames_{0};
  int pending_right_frames_{0};
  int held_left_frames_{0};
  int held_right_frames_{0};
  double accepted_lane_width_m_{0.0};
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__BEV_LANE_RECONSTRUCTOR_HPP_
