#ifndef BEV_PROCESSOR__BEV_LANE_RECONSTRUCTOR_HPP_
#define BEV_PROCESSOR__BEV_LANE_RECONSTRUCTOR_HPP_

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
  int maximum_saturation{255};
  int brightness_blur_kernel{1};
  double vertical_close_m{0.05};
  double minimum_lane_mark_width_m{0.01};
  double maximum_lane_mark_width_m{0.18};
  int row_step_px{2};

  double observation_minimum_x_m{0.20};
  double observation_maximum_x_m{1.80};
  double reconstruction_minimum_x_m{0.20};
  double reconstruction_maximum_x_m{2.30};
  double maximum_extrapolation_m{0.50};

  double expected_lane_width_m{0.625};
  double lane_width_tolerance_m{0.20};
  double initial_center_tolerance_m{0.30};
  double single_lane_initial_tolerance_m{0.20};
  double maximum_lateral_step_m{0.08};
  double maximum_tracking_gap_m{0.20};
  int minimum_points{15};
  double maximum_fit_residual_m{0.06};
  bool allow_single_lane{true};

  double output_line_thickness_m{0.04};
  double temporal_smoothing_alpha{1.0};
  double maximum_temporal_jump_m{0.20};
};

struct LaneCurve
{
  bool valid{false};
  cv::Vec3d coefficients{0.0, 0.0, 0.0};
  int point_count{0};
  double minimum_observed_x_m{0.0};
  double maximum_observed_x_m{0.0};
  double rms_error_m{0.0};

  double lateralAt(double x_m) const;
  double derivativeAt(double x_m) const;
};

struct BevLaneReconstruction
{
  cv::Mat candidate_mask;
  cv::Mat reconstructed_mask;
  LaneCurve center_curve;
  bool valid{false};
  int center_point_count{0};
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
  LaneCurve previous_center_curve_;
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__BEV_LANE_RECONSTRUCTOR_HPP_
