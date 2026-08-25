#ifndef BEV_PROCESSOR__BEV_LANE_SEED_DETECTOR_HPP_
#define BEV_PROCESSOR__BEV_LANE_SEED_DETECTOR_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace bev_processor
{

struct BevLaneSeedDetectorConfig
{
  int image_width{120};
  int image_height{300};

  double roi_bottom_exclusion_ratio{0.09};
  double roi_height_ratio{0.25};
  int minimum_top_hat_response{30};
  int minimum_run_width_px{2};
  int maximum_run_width_px{8};
  double maximum_lateral_step_px{4.0};
  int maximum_gap_rows{4};

  int background_gap_px{1};
  int background_band_width_px{5};
  double minimum_bilateral_contrast{25.0};
  double maximum_background_asymmetry{50.0};
  bool contrast_relaxation_enabled{true};
  double contrast_relaxation_step{5.0};
  int contrast_relaxation_retry_count{5};

  double minimum_track_arc_length_px{20.0};
  double contrast_score_weight{0.30};

  bool slope_filter_enabled{true};
  int slope_median_window{5};
  double maximum_slope_change_px_per_row{2.0};

  double minimum_pair_distance_px{50.0};
  double maximum_pair_distance_px{95.0};

  // Extend an already accepted seed track toward the far end of the BEV.
  // Each rotated window emits one intensity-weighted centroid point.
  bool sliding_window_enabled{true};
  double sliding_window_minimum_seed_arc_length_px{35.0};
  int sliding_window_initial_width_px{6};
  int sliding_window_initial_height_px{10};
  double sliding_window_growth_ratio{1.04};
  int sliding_window_maximum_width_px{25};
  int sliding_window_maximum_height_px{15};
  double sliding_window_step_ratio{0.60};
  int sliding_window_maximum_count{24};
  int sliding_window_minimum_bright_pixels{2};
  int sliding_window_maximum_consecutive_misses{1};
  double sliding_window_maximum_turn_deg_per_window{14.0};
  double sliding_window_maximum_turn_change_deg_per_window{3.0};
  double sliding_window_heading_update_gain{0.80};

  // Convert the selected boundaries into one drive centerline. Pair frames
  // update the measured road width and per-side offset bias; a single
  // boundary is shifted along its local normal by half that width.
  bool centerline_enabled{true};
  double centerline_nominal_lane_width_px{62.0};
  double centerline_width_update_gain{0.10};
  double centerline_resample_spacing_px{2.0};
  double centerline_outlier_distance_px{4.0};
  double centerline_fit_segment_length_px{12.0};
  double centerline_fit_tail_window_px{8.0};
  double centerline_fit_straight_maximum_residual_px{1.5};
  double centerline_fit_straight_maximum_heading_deg{4.0};
  double centerline_fit_maximum_residual_px{5.0};
  double centerline_fit_maximum_turn_deg_per_segment{28.0};
  double centerline_fit_maximum_turn_change_deg_per_segment{10.0};
  double centerline_fit_minimum_forward_progress_ratio{0.20};
  double centerline_fit_hermite_tangent_scale{0.75};
  int centerline_transition_frames{4};
  double centerline_maximum_lateral_jump_px{3.0};
  double centerline_maximum_heading_jump_deg{5.0};

  // Run the same detector in row and column directions every frame, then
  // select seeds from the combined orientation-independent candidate set.
  bool column_tracking_enabled{true};

  // Join touching same-direction fragments, and join row/column tracks when
  // their endpoints touch or their perpendicular run evidence overlaps.
  bool cross_direction_merge_enabled{true};
  double cross_direction_merge_maximum_endpoint_distance_px{3.0};
  double cross_direction_merge_minimum_connector_support_ratio{0.70};
  double cross_direction_merge_maximum_turn_angle_deg{110.0};

  // Lock left/right roles after the first valid pair. A temporarily missing
  // single lane is then labelled by temporal continuity, not screen center.
  bool temporal_side_lock_enabled{true};
  int temporal_side_lock_reset_frames{30};
  double temporal_side_reacquire_base_distance_px{6.0};
  double temporal_side_reacquire_distance_per_missing_frame_px{2.0};
  double temporal_side_reacquire_maximum_distance_px{20.0};
};

struct BevLaneSeed
{
  bool valid{false};
  cv::Point2d image_point;
  std::vector<cv::Point2d> support_points;
  double arc_length_px{0.0};
  double mean_bilateral_contrast{0.0};
  double score{0.0};
};

enum class BevLaneCenterlineSource
{
  NONE = 0,
  PAIR,
  LEFT,
  RIGHT,
};

struct BevLaneSeedDetection
{
  // MONO8 image containing only the generated drive centerline.
  cv::Mat seed_mask;
  // BGR preview of enhanced Top-hat with ROI, evidence and seeds overlaid.
  cv::Mat preview;
  BevLaneSeed left;
  BevLaneSeed right;
  std::vector<cv::Point2d> centerline_points;
  BevLaneCenterlineSource centerline_source{BevLaneCenterlineSource::NONE};
  double estimated_lane_width_px{0.0};
  bool centerline_transition_used{false};
  int roi_top_row{0};
  int roi_bottom_row{0};
  int accepted_track_count{0};
  int accepted_row_track_count{0};
  int accepted_column_track_count{0};
  int merged_track_count{0};
  int strict_evidence_count{0};
  int relaxed_evidence_count{0};
  int slope_break_count{0};
  bool pair_valid{false};
  double pair_distance_px{0.0};
  int pair_distance_samples{0};
  int pair_distance_inliers{0};
  bool side_lock_initialized{false};
  bool temporal_labeling_used{false};
  bool side_lock_reset{false};
  bool column_tracking_used{false};
};

class BevLaneSeedDetector
{
public:
  explicit BevLaneSeedDetector(BevLaneSeedDetectorConfig config);

  BevLaneSeedDetection detect(
    const cv::Mat & gray,
    const cv::Mat & enhanced_top_hat,
    bool create_preview);

private:
  BevLaneSeedDetectorConfig config_;
  bool side_lock_initialized_{false};
  // -1=last confirmed single lane was left, +1=right, 0=no single-side state.
  // The role survives short detection gaps until a valid pair or lock reset.
  int remembered_single_side_{0};
  int both_sides_missing_frames_{0};
  int left_missing_frames_{0};
  int right_missing_frames_{0};
  BevLaneSeed remembered_left_;
  BevLaneSeed remembered_right_;
  double estimated_lane_width_px_{0.0};
  bool estimated_lane_width_initialized_{false};
  double left_center_bias_px_{0.0};
  double right_center_bias_px_{0.0};
  bool center_bias_initialized_{false};
  std::vector<cv::Point2d> previous_centerline_;
  BevLaneCenterlineSource previous_centerline_source_{
    BevLaneCenterlineSource::NONE};
  cv::Point2d centerline_transition_offset_;
  double centerline_transition_heading_deg_{0.0};
  int centerline_transition_frames_remaining_{0};
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__BEV_LANE_SEED_DETECTOR_HPP_
