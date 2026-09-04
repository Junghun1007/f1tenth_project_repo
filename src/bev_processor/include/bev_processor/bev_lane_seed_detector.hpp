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

  // Suppress sustained V/lightning-shaped reversals without treating the
  // pixel stair steps of a monotonic curve as independent slope jumps.
  bool slope_filter_enabled{true};
  int slope_median_window{5};
  double maximum_slope_change_px_per_row{2.0};

  double minimum_pair_distance_px{45.0};
  double maximum_pair_distance_px{100.0};

  // Use an accepted seed track only to initialize a vehicle-near sliding
  // elliptical window tracker. Each accepted window emits one
  // intensity-weighted point; centerline reconstruction consumes these points
  // instead of raw seed runs.
  bool sliding_window_enabled{true};
  double sliding_window_minimum_seed_arc_length_px{15.0};
  int sliding_window_initial_width_px{6};
  int sliding_window_initial_height_px{10};
  double sliding_window_growth_ratio{1.04};
  int sliding_window_maximum_width_px{25};
  int sliding_window_maximum_height_px{15};
  double sliding_window_step_ratio{0.60};
  int sliding_window_maximum_count{40};
  int sliding_window_minimum_bright_pixels{2};
  int sliding_window_maximum_consecutive_misses{2};
  double sliding_window_centroid_boundary_margin_px{2.0};
  double sliding_window_maximum_turn_deg_per_window{14.0};
  double sliding_window_maximum_turn_change_deg_per_window{3.0};
  double sliding_window_heading_update_gain{0.90};

  // Convert the selected boundaries into one drive centerline using the
  // reference reconstructor geometry. Real perpendicular pair midpoints take
  // priority; missing counterpart sections follow the longer boundary's safe
  // half-width normal offset.
  bool centerline_enabled{true};
  double centerline_meter_per_pixel{0.01};
  double centerline_expected_lane_width_m{0.65};
  double centerline_lane_width_tolerance_m{0.08};
  int centerline_minimum_points{6};
  int centerline_minimum_counterpart_points{3};
  double centerline_measured_point_smoothing_weight{0.70};
  double centerline_midpoint_smoothing_weight{0.45};
  double centerline_temporal_current_weight{0.60};
  double centerline_transition_maximum_correction_m{0.15};
  double centerline_transition_correction_decay{0.70};
  double centerline_tangent_window_m{0.12};
  double centerline_maximum_curvature_per_m{1.8};
  double centerline_maximum_heading_step_deg{14.0};
  double centerline_maximum_gap_fill_m{0.30};
  // Sharp pair corners use one locked boundary for the whole corner instead
  // of switching between ambiguous perpendicular correspondences. The
  // centerline can be biased toward that outer boundary without changing the
  // expected road width used to validate lane pairs.
  bool centerline_corner_longer_boundary_enabled{true};
  double centerline_corner_outward_bias_m{0.05};
  double centerline_corner_enter_heading_change_deg{40.0};
  double centerline_corner_exit_heading_change_deg{20.0};

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
  int temporal_side_lock_reset_frames{100};
  double temporal_side_reacquire_base_distance_px{45.0};
  double temporal_side_reacquire_distance_per_missing_frame_px{3.0};
  double temporal_side_reacquire_maximum_distance_px{63.0};
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
  bool centerline_transition_used{false};
  bool centerline_normal_offset_truncated{false};
  int centerline_direct_midpoint_count{0};
  bool centerline_corner_mode_used{false};
  // -1=left boundary, +1=right boundary, 0=not in pair-corner mode.
  int centerline_corner_reference_side{0};
  double centerline_heading_change_deg{0.0};
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
  std::vector<cv::Point2d> previous_centerline_;
  bool previous_centerline_from_pair_{false};
  bool previous_centerline_corner_mode_{false};
  double single_boundary_transition_correction_px_{0.0};
  double corner_transition_correction_px_{0.0};
  bool corner_mode_active_{false};
  // Preferred outer boundary: -1=left, +1=right, 0=not determined.
  int corner_reference_side_{0};
  // Vehicle turn direction: -1=left, +1=right, 0=not determined.
  int corner_turn_direction_{0};
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__BEV_LANE_SEED_DETECTOR_HPP_
