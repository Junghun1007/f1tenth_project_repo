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
  double roi_height_ratio{0.40};
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

struct BevLaneSeedDetection
{
  // MONO8 image containing only the selected left/right seed tracks.
  cv::Mat seed_mask;
  // BGR preview of enhanced Top-hat with ROI, evidence and seeds overlaid.
  cv::Mat preview;
  BevLaneSeed left;
  BevLaneSeed right;
  int roi_top_row{0};
  int roi_bottom_row{0};
  int accepted_track_count{0};
  int strict_evidence_count{0};
  int relaxed_evidence_count{0};
  int slope_break_count{0};
  bool pair_valid{false};
  double pair_distance_px{0.0};
  int pair_distance_samples{0};
  int pair_distance_inliers{0};
};

class BevLaneSeedDetector
{
public:
  explicit BevLaneSeedDetector(BevLaneSeedDetectorConfig config);

  BevLaneSeedDetection detect(
    const cv::Mat & gray,
    const cv::Mat & enhanced_top_hat,
    bool create_preview) const;

private:
  BevLaneSeedDetectorConfig config_;
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__BEV_LANE_SEED_DETECTOR_HPP_
