#include "bev_processor/bev_lane_seed_detector.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <opencv2/core.hpp>

namespace
{

using bev_processor::BevLaneSeedDetection;
using bev_processor::BevLaneSeedDetector;
using bev_processor::BevLaneSeedDetectorConfig;
using bev_processor::BevLaneCenterlineSource;

struct LaneImages
{
  cv::Mat gray;
  cv::Mat response;
};

LaneImages makeVerticalLanes(
  const int width,
  const int height,
  const std::vector<int> & center_columns)
{
  LaneImages images{
    cv::Mat::zeros(height, width, CV_8UC1),
    cv::Mat::zeros(height, width, CV_8UC1)};
  for (const int center : center_columns) {
    for (int row = 0; row < height; ++row) {
      for (int column = center - 1; column <= center + 1; ++column) {
        images.gray.at<unsigned char>(row, column) = 255;
        images.response.at<unsigned char>(row, column) = 255;
      }
    }
  }
  return images;
}

LaneImages makeParallelLanes(
  const int width,
  const int height,
  const std::vector<int> & center_columns_by_row)
{
  LaneImages images{
    cv::Mat::zeros(height, width, CV_8UC1),
    cv::Mat::zeros(height, width, CV_8UC1)};
  for (int row = 0; row < height; ++row) {
    const int center = center_columns_by_row[static_cast<std::size_t>(row)];
    for (const int lane_center : {center - 30, center + 30}) {
      for (int column = lane_center - 1; column <= lane_center + 1; ++column) {
        if (column >= 0 && column < width) {
          images.gray.at<unsigned char>(row, column) = 255;
          images.response.at<unsigned char>(row, column) = 255;
        }
      }
    }
  }
  return images;
}

LaneImages makeTurningParallelLanes(
  const int width,
  const int height,
  const int straight_center_column,
  const int signed_far_shift)
{
  std::vector<int> center_columns(static_cast<std::size_t>(height));
  constexpr int kBendRows = 80;
  for (int row = 0; row < height; ++row) {
    const double progress = row < kBendRows ?
      static_cast<double>(kBendRows - row) /
      static_cast<double>(kBendRows) : 0.0;
    center_columns[static_cast<std::size_t>(row)] =
      straight_center_column + static_cast<int>(std::lround(
      static_cast<double>(signed_far_shift) * progress * progress));
  }
  return makeParallelLanes(width, height, center_columns);
}

void require(const bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void requireSeedNear(
  const BevLaneSeedDetection & detection,
  const bool expect_left,
  const double expected_column)
{
  const auto & expected = expect_left ? detection.left : detection.right;
  const auto & opposite = expect_left ? detection.right : detection.left;
  require(expected.valid, "expected remembered lane side is missing");
  require(!opposite.valid, "reacquired lane was also assigned to the opposite side");
  require(
    std::abs(expected.image_point.x - expected_column) <= 1.0,
    "reacquired lane seed is not the temporally matched candidate");
}

BevLaneSeedDetector makeDetector()
{
  BevLaneSeedDetectorConfig config;
  config.roi_bottom_exclusion_ratio = 0.0;
  config.roi_height_ratio = 1.0;
  config.column_tracking_enabled = false;
  config.cross_direction_merge_enabled = false;
  // Keep dropout-gating tests independent from vehicle tuning defaults.
  config.temporal_side_reacquire_base_distance_px = 10.0;
  config.temporal_side_reacquire_distance_per_missing_frame_px = 2.0;
  config.temporal_side_reacquire_maximum_distance_px = 30.0;
  return BevLaneSeedDetector(config);
}

void initializePair(BevLaneSeedDetector * detector)
{
  const LaneImages initial = makeVerticalLanes(120, 300, {20, 80});
  const BevLaneSeedDetection detection = detector->detect(
    initial.gray, initial.response, false);
  require(detection.pair_valid, "initial pair must pass the lane-width gate");
  require(
    detection.left.valid && detection.right.valid,
    "initial pair must initialize both lane roles");
}

void requirePairNear(
  const BevLaneSeedDetection & detection,
  const double expected_left_column,
  const double expected_right_column)
{
  require(
    detection.left.valid && detection.right.valid,
    "valid pair must select both lane sides");
  require(
    std::abs(detection.left.image_point.x - expected_left_column) <= 1.0,
    "left lane seed is not the current pair's left candidate");
  require(
    std::abs(detection.right.image_point.x - expected_right_column) <= 1.0,
    "right lane seed is not the current pair's right candidate");
}

void selectSingleLeftLane(BevLaneSeedDetector * detector)
{
  const LaneImages single = makeVerticalLanes(120, 300, {25});
  const BevLaneSeedDetection detection = detector->detect(
    single.gray, single.response, false);
  requireSeedNear(detection, true, 25.0);
}

void selectSingleRightLane(BevLaneSeedDetector * detector)
{
  const LaneImages single = makeVerticalLanes(120, 300, {75});
  const BevLaneSeedDetection detection = detector->detect(
    single.gray, single.response, false);
  requireSeedNear(detection, false, 75.0);
}

void insertMissingFrames(BevLaneSeedDetector * detector, const int count)
{
  const LaneImages missing = makeVerticalLanes(120, 300, {});
  for (int index = 0; index < count; ++index) {
    const BevLaneSeedDetection detection = detector->detect(
      missing.gray, missing.response, false);
    require(
      !detection.left.valid && !detection.right.valid,
      "empty frame must not produce a lane seed");
  }
}

void requireNoSeeds(
  const BevLaneSeedDetection & detection,
  const char * message)
{
  require(!detection.left.valid && !detection.right.valid, message);
}

double centerColumnNearRow(
  const BevLaneSeedDetection & detection,
  const double target_row)
{
  require(!detection.centerline_points.empty(), "centerline is empty");
  const auto closest = std::min_element(
    detection.centerline_points.begin(), detection.centerline_points.end(),
    [target_row](const cv::Point2d & first, const cv::Point2d & second) {
      return std::abs(first.y - target_row) < std::abs(second.y - target_row);
    });
  return closest->x;
}

void testLeftLaneKeepsItsRoleAfterSingleLaneDropout()
{
  BevLaneSeedDetector detector = makeDetector();
  initializePair(&detector);
  selectSingleLeftLane(&detector);
  insertMissingFrames(&detector, 1);

  const LaneImages reacquired = makeVerticalLanes(120, 300, {31});
  const BevLaneSeedDetection detection = detector.detect(
    reacquired.gray, reacquired.response, false);
  requireSeedNear(detection, true, 31.0);
}

void testRightLaneKeepsItsRoleAfterSingleLaneDropout()
{
  BevLaneSeedDetector detector = makeDetector();
  initializePair(&detector);
  selectSingleRightLane(&detector);
  insertMissingFrames(&detector, 1);

  const LaneImages reacquired = makeVerticalLanes(120, 300, {69});
  const BevLaneSeedDetection detection = detector.detect(
    reacquired.gray, reacquired.response, false);
  requireSeedNear(detection, false, 69.0);
}

void testSingleLaneRejectsDistantReacquisition()
{
  BevLaneSeedDetector detector = makeDetector();
  initializePair(&detector);
  selectSingleLeftLane(&detector);
  insertMissingFrames(&detector, 1);

  const LaneImages distant = makeVerticalLanes(120, 300, {60});
  const BevLaneSeedDetection detection = detector.detect(
    distant.gray, distant.response, false);
  requireNoSeeds(
    detection, "distant single-lane reacquisition must be rejected");
}

void testReacquisitionAllowanceGrowsWithMissingFrames()
{
  BevLaneSeedDetector detector = makeDetector();
  initializePair(&detector);
  selectSingleLeftLane(&detector);
  insertMissingFrames(&detector, 3);

  // The explicit 10px base plus missing-frame growth permits this 12px move.
  const LaneImages reacquired = makeVerticalLanes(120, 300, {37});
  const BevLaneSeedDetection detection = detector.detect(
    reacquired.gray, reacquired.response, false);
  requireSeedNear(detection, true, 37.0);
}

void testValidPairReinitializesBothSides()
{
  BevLaneSeedDetector detector = makeDetector();
  initializePair(&detector);
  insertMissingFrames(&detector, 1);

  const LaneImages reacquired = makeVerticalLanes(120, 300, {22, 82});
  const BevLaneSeedDetection detection = detector.detect(
    reacquired.gray, reacquired.response, false);
  require(
    detection.pair_valid,
    "two reacquired lanes must pass the pair-width gate");
  requirePairNear(detection, 22.0, 82.0);
}

void testPairRejectsDistantReacquisition()
{
  BevLaneSeedDetector detector = makeDetector();
  initializePair(&detector);
  insertMissingFrames(&detector, 1);

  const LaneImages distant = makeVerticalLanes(120, 300, {40, 100});
  const BevLaneSeedDetection detection = detector.detect(
    distant.gray, distant.response, false);
  require(!detection.pair_valid, "distant lane pair must fail temporal gating");
  requireNoSeeds(detection, "distant lane pair must not produce lane seeds");
}

void testPairProducesExplicitCenterline()
{
  BevLaneSeedDetector detector = makeDetector();
  const LaneImages lanes = makeVerticalLanes(120, 300, {20, 80});
  const BevLaneSeedDetection detection = detector.detect(
    lanes.gray, lanes.response, false);
  require(
    detection.centerline_source == BevLaneCenterlineSource::PAIR,
    "lane pair must produce a pair centerline");
  require(
    detection.centerline_direct_midpoint_count >= 3,
    "lane pair did not use perpendicular midpoint correspondences");
  require(
    std::abs(centerColumnNearRow(detection, 150.0) - 50.0) <= 1.0,
    "pair centerline is not between the two boundaries");
  require(
    detection.seed_mask.at<unsigned char>(150, 50) != 0U,
    "published mono8 mask does not contain the explicit centerline");
  require(
    detection.seed_mask.at<unsigned char>(150, 20) == 0U &&
    detection.seed_mask.at<unsigned char>(150, 80) == 0U,
    "published mono8 mask still contains lane boundaries");
}

void testSingleBoundaryUsesConfiguredRoadWidth()
{
  BevLaneSeedDetector left_detector = makeDetector();
  initializePair(&left_detector);
  const LaneImages left_lane = makeVerticalLanes(120, 300, {20});
  const BevLaneSeedDetection left = left_detector.detect(
    left_lane.gray, left_lane.response, false);
  require(
    left.centerline_source == BevLaneCenterlineSource::LEFT,
    "single left boundary did not produce a left-derived centerline");
  require(
    std::abs(centerColumnNearRow(left, 150.0) - 50.0) <= 1.0,
    "left boundary centerline did not use half the configured road width");

  BevLaneSeedDetector right_detector = makeDetector();
  initializePair(&right_detector);
  const LaneImages right_lane = makeVerticalLanes(120, 300, {80});
  const BevLaneSeedDetection right = right_detector.detect(
    right_lane.gray, right_lane.response, false);
  require(
    right.centerline_source == BevLaneCenterlineSource::RIGHT,
    "single right boundary did not produce a right-derived centerline");
  require(
    std::abs(centerColumnNearRow(right, 150.0) - 50.0) <= 1.0,
    "right boundary centerline did not use half the configured road width");
}

void testPairToSingleTransitionLimitsLateralJump()
{
  BevLaneSeedDetector detector = makeDetector();
  initializePair(&detector);
  const LaneImages shifted_left = makeVerticalLanes(120, 300, {25});
  const BevLaneSeedDetection detection = detector.detect(
    shifted_left.gray, shifted_left.response, false);
  require(detection.centerline_transition_used, "pair-to-single transition unused");
  require(
    std::abs(centerColumnNearRow(detection, 150.0) - 50.0) <= 3.1,
    "pair-to-single centerline exceeded the configured lateral jump");
}

void testAdaptiveSmoothingPreservesStraightToCurve()
{
  BevLaneSeedDetector detector = makeDetector();
  std::vector<int> center_columns(300, 50);
  for (int row = 0; row < 150; ++row) {
    const double distance = static_cast<double>(150 - row);
    center_columns[static_cast<std::size_t>(row)] = static_cast<int>(
      std::lround(50.0 + 20.0 * distance * distance / (150.0 * 150.0)));
  }
  const LaneImages lanes = makeParallelLanes(120, 300, center_columns);
  const BevLaneSeedDetection detection = detector.detect(
    lanes.gray, lanes.response, false);
  require(
    detection.centerline_source == BevLaneCenterlineSource::PAIR,
    "straight-to-curve pair did not produce a centerline");
  require(
    std::abs(centerColumnNearRow(detection, 250.0) - 50.0) <= 2.0,
    "straight centerline segment was displaced");
  require(
    centerColumnNearRow(detection, 5.0) >= 66.0,
    "adaptive smoothing flattened the sustained curve");
}

void testCenterlineUsesSlidingWindowTracking()
{
  BevLaneSeedDetectorConfig config;
  config.roi_bottom_exclusion_ratio = 0.0;
  config.roi_height_ratio = 0.25;
  config.column_tracking_enabled = false;
  config.cross_direction_merge_enabled = false;
  BevLaneSeedDetector detector(config);
  const LaneImages lanes = makeVerticalLanes(120, 300, {20, 80});
  const BevLaneSeedDetection detection = detector.detect(
    lanes.gray, lanes.response, false);
  require(
    detection.centerline_source == BevLaneCenterlineSource::PAIR,
    "sliding-window lane pair did not produce a centerline");
  const auto farthest = std::min_element(
    detection.centerline_points.begin(), detection.centerline_points.end(),
    [](const cv::Point2d & first, const cv::Point2d & second) {
      return first.y < second.y;
    });
  require(
    farthest != detection.centerline_points.end() && farthest->y < 150.0,
    "sliding-window tracking did not carry the centerline above the ROI");
  require(
    std::abs(farthest->x - 50.0) <= 1.0,
    "sliding-window tracking displaced the pair centerline");
}

void testCenterlineSuppressesIsolatedBump()
{
  BevLaneSeedDetectorConfig config;
  config.roi_bottom_exclusion_ratio = 0.0;
  config.roi_height_ratio = 1.0;
  config.maximum_lateral_step_px = 10.0;
  config.slope_filter_enabled = false;
  config.column_tracking_enabled = false;
  config.cross_direction_merge_enabled = false;
  config.centerline_midpoint_smoothing_weight = 0.10;
  BevLaneSeedDetector detector(config);
  std::vector<int> center_columns(300, 50);
  center_columns[150] = 58;
  const LaneImages lanes = makeParallelLanes(120, 300, center_columns);
  const BevLaneSeedDetection detection = detector.detect(
    lanes.gray, lanes.response, false);
  require(
    detection.centerline_source == BevLaneCenterlineSource::PAIR,
    "bumped lane pair did not produce a centerline");
  require(
    std::abs(centerColumnNearRow(detection, 150.0) - 50.0) <= 2.0,
    "isolated centerline bump was not suppressed");
}

void testCornerUsesCurrentTurnOuterBoundaryAfterStraight()
{
  BevLaneSeedDetectorConfig config;
  config.image_width = 200;
  config.image_height = 300;
  config.roi_bottom_exclusion_ratio = 0.0;
  config.roi_height_ratio = 1.0;
  config.column_tracking_enabled = false;
  config.cross_direction_merge_enabled = false;
  config.centerline_corner_enter_heading_change_deg = 30.0;
  config.centerline_corner_exit_heading_change_deg = 10.0;
  BevLaneSeedDetector detector(config);

  const LaneImages left_turn = makeTurningParallelLanes(
    config.image_width, config.image_height, 100, -50);
  const BevLaneSeedDetection left = detector.detect(
    left_turn.gray, left_turn.response, false);
  require(left.centerline_corner_mode_used, "left corner mode was not entered");
  require(
    left.centerline_corner_reference_side == 1,
    "left turn did not use the right outer boundary");

  const LaneImages straight = makeTurningParallelLanes(
    config.image_width, config.image_height, 100, 0);
  const BevLaneSeedDetection middle = detector.detect(
    straight.gray, straight.response, false);
  require(
    !middle.centerline_corner_mode_used,
    "straight pair did not clear the previous corner reference");

  const LaneImages right_turn = makeTurningParallelLanes(
    config.image_width, config.image_height, 100, 50);
  const BevLaneSeedDetection right = detector.detect(
    right_turn.gray, right_turn.response, false);
  require(right.centerline_corner_mode_used, "right corner mode was not entered");
  require(
    right.centerline_corner_reference_side == -1,
    "right turn retained the old right reference instead of the left outer boundary");
}

void testCornerOutwardBiasShiftsCenterlineTowardOuterBoundary()
{
  BevLaneSeedDetectorConfig unbiased_config;
  unbiased_config.image_width = 200;
  unbiased_config.image_height = 300;
  unbiased_config.roi_bottom_exclusion_ratio = 0.0;
  unbiased_config.roi_height_ratio = 1.0;
  unbiased_config.column_tracking_enabled = false;
  unbiased_config.cross_direction_merge_enabled = false;
  unbiased_config.centerline_corner_enter_heading_change_deg = 30.0;
  unbiased_config.centerline_corner_exit_heading_change_deg = 10.0;
  unbiased_config.centerline_corner_outward_bias_m = 0.0;
  BevLaneSeedDetector unbiased_detector(unbiased_config);

  BevLaneSeedDetectorConfig biased_config = unbiased_config;
  biased_config.centerline_corner_outward_bias_m = 0.05;
  BevLaneSeedDetector biased_detector(biased_config);

  const LaneImages right_turn = makeTurningParallelLanes(
    unbiased_config.image_width, unbiased_config.image_height, 100, 50);
  const BevLaneSeedDetection unbiased = unbiased_detector.detect(
    right_turn.gray, right_turn.response, false);
  const BevLaneSeedDetection biased = biased_detector.detect(
    right_turn.gray, right_turn.response, false);
  require(
    unbiased.centerline_corner_reference_side == -1 &&
    biased.centerline_corner_reference_side == -1,
    "right corner did not use the left outer boundary");
  require(
    centerColumnNearRow(biased, 250.0) <=
    centerColumnNearRow(unbiased, 250.0) - 4.0,
    "corner outward bias did not move the centerline toward the outer lane");
}

void testRawSeedEvidenceIsNotUsedAsCenterline()
{
  BevLaneSeedDetectorConfig config;
  config.roi_bottom_exclusion_ratio = 0.0;
  config.roi_height_ratio = 1.0;
  config.column_tracking_enabled = false;
  config.cross_direction_merge_enabled = false;
  config.sliding_window_enabled = false;
  BevLaneSeedDetector detector(config);
  const LaneImages lanes = makeVerticalLanes(120, 300, {20, 80});
  const BevLaneSeedDetection detection = detector.detect(
    lanes.gray, lanes.response, false);
  require(
    detection.centerline_source == BevLaneCenterlineSource::NONE &&
    detection.centerline_points.empty(),
    "raw seed evidence entered centerline reconstruction");
}

}  // namespace

int main()
{
  testLeftLaneKeepsItsRoleAfterSingleLaneDropout();
  testRightLaneKeepsItsRoleAfterSingleLaneDropout();
  testSingleLaneRejectsDistantReacquisition();
  testReacquisitionAllowanceGrowsWithMissingFrames();
  testValidPairReinitializesBothSides();
  testPairRejectsDistantReacquisition();
  testPairProducesExplicitCenterline();
  testSingleBoundaryUsesConfiguredRoadWidth();
  testPairToSingleTransitionLimitsLateralJump();
  testAdaptiveSmoothingPreservesStraightToCurve();
  testCenterlineUsesSlidingWindowTracking();
  testCenterlineSuppressesIsolatedBump();
  testCornerUsesCurrentTurnOuterBoundaryAfterStraight();
  testCornerOutwardBiasShiftsCenterlineTowardOuterBoundary();
  testRawSeedEvidenceIsNotUsedAsCenterline();
  std::cout << "bev_lane_seed_detector_test passed\n";
  return 0;
}
