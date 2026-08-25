#include "bev_processor/bev_lane_seed_detector.hpp"

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

  // 6px base + 3 missing frames * 2px permits this 12px displacement.
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

}  // namespace

int main()
{
  testLeftLaneKeepsItsRoleAfterSingleLaneDropout();
  testRightLaneKeepsItsRoleAfterSingleLaneDropout();
  testSingleLaneRejectsDistantReacquisition();
  testReacquisitionAllowanceGrowsWithMissingFrames();
  testValidPairReinitializesBothSides();
  testPairRejectsDistantReacquisition();
  std::cout << "bev_lane_seed_detector_test passed\n";
  return 0;
}
