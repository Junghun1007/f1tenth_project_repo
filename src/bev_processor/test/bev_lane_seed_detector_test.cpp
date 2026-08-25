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

void testLeftLaneKeepsItsRoleAfterDropoutAndFalsePair()
{
  BevLaneSeedDetector detector = makeDetector();
  initializePair(&detector);

  for (const int column : {35, 50, 65, 80}) {
    const LaneImages single = makeVerticalLanes(120, 300, {column});
    const BevLaneSeedDetection detection = detector.detect(
      single.gray, single.response, false);
    requireSeedNear(detection, true, static_cast<double>(column));
  }

  const LaneImages missing = makeVerticalLanes(120, 300, {});
  const BevLaneSeedDetection lost = detector.detect(
    missing.gray, missing.response, false);
  require(
    !lost.left.valid && !lost.right.valid,
    "empty frame must not produce a lane seed");

  const LaneImages reacquired = makeVerticalLanes(120, 300, {22, 82});
  const BevLaneSeedDetection detection = detector.detect(
    reacquired.gray, reacquired.response, false);
  require(
    detection.pair_valid,
    "test setup must contain a geometrically valid false pair");
  requireSeedNear(detection, true, 82.0);
}

void testRightLaneKeepsItsRoleAfterDropoutAndFalsePair()
{
  BevLaneSeedDetector detector = makeDetector();
  initializePair(&detector);

  for (const int column : {65, 50, 35, 20}) {
    const LaneImages single = makeVerticalLanes(120, 300, {column});
    const BevLaneSeedDetection detection = detector.detect(
      single.gray, single.response, false);
    requireSeedNear(detection, false, static_cast<double>(column));
  }

  const LaneImages missing = makeVerticalLanes(120, 300, {});
  detector.detect(missing.gray, missing.response, false);

  const LaneImages reacquired = makeVerticalLanes(120, 300, {18, 78});
  const BevLaneSeedDetection detection = detector.detect(
    reacquired.gray, reacquired.response, false);
  require(
    detection.pair_valid,
    "test setup must contain a geometrically valid false pair");
  requireSeedNear(detection, false, 18.0);
}

}  // namespace

int main()
{
  testLeftLaneKeepsItsRoleAfterDropoutAndFalsePair();
  testRightLaneKeepsItsRoleAfterDropoutAndFalsePair();
  std::cout << "bev_lane_seed_detector_test passed\n";
  return 0;
}
