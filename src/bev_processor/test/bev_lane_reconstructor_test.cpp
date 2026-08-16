#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "bev_processor/bev_lane_reconstructor.hpp"

namespace
{

constexpr double kXMaxM = 3.0;
constexpr double kYMaxM = 0.6;
constexpr double kMeterPerPixel = 0.01;

void require(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

cv::Point metricPoint(const double x_m, const double y_m)
{
  return cv::Point(
    static_cast<int>(std::lround(
      (kYMaxM - y_m) / kMeterPerPixel - 0.5)),
    static_cast<int>(std::lround(
      (kXMaxM - x_m) / kMeterPerPixel - 0.5)));
}

double farCurveCenter(const double x_m)
{
  if (x_m <= 1.75) {
    return 0.0;
  }
  const double distance_m = x_m - 1.75;
  return 0.30 * distance_m * distance_m;
}

double closeCurveCenter(const double x_m)
{
  return 0.16 * std::sin(1.8 * (x_m - 0.25));
}

double tightCenterlineCurveCenter(const double x_m)
{
  const double distance_m = x_m - 0.20;
  return 0.45 - 0.90 * distance_m * distance_m;
}

double rotatedStraightCenter(const double x_m)
{
  return 0.30 - 0.30 * (x_m - 0.20);
}

std::vector<cv::Point> makeBoundary(
  const double lateral_offset_m,
  const double minimum_x_m,
  const double maximum_x_m,
  double (*center_function)(double))
{
  std::vector<cv::Point> points;
  for (
    double x_m = minimum_x_m;
    x_m <= maximum_x_m + 1.0e-9;
    x_m += 0.01)
  {
    points.push_back(metricPoint(
      x_m, center_function(x_m) + lateral_offset_m));
  }
  return points;
}

void drawBoundary(
  cv::Mat * image,
  const double lateral_offset_m,
  const double minimum_x_m,
  const double maximum_x_m,
  double (*center_function)(double),
  const int brightness,
  const int thickness)
{
  const std::vector<std::vector<cv::Point>> boundary{
    makeBoundary(
      lateral_offset_m, minimum_x_m, maximum_x_m, center_function)};
  cv::polylines(
    *image, boundary, false,
    cv::Scalar(brightness, brightness, brightness), thickness);
}

bev_processor::BevLaneReconstructor makeReconstructor()
{
  bev_processor::BevLaneReconstructorConfig config;
  config.minimum_brightness = 160;
  config.far_minimum_brightness = 110;
  config.maximum_saturation = 80;
  config.observation_minimum_x_m = 0.20;
  config.observation_maximum_x_m = 1.80;
  config.reconstruction_minimum_x_m = 0.20;
  config.reconstruction_maximum_x_m = 2.70;
  config.maximum_extrapolation_m = 0.20;
  config.expected_lane_width_m = 0.60;
  config.lane_width_tolerance_m = 0.075;
  config.minimum_points = 5;
  config.sliding_window_step_m = 0.06;
  config.sliding_window_length_m = 0.18;
  config.sliding_window_half_width_near_m = 0.12;
  config.sliding_window_half_width_far_m = 0.22;
  config.sliding_window_measurement_weight = 0.90;
  config.sliding_window_heading_weight = 0.80;
  config.minimum_window_pixel_count = 6;
  config.maximum_tracking_gap_m = 0.20;
  config.maximum_gap_fill_m = 0.26;
  config.measured_point_smoothing_weight = 0.85;
  config.centerline_preserve_reference_shape = true;
  config.temporal_tracking_enabled = true;
  config.temporal_hold_frames = 5;
  return bev_processor::BevLaneReconstructor(config);
}

double maximumMeasuredX(const std::vector<cv::Point2d> & points)
{
  double maximum_x_m = 0.0;
  for (const auto & point : points) {
    maximum_x_m = std::max(maximum_x_m, point.x);
  }
  return maximum_x_m;
}

double meanCenterError(
  const std::vector<cv::Point2d> & left,
  const std::vector<cv::Point2d> & right,
  double (*center_function)(double))
{
  const std::size_t count = std::min(left.size(), right.size());
  if (count == 0U) {
    return 1.0;
  }
  double total_error_m = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    const double x_m = 0.5 * (left[index].x + right[index].x);
    const double center_m = 0.5 * (left[index].y + right[index].y);
    total_error_m += std::abs(center_m - center_function(x_m));
  }
  return total_error_m / static_cast<double>(count);
}

std::vector<cv::Point2d> makeArcBoundaryMetric(const double side_sign)
{
  constexpr double radius_m = 0.80;
  constexpr double half_width_m = 0.30;
  std::vector<cv::Point2d> points;
  for (double arc_m = 0.0; arc_m <= 1.10; arc_m += 0.01) {
    const double heading = arc_m / radius_m;
    const cv::Point2d center(
      0.20 + radius_m * std::sin(heading),
      -0.20 + radius_m * (1.0 - std::cos(heading)));
    const cv::Point2d left_normal(-std::sin(heading), std::cos(heading));
    points.push_back(center + side_sign * half_width_m * left_normal);
  }
  return points;
}

void drawMetricPolyline(
  cv::Mat * image,
  const std::vector<cv::Point2d> & metric_points)
{
  std::vector<cv::Point> pixels;
  pixels.reserve(metric_points.size());
  for (const auto & point : metric_points) {
    pixels.push_back(metricPoint(point.x, point.y));
  }
  cv::polylines(
    *image, std::vector<std::vector<cv::Point>>{pixels}, false,
    cv::Scalar(245, 245, 245), 5);
}

double meanDistanceToReference(
  const std::vector<cv::Point2d> & measured,
  const std::vector<cv::Point2d> & reference)
{
  if (measured.empty()) {
    return 1.0;
  }
  double total_distance_m = 0.0;
  for (const auto & point : measured) {
    double best_distance_m = 1.0;
    for (const auto & expected : reference) {
      best_distance_m = std::min(
        best_distance_m, cv::norm(point - expected));
    }
    total_distance_m += best_distance_m;
  }
  return total_distance_m / static_cast<double>(measured.size());
}

bool maskHasWhiteNear(
  const cv::Mat & mask,
  const double x_m,
  const double y_m,
  const int radius_px = 3)
{
  cv::Point center = metricPoint(x_m, y_m);
  center.x += (mask.cols - 120) / 2;
  const int first_row = std::max(0, center.y - radius_px);
  const int last_row = std::min(mask.rows - 1, center.y + radius_px);
  const int first_column = std::max(0, center.x - radius_px);
  const int last_column = std::min(mask.cols - 1, center.x + radius_px);
  return cv::countNonZero(mask(cv::Range(first_row, last_row + 1),
      cv::Range(first_column, last_column + 1))) > 0;
}

void testFarCurveIsReacquiredFromActualPixels()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&image, 0.30, 0.20, 1.82, farCurveCenter, 245, 5);
  drawBoundary(&image, -0.30, 0.20, 1.82, farCurveCenter, 245, 5);
  // The curve begins past the 1.8m confidence boundary. It is deliberately
  // dimmer and wider to mimic distant BEV blur.
  drawBoundary(&image, 0.30, 1.78, 2.55, farCurveCenter, 150, 15);
  drawBoundary(&image, -0.30, 1.78, 2.55, farCurveCenter, 150, 15);

  const auto result = makeReconstructor().reconstruct(image);
  require(result.valid, "distant blurred curves must produce a clean mask");
  require(
    maximumMeasuredX(result.left_measured_points) > 2.40,
    "left sliding window must reacquire actual pixels beyond 1.8m");
  require(
    maximumMeasuredX(result.right_measured_points) > 2.40,
    "right sliding window must reacquire actual pixels beyond 1.8m");
  require(
    meanCenterError(
      result.left_measured_points,
      result.right_measured_points,
      farCurveCenter) < 0.055,
    "far tracking must follow measured pixels instead of a straight guess");
}

void testCloseNonQuadraticCurveKeepsPixelShape()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&image, 0.30, 0.20, 2.20, closeCurveCenter, 245, 5);
  drawBoundary(&image, -0.30, 0.20, 2.20, closeCurveCenter, 245, 5);

  const auto result = makeReconstructor().reconstruct(image);
  require(result.valid, "close non-quadratic curve must be tracked");
  require(
    meanCenterError(
      result.left_measured_points,
      result.right_measured_points,
      closeCurveCenter) < 0.035,
    "measured points must remain close to the actual non-quadratic pixels");
}

void testRotatingWindowsFollowTightArc()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  const auto left = makeArcBoundaryMetric(1.0);
  const auto right = makeArcBoundaryMetric(-1.0);
  drawMetricPolyline(&image, left);
  drawMetricPolyline(&image, right);

  auto reconstructor = makeReconstructor();
  const auto result = reconstructor.reconstruct(image);
  require(result.valid, "rotated sliding windows must follow a tight arc");
  require(
    result.left_measured_points.size() >= 8U &&
    result.right_measured_points.size() >= 8U,
    "both rotating windows must keep measuring the tight arc");
  require(
    meanDistanceToReference(result.left_measured_points, left) < 0.045 &&
    meanDistanceToReference(result.right_measured_points, right) < 0.045,
    "tight-arc measurements must remain on the actual pixels");
  bool left_window_found = false;
  bool right_window_found = false;
  for (const auto & window : result.sliding_windows) {
    left_window_found = left_window_found || window.left_lane;
    right_window_found = right_window_found || !window.left_lane;
    for (const auto & corner : window.corners) {
      require(
        std::isfinite(corner.x) && std::isfinite(corner.y),
        "sliding-window preview corners must stay finite");
    }
  }
  require(
    left_window_found && right_window_found,
    "debug output must expose both rotating search-window tracks");
}

void testSingleBoundaryGeneratesCenterlineDirectly()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&image, 0.30, 0.20, 1.80, closeCurveCenter, 245, 5);

  const auto result = makeReconstructor().reconstruct(image);
  require(result.valid, "one actual boundary must still produce an output");
  require(
    result.reconstructed_mask.cols == 260 &&
    result.centerline_reconstructed_mask.size() ==
    result.reconstructed_mask.size() &&
    result.left_reconstructed_mask.size() == result.reconstructed_mask.size() &&
    result.right_reconstructed_mask.size() == result.reconstructed_mask.size(),
    "centerline output must include the configured lateral margins");
  require(
    result.left_measured_points.size() >= 5U,
    "the visible boundary must be represented by actual measurements");
  require(
    cv::countNonZero(result.left_reconstructed_mask) > 0 &&
    cv::countNonZero(result.right_reconstructed_mask) == 0,
    "a missing boundary must not be replaced with a synthetic lane line");
  require(
    result.centerline_from_single_boundary &&
    result.centerline_point_count > 0,
    "one measured boundary must directly generate the drive centerline");
  const int row = metricPoint(1.0, 0.0).y;
  int run_count = 0;
  bool inside_run = false;
  const auto * pixels = result.reconstructed_mask.ptr<std::uint8_t>(row);
  for (int column = 0; column < result.reconstructed_mask.cols; ++column) {
    if (pixels[column] != 0U && !inside_run) {
      ++run_count;
      inside_run = true;
    } else if (pixels[column] == 0U) {
      inside_run = false;
    }
  }
  require(run_count == 1, "the published lane output must contain one centerline");
}

void testMissingPixelsStopLongPrediction()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&image, 0.30, 0.20, 1.55, farCurveCenter, 245, 5);
  drawBoundary(&image, -0.30, 0.20, 1.55, farCurveCenter, 245, 5);

  const auto result = makeReconstructor().reconstruct(image);
  require(result.valid, "the observed lane segment must remain available");
  require(
    result.reconstructed_maximum_x_m < 1.85,
    "prediction without pixels must stop after the short extrapolation limit");
}

void testBroadBrightSceneryDoesNotPullTracker()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&image, 0.30, 0.20, 1.80, farCurveCenter, 245, 5);
  drawBoundary(&image, -0.30, 0.20, 1.80, farCurveCenter, 245, 5);
  cv::rectangle(
    image, metricPoint(1.55, 0.55), metricPoint(0.80, 0.37),
    cv::Scalar(245, 245, 245), cv::FILLED);

  const auto result = makeReconstructor().reconstruct(image);
  require(result.valid, "actual lanes must survive a nearby bright region");
  require(
    maximumMeasuredX(result.left_measured_points) > 1.50,
    "the thin actual lane must be preferred over the broad bright region");
  for (const auto & point : result.left_measured_points) {
    require(
      std::abs(point.y - 0.30) < 0.07,
      "broad scenery must not pull the measured lane sideways");
  }
}

void testBrightBackgroundIsNotAcceptedAsRoadLane()
{
  cv::Mat image(300, 120, CV_8UC3, cv::Scalar(180, 180, 180));
  drawBoundary(&image, 0.30, 0.20, 1.70, farCurveCenter, 245, 5);
  drawBoundary(&image, -0.30, 0.20, 1.70, farCurveCenter, 245, 5);

  require(
    !makeReconstructor().reconstruct(image).valid,
    "white lines without a dark local background must be rejected");
}

void testDefaultOutputLinesStayThin()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&image, 0.30, 0.20, 1.70, farCurveCenter, 245, 5);
  drawBoundary(&image, -0.30, 0.20, 1.70, farCurveCenter, 245, 5);

  const auto result = makeReconstructor().reconstruct(image);
  const int white_pixels = cv::countNonZero(
    result.reconstructed_mask.row(metricPoint(1.0, 0.0).y));
  require(
    white_pixels >= 2 && white_pixels <= 5,
    "the 2cm output setting must render one thin centerline");
}

void testTemporalGateRequiresFourRepeatedLargeChanges()
{
  auto reconstructor = makeReconstructor();
  cv::Mat baseline = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&baseline, 0.30, 0.20, 1.70, farCurveCenter, 245, 5);
  drawBoundary(&baseline, -0.30, 0.20, 1.70, farCurveCenter, 245, 5);
  require(reconstructor.reconstruct(baseline).valid, "baseline must be valid");

  cv::Mat shifted = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&shifted, 0.48, 0.20, 1.70, farCurveCenter, 245, 5);
  drawBoundary(&shifted, -0.12, 0.20, 1.70, farCurveCenter, 245, 5);
  const auto first_shift = reconstructor.reconstruct(shifted);
  const auto second_shift = reconstructor.reconstruct(shifted);
  const auto third_shift = reconstructor.reconstruct(shifted);
  require(
    first_shift.temporal_hold_used &&
    second_shift.temporal_hold_used &&
    third_shift.temporal_hold_used,
    "three large frame-to-frame jumps must still hold the accepted lane");
  require(
    maskHasWhiteNear(first_shift.reconstructed_mask, 1.0, 0.0) &&
    !maskHasWhiteNear(first_shift.reconstructed_mask, 1.0, 0.18),
    "an unconfirmed jump must not move the rendered centerline");

  const auto confirmed_shift = reconstructor.reconstruct(shifted);
  require(
    !confirmed_shift.temporal_hold_used &&
    maskHasWhiteNear(confirmed_shift.reconstructed_mask, 1.0, 0.18),
    "the same large change in four frames must be accepted");
}

void testTemporalHoldExpiresAfterFiveFrames()
{
  auto reconstructor = makeReconstructor();
  cv::Mat baseline = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&baseline, 0.30, 0.20, 1.70, farCurveCenter, 245, 5);
  drawBoundary(&baseline, -0.30, 0.20, 1.70, farCurveCenter, 245, 5);
  require(reconstructor.reconstruct(baseline).valid, "baseline must be valid");

  const cv::Mat blank = cv::Mat::zeros(300, 120, CV_8UC3);
  std::vector<bev_processor::BevLaneReconstruction> held_results;
  for (int frame = 0; frame < 5; ++frame) {
    held_results.push_back(reconstructor.reconstruct(blank));
  }
  const auto expired = reconstructor.reconstruct(blank);
  require(
    std::all_of(
      held_results.begin(), held_results.end(),
      [](const bev_processor::BevLaneReconstruction & result) {
        return result.valid && result.temporal_hold_used;
      }),
    "the accepted lane must be held for five missing frames");
  require(
    !expired.valid,
    "stale lanes must disappear after the configured hold duration");
}

void testValidShortLaneIsNotExtended()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&image, 0.30, 0.20, 2.00, farCurveCenter, 245, 5);
  drawBoundary(&image, -0.30, 0.20, 1.00, farCurveCenter, 245, 5);

  const auto result = makeReconstructor().reconstruct(image);
  require(result.valid, "a partially visible lane pair must remain valid");
  require(
    result.right_measured_points.size() >= 5U &&
    maximumMeasuredX(result.right_measured_points) < 1.25,
    "the short boundary must remain a valid measured lane");
  require(
    !result.centerline_from_single_boundary &&
    !maskHasWhiteNear(result.right_reconstructed_mask, 1.70, -0.30),
    "a valid short boundary must remain measured without synthetic extension");
}

void testShortNormalDistanceCounterpartContributesToCenterline()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  constexpr double heading_rad = -0.5235987755982988;  // -30 degrees.
  const cv::Point2d tangent(std::cos(heading_rad), std::sin(heading_rad));
  const cv::Point2d left_normal(-tangent.y, tangent.x);
  const cv::Point2d center_start(0.42, 0.25);
  const auto makeParallelBoundary = [&](
      const double side_sign, const double length_m) {
      std::vector<cv::Point2d> points;
      for (double distance_m = 0.0;
        distance_m <= length_m + 1.0e-9; distance_m += 0.01)
      {
        points.push_back(
          center_start + distance_m * tangent +
          side_sign * 0.325 * left_normal);
      }
      return points;
    };
  drawMetricPolyline(&image, makeParallelBoundary(1.0, 0.75));
  drawMetricPolyline(&image, makeParallelBoundary(-1.0, 0.32));

  bev_processor::BevLaneReconstructorConfig config;
  config.expected_lane_width_m = 0.65;
  config.lane_width_tolerance_m = 0.07;
  config.minimum_points = 8;
  config.minimum_counterpart_points = 3;
  config.initial_center_tolerance_m = 0.60;
  config.single_lane_initial_tolerance_m = 0.60;
  config.temporal_tracking_enabled = false;
  config.sliding_window_half_width_near_m = 0.12;
  config.sliding_window_half_width_far_m = 0.14;
  bev_processor::BevLaneReconstructor reconstructor(config);
  const auto result = reconstructor.reconstruct(image);

  require(result.valid, "a steep parallel pair must produce output");
  require(
    result.left_measured_points.size() >= 8U &&
    result.right_measured_points.size() >= 3U,
    "normal-distance pairing must retain the short visible counterpart");
  require(
    !result.centerline_from_single_boundary &&
    result.centerline_point_count > 0,
    "real counterpart pixels must contribute to the fused centerline");
}

void testTightCurveGeneratesHalfWidthCenterline()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(
    &image, 0.0, 0.20, 0.90, tightCenterlineCurveCenter, 245, 5);

  bev_processor::BevLaneReconstructorConfig config;
  config.expected_lane_width_m = 0.65;
  config.lane_width_tolerance_m = 0.07;
  config.minimum_points = 5;
  config.minimum_counterpart_points = 3;
  config.initial_center_tolerance_m = 0.60;
  config.single_lane_initial_tolerance_m = 0.60;
  config.temporal_tracking_enabled = false;
  config.centerline_preserve_reference_shape = false;
  config.sliding_window_half_width_near_m = 0.12;
  config.sliding_window_half_width_far_m = 0.14;
  bev_processor::BevLaneReconstructor reconstructor(config);
  const auto result = reconstructor.reconstruct(image);

  require(result.valid, "a single tight curve must produce output");
  require(
    result.centerline_from_single_boundary &&
    result.centerline_point_count > 0,
    "a safe portion of the half-width centerline must remain visible");
  require(
    cv::countNonZero(result.right_reconstructed_mask) == 0,
    "tight-curve centerline generation must not draw a synthetic boundary");

  constexpr double reference_x_m = 0.90;
  const double derivative = -1.80 * (reference_x_m - 0.20);
  const cv::Point2d raw_tangent(1.0, derivative);
  const cv::Point2d tangent = raw_tangent / cv::norm(raw_tangent);
  const cv::Point2d right_normal(tangent.y, -tangent.x);
  const cv::Point2d expected =
    cv::Point2d(
      reference_x_m, tightCenterlineCurveCenter(reference_x_m)) +
    0.325 * right_normal;
  require(
    maskHasWhiteNear(
      result.reconstructed_mask, expected.x, expected.y, 8),
    "tight-curve output must use the curvature-aware half-width offset");
}

void testCenterlineCanPreserveReferenceShape()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&image, 0.30, 0.20, 1.80, closeCurveCenter, 245, 5);

  const auto result = makeReconstructor().reconstruct(image);
  require(result.valid, "a single curved boundary must produce an output");
  for (const double x_m : {0.70, 1.10, 1.50}) {
    require(
      maskHasWhiteNear(
        result.reconstructed_mask,
        x_m, closeCurveCenter(x_m),
        4),
      "the centerline must retain the same-X reference shape when requested");
  }
}

void testCenterlineUsesHalfConfiguredWidthInsteadOfLearnedWidth()
{
  auto reconstructor = makeReconstructor();
  cv::Mat measured_pair = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&measured_pair, 0.33, 0.20, 1.70, farCurveCenter, 245, 5);
  drawBoundary(&measured_pair, -0.33, 0.20, 1.70, farCurveCenter, 245, 5);
  const auto learned = reconstructor.reconstruct(measured_pair);
  require(
    learned.measured_lane_width_m > 0.63,
    "the setup frame must learn a width wider than the configured 60cm");

  cv::Mat left_only = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&left_only, 0.30, 0.20, 1.70, farCurveCenter, 245, 5);
  bev_processor::BevLaneReconstruction centered;
  for (int frame = 0; frame <= 12; ++frame) {
    centered = reconstructor.reconstruct(left_only);
  }
  require(
    centered.centerline_point_count > 0,
    "the single measured boundary must generate a centerline");
  require(
    maskHasWhiteNear(centered.reconstructed_mask, 1.0, 0.0, 3),
    "centerline must use half of the configured 60cm width");
  require(
    !maskHasWhiteNear(centered.reconstructed_mask, 1.0, -0.03, 1),
    "a previously learned wider pair must not move the centerline");
}

void testMeasuredPairGeneratesDirectMidpointCenterline()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&image, 0.35, 0.20, 1.50, farCurveCenter, 245, 5);
  drawBoundary(&image, -0.25, 0.20, 1.50, farCurveCenter, 245, 5);

  bev_processor::BevLaneReconstructorConfig config;
  config.initial_center_tolerance_m = 0.60;
  config.single_lane_initial_tolerance_m = 0.60;
  config.temporal_tracking_enabled = false;
  config.centerline_preserve_reference_shape = false;
  bev_processor::BevLaneReconstructor reconstructor(config);
  const auto result = reconstructor.reconstruct(image);

  require(result.valid, "a measured pair must generate a centerline");
  require(
    !result.centerline_from_single_boundary &&
    result.centerline_point_count > 0,
    "both measured boundaries must be the source of the centerline");
  require(
    maskHasWhiteNear(result.reconstructed_mask, 0.90, 0.05, 3),
    "paired centerline must pass through the midpoint of measured boundaries");
}

void testPairToSingleTransitionDecaysWithoutHidingNewCenterline()
{
  bev_processor::BevLaneReconstructorConfig config;
  config.initial_center_tolerance_m = 0.60;
  config.single_lane_initial_tolerance_m = 0.60;
  config.temporal_tracking_enabled = false;
  config.centerline_preserve_reference_shape = true;
  config.centerline_transition_maximum_correction_m = 0.15;
  config.centerline_transition_correction_decay = 0.65;
  bev_processor::BevLaneReconstructor reconstructor(config);

  cv::Mat pair = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&pair, 0.35, 0.20, 1.50, farCurveCenter, 245, 5);
  drawBoundary(&pair, -0.25, 0.20, 1.50, farCurveCenter, 245, 5);
  const auto paired = reconstructor.reconstruct(pair);
  require(
    paired.valid &&
    maskHasWhiteNear(paired.reconstructed_mask, 0.90, 0.05, 3),
    "setup pair must establish the measured midpoint centerline");

  cv::Mat left_only = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&left_only, 0.42, 0.20, 1.50, farCurveCenter, 245, 5);
  const auto first_single = reconstructor.reconstruct(left_only);
  require(
    first_single.centerline_from_single_boundary &&
    maskHasWhiteNear(first_single.reconstructed_mask, 0.90, 0.05, 3),
    "first single-boundary frame must stay aligned with the paired centerline");

  bev_processor::BevLaneReconstruction settled = first_single;
  for (int frame = 0; frame < 9; ++frame) {
    settled = reconstructor.reconstruct(left_only);
  }
  require(
    maskHasWhiteNear(settled.reconstructed_mask, 0.90, 0.12, 3),
    "transition correction must decay so the new lateral center is preserved");
}

void testCurrentFrameRotationUpdatesCenterlineImmediately()
{
  bev_processor::BevLaneReconstructorConfig config;
  config.initial_center_tolerance_m = 0.60;
  config.single_lane_initial_tolerance_m = 0.60;
  config.temporal_tracking_enabled = false;
  config.temporal_hold_frames = 0;
  config.centerline_preserve_reference_shape = false;
  bev_processor::BevLaneReconstructor reconstructor(config);

  cv::Mat first = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&first, 0.30, 0.20, 1.40, farCurveCenter, 245, 5);
  require(reconstructor.reconstruct(first).valid, "first frame must be valid");

  cv::Mat rotated = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&rotated, 0.0, 0.20, 1.40, rotatedStraightCenter, 245, 5);
  const auto result = reconstructor.reconstruct(rotated);
  const cv::Point2d tangent = cv::Point2d(1.0, -0.30) /
    cv::norm(cv::Point2d(1.0, -0.30));
  const cv::Point2d reference_point(0.80, rotatedStraightCenter(0.80));
  const cv::Point2d expected = reference_point +
    0.30 * cv::Point2d(tangent.y, -tangent.x);

  require(result.valid, "rotated current frame must remain valid");
  require(
    !result.temporal_hold_used,
    "current-view mode must never hold a previous vehicle-frame lane");
  require(
    maskHasWhiteNear(
      result.reconstructed_mask, expected.x, expected.y, 5),
    "centerline must move immediately to the current frame half-width offset");
}

void testEitherVisibleSideGeneratesCenterlineEveryFrame()
{
  bev_processor::BevLaneReconstructorConfig config;
  config.initial_center_tolerance_m = 0.60;
  config.single_lane_initial_tolerance_m = 0.60;
  config.temporal_tracking_enabled = false;
  config.temporal_hold_frames = 0;
  config.centerline_preserve_reference_shape = false;
  bev_processor::BevLaneReconstructor reconstructor(config);

  cv::Mat left_only = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&left_only, 0.30, 0.20, 1.40, farCurveCenter, 245, 5);
  const auto from_left = reconstructor.reconstruct(left_only);
  require(
    from_left.left_measured_points.size() >=
    static_cast<std::size_t>(config.minimum_points) &&
    from_left.right_measured_points.empty(),
    "a visible left lane must remain measured while the right is missing");
  require(
    from_left.centerline_from_single_boundary &&
    from_left.centerline_point_count > 0 &&
    cv::countNonZero(from_left.right_reconstructed_mask) == 0 &&
    maskHasWhiteNear(from_left.reconstructed_mask, 0.80, 0.0, 4),
    "a visible left lane must directly generate the centerline");

  cv::Mat right_only = cv::Mat::zeros(300, 120, CV_8UC3);
  drawBoundary(&right_only, -0.30, 0.20, 1.40, farCurveCenter, 245, 5);
  const auto from_right = reconstructor.reconstruct(right_only);
  require(
    from_right.right_measured_points.size() >=
    static_cast<std::size_t>(config.minimum_points) &&
    from_right.left_measured_points.empty(),
    "a visible right lane must remain measured while the left is missing");
  require(
    from_right.centerline_from_single_boundary &&
    from_right.centerline_point_count > 0 &&
    cv::countNonZero(from_right.left_reconstructed_mask) == 0 &&
    maskHasWhiteNear(from_right.reconstructed_mask, 0.80, 0.0, 4),
    "a visible right lane must directly generate the centerline");
  require(
    !from_left.temporal_hold_used && !from_right.temporal_hold_used,
    "side changes must use each current frame without a stale held lane");
}

void testLowSaturationGateRemainsOptional()
{
  cv::Mat image = cv::Mat::zeros(300, 120, CV_8UC3);
  const auto left = makeBoundary(0.30, 0.20, 1.70, farCurveCenter);
  const auto right = makeBoundary(-0.30, 0.20, 1.70, farCurveCenter);
  cv::polylines(
    image, std::vector<std::vector<cv::Point>>{left}, false,
    cv::Scalar(0, 0, 255), 5);
  cv::polylines(
    image, std::vector<std::vector<cv::Point>>{right}, false,
    cv::Scalar(0, 0, 255), 5);

  bev_processor::BevLaneReconstructorConfig config;
  config.minimum_brightness = 70;
  config.maximum_saturation = 80;
  config.expected_lane_width_m = 0.60;
  config.lane_width_tolerance_m = 0.14;
  bev_processor::BevLaneReconstructor reconstructor(config);
  require(
    !reconstructor.reconstruct(image).valid,
    "the optional saturation gate must reject strongly colored markings");
}

}  // namespace

int main()
{
  testFarCurveIsReacquiredFromActualPixels();
  testCloseNonQuadraticCurveKeepsPixelShape();
  testSingleBoundaryGeneratesCenterlineDirectly();
  testMissingPixelsStopLongPrediction();
  testBroadBrightSceneryDoesNotPullTracker();
  testBrightBackgroundIsNotAcceptedAsRoadLane();
  testDefaultOutputLinesStayThin();
  testTemporalGateRequiresFourRepeatedLargeChanges();
  testTemporalHoldExpiresAfterFiveFrames();
  testValidShortLaneIsNotExtended();
  testShortNormalDistanceCounterpartContributesToCenterline();
  testTightCurveGeneratesHalfWidthCenterline();
  testCenterlineCanPreserveReferenceShape();
  testCenterlineUsesHalfConfiguredWidthInsteadOfLearnedWidth();
  testMeasuredPairGeneratesDirectMidpointCenterline();
  testPairToSingleTransitionDecaysWithoutHidingNewCenterline();
  testCurrentFrameRotationUpdatesCenterlineImmediately();
  testEitherVisibleSideGeneratesCenterlineEveryFrame();
  testLowSaturationGateRemainsOptional();
  testRotatingWindowsFollowTightArc();
  std::cout << "BEV sliding-window lane tests passed\n";
  return EXIT_SUCCESS;
}
