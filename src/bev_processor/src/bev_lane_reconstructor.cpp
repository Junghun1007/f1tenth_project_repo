#include "bev_processor/bev_lane_reconstructor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace bev_processor
{
namespace
{

struct Run
{
  int first_column{0};
  int last_column{0};

  int width() const
  {
    return last_column - first_column + 1;
  }

  double centerColumn() const
  {
    return 0.5 * static_cast<double>(first_column + last_column);
  }
};

struct Candidate
{
  double lateral_m{0.0};
};

struct TrackSegment
{
  std::vector<cv::Point2d> center_points;
  std::vector<double> pair_widths_m;
  bool initialized{false};
  double tracked_left_m{0.0};
  double tracked_right_m{0.0};
  int last_observed_row{0};
  double missing_distance_m{0.0};

  int score() const
  {
    return
      static_cast<int>(center_points.size()) +
      2 * static_cast<int>(pair_widths_m.size());
  }
};

int makeOdd(const int value)
{
  const int positive = std::max(1, value);
  return positive % 2 == 0 ? positive + 1 : positive;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const auto middle = values.begin() +
    static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  double value = *middle;
  if (values.size() % 2U == 0U) {
    value = 0.5 * (value + *std::max_element(values.begin(), middle));
  }
  return value;
}

std::vector<Run> findRuns(
  const cv::Mat & binary,
  const int row,
  const int minimum_width_px,
  const int maximum_width_px)
{
  std::vector<Run> runs;
  const auto * pixels = binary.ptr<std::uint8_t>(row);
  int column = 0;
  while (column < binary.cols) {
    while (column < binary.cols && pixels[column] == 0U) {
      ++column;
    }
    if (column >= binary.cols) {
      break;
    }
    const int first_column = column;
    while (column < binary.cols && pixels[column] != 0U) {
      ++column;
    }
    const Run run{first_column, column - 1};
    if (
      run.width() >= minimum_width_px &&
      run.width() <= maximum_width_px)
    {
      runs.push_back(run);
    }
  }
  return runs;
}

LaneCurve fitCurve(
  const std::vector<cv::Point2d> & input_points,
  const int minimum_points,
  const double maximum_residual_m)
{
  LaneCurve result;
  if (static_cast<int>(input_points.size()) < minimum_points) {
    return result;
  }

  std::vector<cv::Point2d> points = input_points;
  cv::Vec3d coefficients(0.0, 0.0, 0.0);
  for (int iteration = 0; iteration < 3; ++iteration) {
    if (static_cast<int>(points.size()) < minimum_points) {
      return result;
    }

    cv::Mat design(static_cast<int>(points.size()), 3, CV_64F);
    cv::Mat observations(static_cast<int>(points.size()), 1, CV_64F);
    for (std::size_t index = 0; index < points.size(); ++index) {
      const double x_m = points[index].x;
      design.at<double>(static_cast<int>(index), 0) = x_m * x_m;
      design.at<double>(static_cast<int>(index), 1) = x_m;
      design.at<double>(static_cast<int>(index), 2) = 1.0;
      observations.at<double>(static_cast<int>(index), 0) = points[index].y;
    }

    cv::Mat solved;
    if (!cv::solve(design, observations, solved, cv::DECOMP_SVD)) {
      return result;
    }
    coefficients = cv::Vec3d(
      solved.at<double>(0, 0),
      solved.at<double>(1, 0),
      solved.at<double>(2, 0));

    if (iteration == 2) {
      break;
    }

    std::vector<double> residuals;
    residuals.reserve(points.size());
    for (const auto & point : points) {
      const double predicted =
        coefficients[0] * point.x * point.x +
        coefficients[1] * point.x + coefficients[2];
      residuals.push_back(std::abs(point.y - predicted));
    }
    const double robust_scale = std::max(
      0.005, 1.4826 * median(residuals));
    const double cutoff = std::min(
      2.0 * maximum_residual_m,
      std::max(maximum_residual_m, 2.5 * robust_scale));
    std::vector<cv::Point2d> inliers;
    inliers.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      if (residuals[index] <= cutoff) {
        inliers.push_back(points[index]);
      }
    }
    if (inliers.size() == points.size()) {
      break;
    }
    points = std::move(inliers);
  }

  double squared_error = 0.0;
  double minimum_x_m = std::numeric_limits<double>::infinity();
  double maximum_x_m = -std::numeric_limits<double>::infinity();
  for (const auto & point : points) {
    const double error = point.y -
      (coefficients[0] * point.x * point.x +
      coefficients[1] * point.x + coefficients[2]);
    squared_error += error * error;
    minimum_x_m = std::min(minimum_x_m, point.x);
    maximum_x_m = std::max(maximum_x_m, point.x);
  }
  const double rms_error_m = std::sqrt(
    squared_error / static_cast<double>(points.size()));
  if (
    !std::isfinite(rms_error_m) ||
    rms_error_m > maximum_residual_m)
  {
    return result;
  }

  result.valid = true;
  result.coefficients = coefficients;
  result.point_count = static_cast<int>(points.size());
  result.minimum_observed_x_m = minimum_x_m;
  result.maximum_observed_x_m = maximum_x_m;
  result.rms_error_m = rms_error_m;
  return result;
}

void keepBetterSegment(TrackSegment segment, TrackSegment * best)
{
  if (
    segment.score() > best->score() ||
    (segment.score() == best->score() &&
    segment.center_points.size() > best->center_points.size()))
  {
    *best = std::move(segment);
  }
}

bool initializeTrack(
  const std::vector<Candidate> & candidates,
  const BevLaneReconstructorConfig & config,
  TrackSegment * segment)
{
  const double minimum_width_m =
    config.expected_lane_width_m - config.lane_width_tolerance_m;
  const double maximum_width_m =
    config.expected_lane_width_m + config.lane_width_tolerance_m;
  double best_score = std::numeric_limits<double>::infinity();
  bool found_pair = false;
  for (std::size_t first = 0; first < candidates.size(); ++first) {
    for (std::size_t second = first + 1U; second < candidates.size(); ++second) {
      const double left_m = std::max(
        candidates[first].lateral_m, candidates[second].lateral_m);
      const double right_m = std::min(
        candidates[first].lateral_m, candidates[second].lateral_m);
      const double width_m = left_m - right_m;
      const double center_m = 0.5 * (left_m + right_m);
      if (
        width_m < minimum_width_m || width_m > maximum_width_m ||
        std::abs(center_m) > config.initial_center_tolerance_m)
      {
        continue;
      }
      const double score =
        std::abs(center_m) +
        0.5 * std::abs(width_m - config.expected_lane_width_m);
      if (score < best_score) {
        best_score = score;
        segment->tracked_left_m = left_m;
        segment->tracked_right_m = right_m;
        found_pair = true;
      }
    }
  }
  if (found_pair) {
    segment->initialized = true;
    return true;
  }

  if (!config.allow_single_lane) {
    return false;
  }
  const double expected_left_m = 0.5 * config.expected_lane_width_m;
  const double expected_right_m = -0.5 * config.expected_lane_width_m;
  double best_error_m = std::numeric_limits<double>::infinity();
  bool single_is_left = false;
  double best_lateral_m = 0.0;
  for (const auto & candidate : candidates) {
    const double left_error_m =
      std::abs(candidate.lateral_m - expected_left_m);
    const double right_error_m =
      std::abs(candidate.lateral_m - expected_right_m);
    if (left_error_m < best_error_m) {
      best_error_m = left_error_m;
      best_lateral_m = candidate.lateral_m;
      single_is_left = true;
    }
    if (right_error_m < best_error_m) {
      best_error_m = right_error_m;
      best_lateral_m = candidate.lateral_m;
      single_is_left = false;
    }
  }
  if (best_error_m > config.single_lane_initial_tolerance_m) {
    return false;
  }

  segment->initialized = true;
  if (single_is_left) {
    segment->tracked_left_m = best_lateral_m;
    segment->tracked_right_m = best_lateral_m - config.expected_lane_width_m;
  } else {
    segment->tracked_right_m = best_lateral_m;
    segment->tracked_left_m = best_lateral_m + config.expected_lane_width_m;
  }
  return true;
}

bool updateWithPair(
  const std::vector<Candidate> & candidates,
  const double x_m,
  const int row,
  const BevLaneReconstructorConfig & config,
  TrackSegment * segment)
{
  const double minimum_width_m =
    config.expected_lane_width_m - config.lane_width_tolerance_m;
  const double maximum_width_m =
    config.expected_lane_width_m + config.lane_width_tolerance_m;
  const double row_factor = std::max(
    1.0,
    static_cast<double>(std::abs(segment->last_observed_row - row)) /
    static_cast<double>(std::max(1, config.row_step_px)));
  const double movement_limit_m = config.maximum_lateral_step_m * row_factor;
  double best_score = std::numeric_limits<double>::infinity();
  std::optional<std::pair<double, double>> best_pair;
  for (std::size_t first = 0; first < candidates.size(); ++first) {
    for (std::size_t second = first + 1U; second < candidates.size(); ++second) {
      const double left_m = std::max(
        candidates[first].lateral_m, candidates[second].lateral_m);
      const double right_m = std::min(
        candidates[first].lateral_m, candidates[second].lateral_m);
      const double width_m = left_m - right_m;
      if (
        width_m < minimum_width_m || width_m > maximum_width_m ||
        std::abs(left_m - segment->tracked_left_m) > movement_limit_m ||
        std::abs(right_m - segment->tracked_right_m) > movement_limit_m)
      {
        continue;
      }
      const double score =
        std::abs(left_m - segment->tracked_left_m) +
        std::abs(right_m - segment->tracked_right_m) +
        0.5 * std::abs(width_m - config.expected_lane_width_m);
      if (score < best_score) {
        best_score = score;
        best_pair = std::make_pair(left_m, right_m);
      }
    }
  }
  if (!best_pair.has_value()) {
    return false;
  }

  segment->tracked_left_m = best_pair->first;
  segment->tracked_right_m = best_pair->second;
  segment->center_points.emplace_back(
    x_m, 0.5 * (best_pair->first + best_pair->second));
  segment->pair_widths_m.push_back(best_pair->first - best_pair->second);
  segment->last_observed_row = row;
  segment->missing_distance_m = 0.0;
  return true;
}

bool updateWithSingle(
  const std::vector<Candidate> & candidates,
  const double x_m,
  const int row,
  const BevLaneReconstructorConfig & config,
  TrackSegment * segment)
{
  if (!config.allow_single_lane || candidates.empty()) {
    return false;
  }
  const double row_factor = std::max(
    1.0,
    static_cast<double>(std::abs(segment->last_observed_row - row)) /
    static_cast<double>(std::max(1, config.row_step_px)));
  const double movement_limit_m = config.maximum_lateral_step_m * row_factor;
  double best_error_m = std::numeric_limits<double>::infinity();
  double best_lateral_m = 0.0;
  bool is_left = false;
  for (const auto & candidate : candidates) {
    const double left_error_m =
      std::abs(candidate.lateral_m - segment->tracked_left_m);
    const double right_error_m =
      std::abs(candidate.lateral_m - segment->tracked_right_m);
    if (left_error_m <= movement_limit_m && left_error_m < best_error_m) {
      best_error_m = left_error_m;
      best_lateral_m = candidate.lateral_m;
      is_left = true;
    }
    if (right_error_m <= movement_limit_m && right_error_m < best_error_m) {
      best_error_m = right_error_m;
      best_lateral_m = candidate.lateral_m;
      is_left = false;
    }
  }
  if (!std::isfinite(best_error_m)) {
    return false;
  }

  if (is_left) {
    const double shift_m = best_lateral_m - segment->tracked_left_m;
    segment->tracked_left_m = best_lateral_m;
    segment->tracked_right_m += shift_m;
  } else {
    const double shift_m = best_lateral_m - segment->tracked_right_m;
    segment->tracked_right_m = best_lateral_m;
    segment->tracked_left_m += shift_m;
  }
  segment->center_points.emplace_back(
    x_m, 0.5 * (segment->tracked_left_m + segment->tracked_right_m));
  segment->last_observed_row = row;
  segment->missing_distance_m = 0.0;
  return true;
}

}  // namespace

double LaneCurve::lateralAt(const double x_m) const
{
  return
    coefficients[0] * x_m * x_m +
    coefficients[1] * x_m + coefficients[2];
}

double LaneCurve::derivativeAt(const double x_m) const
{
  return 2.0 * coefficients[0] * x_m + coefficients[1];
}

BevLaneReconstructor::BevLaneReconstructor(BevLaneReconstructorConfig config)
: config_(std::move(config))
{
  if (
    config_.x_min_m < 0.0 || config_.x_max_m <= config_.x_min_m ||
    config_.y_max_m <= config_.y_min_m || config_.meter_per_pixel <= 0.0 ||
    config_.image_width <= 0 || config_.image_height <= 0 ||
    config_.minimum_brightness < 0 || config_.minimum_brightness > 255 ||
    config_.maximum_saturation < 0 || config_.maximum_saturation > 255 ||
    config_.brightness_blur_kernel <= 0 ||
    config_.brightness_blur_kernel % 2 == 0 ||
    config_.vertical_close_m <= 0.0 ||
    config_.minimum_lane_mark_width_m <= 0.0 ||
    config_.maximum_lane_mark_width_m < config_.minimum_lane_mark_width_m ||
    config_.row_step_px <= 0 ||
    config_.observation_minimum_x_m < config_.x_min_m ||
    config_.observation_maximum_x_m <= config_.observation_minimum_x_m ||
    config_.observation_maximum_x_m > config_.x_max_m ||
    config_.reconstruction_minimum_x_m < config_.x_min_m ||
    config_.reconstruction_maximum_x_m <= config_.reconstruction_minimum_x_m ||
    config_.reconstruction_maximum_x_m > config_.x_max_m ||
    config_.maximum_extrapolation_m < 0.0 ||
    config_.expected_lane_width_m <= 0.0 ||
    config_.lane_width_tolerance_m <= 0.0 ||
    config_.lane_width_tolerance_m >= config_.expected_lane_width_m ||
    config_.initial_center_tolerance_m <= 0.0 ||
    config_.single_lane_initial_tolerance_m <= 0.0 ||
    config_.maximum_lateral_step_m <= 0.0 ||
    config_.maximum_tracking_gap_m <= 0.0 || config_.minimum_points < 3 ||
    config_.maximum_fit_residual_m <= 0.0 ||
    config_.output_line_thickness_m <= 0.0 ||
    config_.temporal_smoothing_alpha <= 0.0 ||
    config_.temporal_smoothing_alpha > 1.0 ||
    config_.maximum_temporal_jump_m <= 0.0)
  {
    throw std::invalid_argument("invalid BEV lane reconstruction configuration");
  }
  const int expected_width = static_cast<int>(std::llround(
      (config_.y_max_m - config_.y_min_m) / config_.meter_per_pixel));
  const int expected_height = static_cast<int>(std::llround(
      (config_.x_max_m - config_.x_min_m) / config_.meter_per_pixel));
  if (
    config_.image_width != expected_width ||
    config_.image_height != expected_height)
  {
    throw std::invalid_argument(
            "lane reconstruction dimensions do not match BEV bounds");
  }
}

BevLaneReconstruction BevLaneReconstructor::reconstruct(const cv::Mat & bev_bgr)
{
  if (
    bev_bgr.empty() || bev_bgr.type() != CV_8UC3 ||
    bev_bgr.cols != config_.image_width ||
    bev_bgr.rows != config_.image_height)
  {
    throw std::invalid_argument(
            "lane reconstruction expects the configured CV_8UC3 BEV image");
  }

  BevLaneReconstruction result;
  cv::Mat gray;
  cv::cvtColor(bev_bgr, gray, cv::COLOR_BGR2GRAY);
  if (config_.brightness_blur_kernel > 1) {
    cv::GaussianBlur(
      gray, gray,
      {config_.brightness_blur_kernel, config_.brightness_blur_kernel}, 0.0);
  }
  cv::threshold(
    gray, result.candidate_mask, config_.minimum_brightness, 255,
    cv::THRESH_BINARY);

  if (config_.maximum_saturation < 255) {
    cv::Mat hsv;
    cv::cvtColor(bev_bgr, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    cv::Mat low_saturation;
    cv::threshold(
      channels[1], low_saturation, config_.maximum_saturation, 255,
      cv::THRESH_BINARY_INV);
    cv::bitwise_and(
      result.candidate_mask, low_saturation, result.candidate_mask);
  }

  const int vertical_close_px = makeOdd(static_cast<int>(std::lround(
      config_.vertical_close_m / config_.meter_per_pixel)));
  if (vertical_close_px > 1) {
    const cv::Mat close_kernel = cv::getStructuringElement(
      cv::MORPH_RECT, {1, vertical_close_px});
    cv::morphologyEx(
      result.candidate_mask, result.candidate_mask,
      cv::MORPH_CLOSE, close_kernel);
  }

  const int minimum_run_width_px = std::max(
    1, static_cast<int>(std::lround(
      config_.minimum_lane_mark_width_m / config_.meter_per_pixel)));
  const int maximum_run_width_px = std::max(
    minimum_run_width_px,
    static_cast<int>(std::lround(
      config_.maximum_lane_mark_width_m / config_.meter_per_pixel)));
  const auto xAtRow = [this](const int row) {
      return config_.x_max_m -
             (static_cast<double>(row) + 0.5) * config_.meter_per_pixel;
    };
  const auto yAtColumn = [this](const double column) {
      return config_.y_max_m -
             (column + 0.5) * config_.meter_per_pixel;
    };
  const auto rowAtX = [this](const double x_m) {
      return static_cast<int>(std::lround(
        (config_.x_max_m - x_m) / config_.meter_per_pixel - 0.5));
    };

  const int near_row = std::clamp(
    rowAtX(config_.observation_minimum_x_m), 0, config_.image_height - 1);
  const int far_row = std::clamp(
    rowAtX(config_.observation_maximum_x_m), 0, config_.image_height - 1);
  TrackSegment current;
  TrackSegment best;
  for (
    int row = near_row;
    row >= far_row;
    row -= config_.row_step_px)
  {
    std::vector<Candidate> candidates;
    for (const auto & run : findRuns(
        result.candidate_mask, row,
        minimum_run_width_px, maximum_run_width_px))
    {
      candidates.push_back(Candidate{yAtColumn(run.centerColumn())});
    }

    if (!current.initialized) {
      if (!initializeTrack(candidates, config_, &current)) {
        continue;
      }
      current.last_observed_row = row;
    }

    const double x_m = xAtRow(row);
    if (
      updateWithPair(candidates, x_m, row, config_, &current) ||
      updateWithSingle(candidates, x_m, row, config_, &current))
    {
      continue;
    }

    current.missing_distance_m +=
      static_cast<double>(config_.row_step_px) * config_.meter_per_pixel;
    if (current.missing_distance_m > config_.maximum_tracking_gap_m) {
      keepBetterSegment(std::move(current), &best);
      current = TrackSegment{};
    }
  }
  keepBetterSegment(std::move(current), &best);

  result.center_point_count = static_cast<int>(best.center_points.size());
  result.measured_lane_width_m = best.pair_widths_m.empty() ?
    config_.expected_lane_width_m : median(best.pair_widths_m);
  result.center_curve = fitCurve(
    best.center_points, config_.minimum_points,
    config_.maximum_fit_residual_m);
  result.reconstructed_mask = cv::Mat::zeros(
    config_.image_height, config_.image_width, CV_8UC1);
  if (!result.center_curve.valid) {
    return result;
  }

  if (
    previous_center_curve_.valid &&
    config_.temporal_smoothing_alpha < 1.0)
  {
    const double check_near_x_m = result.center_curve.minimum_observed_x_m;
    const double check_far_x_m = result.center_curve.maximum_observed_x_m;
    const double maximum_jump_m = std::max(
      std::abs(
        result.center_curve.lateralAt(check_near_x_m) -
        previous_center_curve_.lateralAt(check_near_x_m)),
      std::abs(
        result.center_curve.lateralAt(check_far_x_m) -
        previous_center_curve_.lateralAt(check_far_x_m)));
    if (maximum_jump_m <= config_.maximum_temporal_jump_m) {
      result.center_curve.coefficients =
        config_.temporal_smoothing_alpha * result.center_curve.coefficients +
        (1.0 - config_.temporal_smoothing_alpha) *
        previous_center_curve_.coefficients;
    }
  }
  previous_center_curve_ = result.center_curve;

  const double reconstruction_start_x_m = std::max(
    config_.reconstruction_minimum_x_m,
    result.center_curve.minimum_observed_x_m);
  const double reconstruction_end_x_m = std::min(
    config_.reconstruction_maximum_x_m,
    result.center_curve.maximum_observed_x_m +
    config_.maximum_extrapolation_m);
  if (reconstruction_end_x_m <= reconstruction_start_x_m) {
    return result;
  }

  const auto pointAtMetric = [this](const double x_m, const double y_m) {
      const int row = static_cast<int>(std::lround(
        (config_.x_max_m - x_m) / config_.meter_per_pixel - 0.5));
      const int column = static_cast<int>(std::lround(
        (config_.y_max_m - y_m) / config_.meter_per_pixel - 0.5));
      return cv::Point(column, row);
    };
  const auto pointInside = [this](const cv::Point & point) {
      return
        point.x >= 0 && point.x < config_.image_width &&
        point.y >= 0 && point.y < config_.image_height;
    };
  const int thickness_px = std::max(
    1, static_cast<int>(std::lround(
      config_.output_line_thickness_m / config_.meter_per_pixel)));
  // Same-X separation grows slightly on a curve, so use the known physical
  // track width for the normal offset and keep the measured width for
  // diagnostics only.
  const double half_lane_width_m = 0.5 * config_.expected_lane_width_m;
  std::optional<cv::Point> previous_left;
  std::optional<cv::Point> previous_right;
  for (
    double x_m = reconstruction_start_x_m;
    x_m <= reconstruction_end_x_m + 0.5 * config_.meter_per_pixel;
    x_m += config_.meter_per_pixel)
  {
    const double center_y_m = result.center_curve.lateralAt(x_m);
    const double derivative = result.center_curve.derivativeAt(x_m);
    const double normal_length = std::sqrt(1.0 + derivative * derivative);
    const double normal_x_m = -derivative / normal_length;
    const double normal_y_m = 1.0 / normal_length;
    const cv::Point left = pointAtMetric(
      x_m + half_lane_width_m * normal_x_m,
      center_y_m + half_lane_width_m * normal_y_m);
    const cv::Point right = pointAtMetric(
      x_m - half_lane_width_m * normal_x_m,
      center_y_m - half_lane_width_m * normal_y_m);

    if (pointInside(left)) {
      if (previous_left.has_value()) {
        cv::line(
          result.reconstructed_mask, *previous_left, left,
          cv::Scalar(255), thickness_px, cv::LINE_8);
      }
      previous_left = left;
    } else {
      previous_left.reset();
    }
    if (pointInside(right)) {
      if (previous_right.has_value()) {
        cv::line(
          result.reconstructed_mask, *previous_right, right,
          cv::Scalar(255), thickness_px, cv::LINE_8);
      }
      previous_right = right;
    } else {
      previous_right.reset();
    }
  }

  result.valid = cv::countNonZero(result.reconstructed_mask) > 0;
  result.reconstructed_maximum_x_m = reconstruction_end_x_m;
  return result;
}

void BevLaneReconstructor::reset()
{
  previous_center_curve_ = LaneCurve{};
}

}  // namespace bev_processor
