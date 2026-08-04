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

struct Seed
{
  bool valid{false};
  double x_m{0.0};
  cv::Point2d left;
  cv::Point2d right;
  bool left_measured{false};
  bool right_measured{false};
};

struct WindowMeasurement
{
  bool valid{false};
  cv::Point2d point;
  int pixel_count{0};
};

struct LaneTracker
{
  bool active{false};
  cv::Point2d position;
  cv::Point2d tangent{1.0, 0.0};
  std::vector<cv::Point2d> measured_points;
  int missing_windows{0};
};

int makeOdd(const int value)
{
  const int positive = std::max(1, value);
  return positive % 2 == 0 ? positive + 1 : positive;
}

double norm(const cv::Point2d & vector)
{
  return std::hypot(vector.x, vector.y);
}

cv::Point2d normalized(
  const cv::Point2d & vector,
  const cv::Point2d & fallback = cv::Point2d(1.0, 0.0))
{
  const double length = norm(vector);
  return length > 1.0e-9 ? vector * (1.0 / length) : fallback;
}

double dot(const cv::Point2d & first, const cv::Point2d & second)
{
  return first.x * second.x + first.y * second.y;
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

double xAtRow(
  const BevLaneReconstructorConfig & config,
  const double row)
{
  return config.x_max_m - (row + 0.5) * config.meter_per_pixel;
}

double yAtColumn(
  const BevLaneReconstructorConfig & config,
  const double column)
{
  return config.y_max_m - (column + 0.5) * config.meter_per_pixel;
}

int rowAtX(
  const BevLaneReconstructorConfig & config,
  const double x_m)
{
  return static_cast<int>(std::lround(
    (config.x_max_m - x_m) / config.meter_per_pixel - 0.5));
}

int columnAtY(
  const BevLaneReconstructorConfig & config,
  const double y_m)
{
  return static_cast<int>(std::lround(
    (config.y_max_m - y_m) / config.meter_per_pixel - 0.5));
}

cv::Point imagePoint(
  const BevLaneReconstructorConfig & config,
  const cv::Point2d & metric_point)
{
  return cv::Point(
    columnAtY(config, metric_point.y),
    rowAtX(config, metric_point.x));
}

bool imagePointInside(
  const BevLaneReconstructorConfig & config,
  const cv::Point & point)
{
  return
    point.x >= 0 && point.x < config.image_width &&
    point.y >= 0 && point.y < config.image_height;
}

double farBlend(
  const BevLaneReconstructorConfig & config,
  const double x_m)
{
  const double denominator = std::max(
    config.meter_per_pixel,
    config.reconstruction_maximum_x_m - config.observation_maximum_x_m);
  return std::clamp(
    (x_m - config.observation_maximum_x_m) / denominator,
    0.0,
    1.0);
}

int brightnessThresholdAt(
  const BevLaneReconstructorConfig & config,
  const double x_m)
{
  const double blend = farBlend(config, x_m);
  return static_cast<int>(std::lround(
    (1.0 - blend) * static_cast<double>(config.minimum_brightness) +
    blend * static_cast<double>(config.far_minimum_brightness)));
}

Seed findSeed(
  const cv::Mat & candidate_mask,
  const BevLaneReconstructorConfig & config)
{
  const int minimum_run_width_px = std::max(
    1, static_cast<int>(std::lround(
      config.minimum_lane_mark_width_m / config.meter_per_pixel)));
  const int maximum_run_width_px = std::max(
    minimum_run_width_px,
    static_cast<int>(std::lround(
      config.maximum_lane_mark_width_m / config.meter_per_pixel)));
  const double initialization_maximum_x_m = std::min(
    config.observation_maximum_x_m,
    config.observation_minimum_x_m + 0.80);
  const int near_row = std::clamp(
    rowAtX(config, config.observation_minimum_x_m),
    0, config.image_height - 1);
  const int far_row = std::clamp(
    rowAtX(config, initialization_maximum_x_m),
    0, config.image_height - 1);
  const double minimum_lane_width_m =
    config.expected_lane_width_m - config.lane_width_tolerance_m;
  const double maximum_lane_width_m =
    config.expected_lane_width_m + config.lane_width_tolerance_m;

  Seed best_pair;
  double best_pair_score = std::numeric_limits<double>::infinity();
  Seed best_single;
  double best_single_score = std::numeric_limits<double>::infinity();
  for (
    int row = near_row;
    row >= far_row;
    row -= config.row_step_px)
  {
    const double x_m = xAtRow(config, row);
    const auto runs = findRuns(
      candidate_mask, row, minimum_run_width_px, maximum_run_width_px);
    std::vector<double> laterals_m;
    laterals_m.reserve(runs.size());
    for (const auto & run : runs) {
      laterals_m.push_back(yAtColumn(config, run.centerColumn()));
    }

    for (std::size_t first = 0; first < laterals_m.size(); ++first) {
      for (
        std::size_t second = first + 1U;
        second < laterals_m.size();
        ++second)
      {
        const double left_m = std::max(
          laterals_m[first], laterals_m[second]);
        const double right_m = std::min(
          laterals_m[first], laterals_m[second]);
        const double width_m = left_m - right_m;
        const double center_m = 0.5 * (left_m + right_m);
        if (
          width_m < minimum_lane_width_m ||
          width_m > maximum_lane_width_m ||
          std::abs(center_m) > config.initial_center_tolerance_m)
        {
          continue;
        }
        const double score =
          std::abs(center_m) +
          0.5 * std::abs(width_m - config.expected_lane_width_m) +
          (x_m - config.observation_minimum_x_m);
        if (score < best_pair_score) {
          best_pair_score = score;
          best_pair.valid = true;
          best_pair.x_m = x_m;
          best_pair.left = cv::Point2d(x_m, left_m);
          best_pair.right = cv::Point2d(x_m, right_m);
          best_pair.left_measured = true;
          best_pair.right_measured = true;
        }
      }
    }

    if (config.allow_single_lane) {
      const double expected_left_m = 0.5 * config.expected_lane_width_m;
      const double expected_right_m = -0.5 * config.expected_lane_width_m;
      for (const double lateral_m : laterals_m) {
        const double left_error_m = std::abs(lateral_m - expected_left_m);
        const double right_error_m = std::abs(lateral_m - expected_right_m);
        const double lateral_error_m = std::min(
          left_error_m, right_error_m);
        const double score = lateral_error_m +
          (x_m - config.observation_minimum_x_m);
        if (
          lateral_error_m > config.single_lane_initial_tolerance_m ||
          score >= best_single_score)
        {
          continue;
        }
        best_single_score = score;
        best_single.valid = true;
        best_single.x_m = x_m;
        if (left_error_m <= right_error_m) {
          best_single.left = cv::Point2d(x_m, lateral_m);
          best_single.right = cv::Point2d(
            x_m, lateral_m - config.expected_lane_width_m);
          best_single.left_measured = true;
          best_single.right_measured = false;
        } else {
          best_single.right = cv::Point2d(x_m, lateral_m);
          best_single.left = cv::Point2d(
            x_m, lateral_m + config.expected_lane_width_m);
          best_single.left_measured = false;
          best_single.right_measured = true;
        }
      }
    }
  }
  return best_pair.valid ? best_pair : best_single;
}

WindowMeasurement measureWindow(
  const cv::Mat & candidate_mask,
  const cv::Mat & gray,
  const BevLaneReconstructorConfig & config,
  const cv::Point2d & prediction,
  const cv::Point2d & tangent,
  const double half_width_m,
  const int missing_windows)
{
  WindowMeasurement result;
  const double half_length_m = 0.5 * config.sliding_window_length_m;
  const double expanded_half_width_m = half_width_m *
    (1.0 + 0.25 * static_cast<double>(missing_windows));
  const double search_radius_m = half_length_m + expanded_half_width_m;
  const int minimum_row = std::clamp(
    rowAtX(config, prediction.x + search_radius_m),
    0, config.image_height - 1);
  const int maximum_row = std::clamp(
    rowAtX(config, prediction.x - search_radius_m),
    0, config.image_height - 1);
  const int minimum_column = std::clamp(
    columnAtY(config, prediction.y + search_radius_m),
    0, config.image_width - 1);
  const int maximum_column = std::clamp(
    columnAtY(config, prediction.y - search_radius_m),
    0, config.image_width - 1);
  const cv::Point2d normal(-tangent.y, tangent.x);
  double total_weight = 0.0;
  cv::Point2d weighted_sum(0.0, 0.0);
  for (int row = minimum_row; row <= maximum_row; ++row) {
    const auto * candidate = candidate_mask.ptr<std::uint8_t>(row);
    const auto * brightness = gray.ptr<std::uint8_t>(row);
    const double x_m = xAtRow(config, row);
    for (int column = minimum_column; column <= maximum_column; ++column) {
      if (candidate[column] == 0U) {
        continue;
      }
      const cv::Point2d point(x_m, yAtColumn(config, column));
      const cv::Point2d delta = point - prediction;
      const double longitudinal_m = dot(delta, tangent);
      const double lateral_m = dot(delta, normal);
      // Keep only a small overlap behind the predicted center. A symmetric
      // window repeatedly re-measures the previous tape pixels and can stall
      // instead of advancing along the lane.
      if (
        longitudinal_m < -0.25 * config.sliding_window_step_m ||
        longitudinal_m > half_length_m ||
        std::abs(lateral_m) > expanded_half_width_m)
      {
        continue;
      }
      const double weight =
        1.0 + static_cast<double>(brightness[column]) / 255.0;
      weighted_sum += point * weight;
      total_weight += weight;
      ++result.pixel_count;
    }
  }
  if (
    result.pixel_count < config.minimum_window_pixel_count ||
    total_weight <= 0.0)
  {
    return result;
  }
  result.point = weighted_sum * (1.0 / total_weight);
  result.valid = true;
  return result;
}

void updateTracker(
  LaneTracker * tracker,
  const cv::Point2d & prediction,
  const WindowMeasurement & measurement,
  const BevLaneReconstructorConfig & config)
{
  if (!tracker->active) {
    return;
  }
  if (!measurement.valid) {
    tracker->position = prediction;
    ++tracker->missing_windows;
    const int maximum_missing_windows = std::max(
      1, static_cast<int>(std::ceil(
        config.maximum_tracking_gap_m / config.sliding_window_step_m)));
    if (tracker->missing_windows > maximum_missing_windows) {
      tracker->active = false;
    }
    return;
  }

  const cv::Point2d previous_position = tracker->position;
  const double measurement_weight = config.sliding_window_measurement_weight;
  tracker->position =
    measurement_weight * measurement.point +
    (1.0 - measurement_weight) * prediction;
  cv::Point2d measured_direction = normalized(
    measurement.point - previous_position, tracker->tangent);
  if (dot(measured_direction, tracker->tangent) < 0.0) {
    measured_direction *= -1.0;
  }
  const double heading_weight = config.sliding_window_heading_weight;
  tracker->tangent = normalized(
    heading_weight * measured_direction +
    (1.0 - heading_weight) * tracker->tangent,
    tracker->tangent);
  tracker->measured_points.push_back(measurement.point);
  tracker->missing_windows = 0;
}

std::vector<cv::Point2d> smoothMeasuredPoints(
  const std::vector<cv::Point2d> & points,
  const double measured_weight)
{
  if (points.size() < 3U) {
    return points;
  }
  std::vector<cv::Point2d> smoothed = points;
  const double neighbor_weight = 0.5 * (1.0 - measured_weight);
  for (std::size_t index = 1; index + 1U < points.size(); ++index) {
    smoothed[index] =
      measured_weight * points[index] +
      neighbor_weight * points[index - 1U] +
      neighbor_weight * points[index + 1U];
  }
  return smoothed;
}

std::vector<cv::Point2d> extendByTangent(
  std::vector<cv::Point2d> points,
  const BevLaneReconstructorConfig & config)
{
  if (points.size() < 2U || config.maximum_extrapolation_m <= 0.0) {
    return points;
  }
  const cv::Point2d tangent = normalized(
    points.back() - points[points.size() - 2U]);
  const cv::Point2d start = points.back();
  for (
    double distance_m = config.meter_per_pixel;
    distance_m <= config.maximum_extrapolation_m + 1.0e-9;
    distance_m += config.meter_per_pixel)
  {
    const cv::Point2d point = start + distance_m * tangent;
    if (
      point.x < config.reconstruction_minimum_x_m ||
      point.x > config.reconstruction_maximum_x_m ||
      point.y < config.y_min_m || point.y > config.y_max_m)
    {
      break;
    }
    points.push_back(point);
  }
  return points;
}

std::vector<cv::Point2d> offsetLane(
  const std::vector<cv::Point2d> & reference,
  const double offset_m)
{
  std::vector<cv::Point2d> offset;
  offset.reserve(reference.size());
  for (std::size_t index = 0; index < reference.size(); ++index) {
    cv::Point2d tangent;
    if (index == 0U && reference.size() > 1U) {
      tangent = reference[1U] - reference[0U];
    } else if (index + 1U == reference.size() && index > 0U) {
      tangent = reference[index] - reference[index - 1U];
    } else if (index > 0U && index + 1U < reference.size()) {
      tangent = reference[index + 1U] - reference[index - 1U];
    } else {
      tangent = cv::Point2d(1.0, 0.0);
    }
    tangent = normalized(tangent);
    const cv::Point2d left_normal(-tangent.y, tangent.x);
    offset.push_back(reference[index] + offset_m * left_normal);
  }
  return offset;
}

double drawMeasuredLane(
  const std::vector<cv::Point2d> & points,
  const BevLaneReconstructorConfig & config,
  cv::Mat * mask)
{
  if (points.empty()) {
    return 0.0;
  }
  const int thickness_px = std::max(
    1, static_cast<int>(std::lround(
      config.output_line_thickness_m / config.meter_per_pixel)));
  std::optional<cv::Point> previous_image_point;
  std::optional<cv::Point2d> previous_metric_point;
  double maximum_x_m = 0.0;
  for (const auto & point : points) {
    const cv::Point pixel = imagePoint(config, point);
    if (!imagePointInside(config, pixel)) {
      previous_image_point.reset();
      previous_metric_point.reset();
      continue;
    }
    maximum_x_m = std::max(maximum_x_m, point.x);
    if (
      previous_image_point.has_value() &&
      previous_metric_point.has_value() &&
      norm(point - *previous_metric_point) <= config.maximum_gap_fill_m)
    {
      cv::line(
        *mask, *previous_image_point, pixel,
        cv::Scalar(255), thickness_px, cv::LINE_8);
    } else {
      cv::circle(
        *mask, pixel, std::max(1, thickness_px / 2),
        cv::Scalar(255), cv::FILLED, cv::LINE_8);
    }
    previous_image_point = pixel;
    previous_metric_point = point;
  }
  return maximum_x_m;
}

}  // namespace

BevLaneReconstructor::BevLaneReconstructor(BevLaneReconstructorConfig config)
: config_(std::move(config))
{
  if (
    config_.x_min_m < 0.0 || config_.x_max_m <= config_.x_min_m ||
    config_.y_max_m <= config_.y_min_m || config_.meter_per_pixel <= 0.0 ||
    config_.image_width <= 0 || config_.image_height <= 0 ||
    config_.minimum_brightness < 0 || config_.minimum_brightness > 255 ||
    config_.far_minimum_brightness < 0 ||
    config_.far_minimum_brightness > 255 ||
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
    config_.reconstruction_maximum_x_m <= config_.observation_maximum_x_m ||
    config_.reconstruction_maximum_x_m > config_.x_max_m ||
    config_.maximum_extrapolation_m < 0.0 ||
    config_.sliding_window_step_m <= 0.0 ||
    config_.sliding_window_length_m <= 0.0 ||
    config_.sliding_window_half_width_near_m <= 0.0 ||
    config_.sliding_window_half_width_far_m <
    config_.sliding_window_half_width_near_m ||
    config_.sliding_window_measurement_weight <= 0.0 ||
    config_.sliding_window_measurement_weight > 1.0 ||
    config_.sliding_window_heading_weight <= 0.0 ||
    config_.sliding_window_heading_weight > 1.0 ||
    config_.maximum_tracking_arc_length_m <= 0.0 ||
    config_.maximum_gap_fill_m <= 0.0 ||
    config_.measured_point_smoothing_weight <= 0.0 ||
    config_.measured_point_smoothing_weight > 1.0 ||
    config_.minimum_window_pixel_count <= 0 ||
    config_.expected_lane_width_m <= 0.0 ||
    config_.lane_width_tolerance_m <= 0.0 ||
    config_.lane_width_tolerance_m >= config_.expected_lane_width_m ||
    config_.initial_center_tolerance_m <= 0.0 ||
    config_.single_lane_initial_tolerance_m <= 0.0 ||
    config_.maximum_tracking_gap_m <= 0.0 || config_.minimum_points < 2 ||
    config_.output_line_thickness_m <= 0.0)
  {
    throw std::invalid_argument(
            "invalid BEV sliding-window lane configuration");
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

  cv::Mat saturation;
  if (config_.maximum_saturation < 255) {
    cv::Mat hsv;
    cv::cvtColor(bev_bgr, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);
    saturation = channels[1];
  }
  result.candidate_mask = cv::Mat::zeros(
    config_.image_height, config_.image_width, CV_8UC1);
  for (int row = 0; row < config_.image_height; ++row) {
    const double x_m = xAtRow(config_, row);
    const int brightness_threshold = brightnessThresholdAt(config_, x_m);
    const auto * brightness = gray.ptr<std::uint8_t>(row);
    const auto * saturation_row = saturation.empty() ?
      nullptr : saturation.ptr<std::uint8_t>(row);
    auto * candidate = result.candidate_mask.ptr<std::uint8_t>(row);
    for (int column = 0; column < config_.image_width; ++column) {
      const bool bright_enough = brightness[column] > brightness_threshold;
      const bool saturation_ok =
        saturation_row == nullptr ||
        saturation_row[column] <= config_.maximum_saturation;
      candidate[column] = bright_enough && saturation_ok ? 255U : 0U;
    }
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

  result.reconstructed_mask = cv::Mat::zeros(
    config_.image_height, config_.image_width, CV_8UC1);
  const Seed seed = findSeed(result.candidate_mask, config_);
  if (!seed.valid) {
    return result;
  }

  LaneTracker left;
  left.active = true;
  left.position = seed.left;
  if (seed.left_measured) {
    left.measured_points.push_back(seed.left);
  }
  LaneTracker right;
  right.active = true;
  right.position = seed.right;
  if (seed.right_measured) {
    right.measured_points.push_back(seed.right);
  }
  std::vector<double> measured_widths_m;
  if (seed.left_measured && seed.right_measured) {
    measured_widths_m.push_back(norm(seed.left - seed.right));
  }

  const int maximum_steps = std::max(
    1, static_cast<int>(std::ceil(
      config_.maximum_tracking_arc_length_m /
      config_.sliding_window_step_m)));
  for (int step = 0; step < maximum_steps; ++step) {
    if (!left.active && !right.active) {
      break;
    }

    const cv::Point2d left_prediction =
      left.position + config_.sliding_window_step_m * left.tangent;
    const cv::Point2d right_prediction =
      right.position + config_.sliding_window_step_m * right.tangent;
    const double left_half_width_m =
      (1.0 - farBlend(config_, left_prediction.x)) *
      config_.sliding_window_half_width_near_m +
      farBlend(config_, left_prediction.x) *
      config_.sliding_window_half_width_far_m;
    const double right_half_width_m =
      (1.0 - farBlend(config_, right_prediction.x)) *
      config_.sliding_window_half_width_near_m +
      farBlend(config_, right_prediction.x) *
      config_.sliding_window_half_width_far_m;

    WindowMeasurement left_measurement;
    if (left.active) {
      left_measurement = measureWindow(
        result.candidate_mask, gray, config_,
        left_prediction, left.tangent,
        left_half_width_m, left.missing_windows);
    }
    WindowMeasurement right_measurement;
    if (right.active) {
      right_measurement = measureWindow(
        result.candidate_mask, gray, config_,
        right_prediction, right.tangent,
        right_half_width_m, right.missing_windows);
    }

    if (left_measurement.valid && right_measurement.valid) {
      const double measured_width_m = norm(
        left_measurement.point - right_measurement.point);
      const double minimum_width_m =
        config_.expected_lane_width_m - config_.lane_width_tolerance_m;
      const double maximum_width_m =
        config_.expected_lane_width_m + config_.lane_width_tolerance_m;
      if (
        measured_width_m >= minimum_width_m &&
        measured_width_m <= maximum_width_m)
      {
        measured_widths_m.push_back(measured_width_m);
      }
    }

    updateTracker(
      &left, left_prediction, left_measurement, config_);
    updateTracker(
      &right, right_prediction, right_measurement, config_);
    if (left_measurement.valid && !right_measurement.valid && right.active) {
      right.tangent = normalized(
        0.5 * right.tangent + 0.5 * left.tangent, right.tangent);
    } else if (
      right_measurement.valid && !left_measurement.valid && left.active)
    {
      left.tangent = normalized(
        0.5 * left.tangent + 0.5 * right.tangent, left.tangent);
    }

    const auto outsideTrackingArea = [this](const LaneTracker & tracker) {
        return
          tracker.position.x > config_.reconstruction_maximum_x_m ||
          tracker.position.x < config_.reconstruction_minimum_x_m - 0.10 ||
          tracker.position.y > config_.y_max_m + 0.10 ||
          tracker.position.y < config_.y_min_m - 0.10;
      };
    if (left.active && outsideTrackingArea(left)) {
      left.active = false;
    }
    if (right.active && outsideTrackingArea(right)) {
      right.active = false;
    }
  }

  result.left_measured_points = left.measured_points;
  result.right_measured_points = right.measured_points;
  result.measured_point_count = static_cast<int>(
    left.measured_points.size() + right.measured_points.size());
  result.measured_lane_width_m = measured_widths_m.empty() ?
    config_.expected_lane_width_m : median(measured_widths_m);

  const bool left_valid =
    static_cast<int>(left.measured_points.size()) >= config_.minimum_points;
  const bool right_valid =
    static_cast<int>(right.measured_points.size()) >= config_.minimum_points;
  if (!left_valid && !right_valid) {
    return result;
  }

  std::vector<cv::Point2d> left_output;
  std::vector<cv::Point2d> right_output;
  if (left_valid) {
    left_output = smoothMeasuredPoints(
      left.measured_points, config_.measured_point_smoothing_weight);
  }
  if (right_valid) {
    right_output = smoothMeasuredPoints(
      right.measured_points, config_.measured_point_smoothing_weight);
  }
  if (left_valid && !right_valid && config_.allow_single_lane) {
    right_output = offsetLane(left_output, -config_.expected_lane_width_m);
  } else if (right_valid && !left_valid && config_.allow_single_lane) {
    left_output = offsetLane(right_output, config_.expected_lane_width_m);
  }

  left_output = extendByTangent(std::move(left_output), config_);
  right_output = extendByTangent(std::move(right_output), config_);
  result.reconstructed_maximum_x_m = std::max(
    drawMeasuredLane(left_output, config_, &result.reconstructed_mask),
    drawMeasuredLane(right_output, config_, &result.reconstructed_mask));
  result.valid = cv::countNonZero(result.reconstructed_mask) > 0;
  return result;
}

void BevLaneReconstructor::reset()
{
  // The pixel-first sliding-window tracker intentionally keeps no temporal
  // curve state. Every frame is re-anchored to the current BEV pixels.
}

}  // namespace bev_processor
