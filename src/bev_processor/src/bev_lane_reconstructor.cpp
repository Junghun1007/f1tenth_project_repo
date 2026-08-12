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
  double selection_score_m{std::numeric_limits<double>::infinity()};
  double local_contrast{0.0};
};

struct LateralBin
{
  int pixel_count{0};
  double total_weight{0.0};
  double total_brightness{0.0};
  cv::Point2d weighted_sum{0.0, 0.0};
  double minimum_lateral_m{std::numeric_limits<double>::infinity()};
  double maximum_lateral_m{-std::numeric_limits<double>::infinity()};
};

struct LaneTracker
{
  bool active{false};
  cv::Point2d position;
  cv::Point2d tangent{1.0, 0.0};
  std::vector<cv::Point2d> measured_points;
  int missing_windows{0};
};

struct TemporalTrackResult
{
  std::vector<cv::Point2d> points;
  bool from_current_frame{false};
  bool held_previous_frame{false};
};

struct CorrespondingLaneResult
{
  std::vector<cv::Point2d> points;
  int inferred_point_count{0};
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

BevLaneReconstruction::SlidingWindow makeSlidingWindowDebug(
  const cv::Point2d & prediction,
  const cv::Point2d & tangent,
  const double half_width_m,
  const int missing_windows,
  const double sliding_window_length_m,
  const double sliding_window_step_m,
  const bool left_lane,
  const bool measurement_found)
{
  const cv::Point2d unit_tangent = normalized(tangent);
  const cv::Point2d normal(-unit_tangent.y, unit_tangent.x);
  const double expanded_half_width_m = half_width_m *
    (1.0 + 0.25 * static_cast<double>(missing_windows));
  const double minimum_longitudinal_m = -0.25 * sliding_window_step_m;
  const double maximum_longitudinal_m = 0.5 * sliding_window_length_m;
  const cv::Point2d near_center =
    prediction + minimum_longitudinal_m * unit_tangent;
  const cv::Point2d far_center =
    prediction + maximum_longitudinal_m * unit_tangent;

  BevLaneReconstruction::SlidingWindow window;
  window.corners = {
    near_center + expanded_half_width_m * normal,
    far_center + expanded_half_width_m * normal,
    far_center - expanded_half_width_m * normal,
    near_center - expanded_half_width_m * normal};
  window.left_lane = left_lane;
  window.measurement_found = measurement_found;
  return window;
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

double appearanceBlend(
  const BevLaneReconstructorConfig & config,
  const double x_m)
{
  const double range_m = std::max(
    config.meter_per_pixel,
    config.observation_maximum_x_m - config.observation_minimum_x_m);
  return std::clamp(
    (x_m - config.observation_minimum_x_m) / range_m,
    0.0,
    1.0);
}

bool hasDarkLocalBackground(
  const cv::Mat & gray,
  const int row,
  const Run & run,
  const BevLaneReconstructorConfig & config)
{
  double center_total = 0.0;
  int center_count = 0;
  const auto * brightness = gray.ptr<std::uint8_t>(row);
  for (int column = run.first_column; column <= run.last_column; ++column) {
    center_total += static_cast<double>(brightness[column]);
    ++center_count;
  }

  const int band_width_px = std::max(
    1, static_cast<int>(std::lround(
      config.local_background_band_m / config.meter_per_pixel)));
  constexpr int kBackgroundGapPx = 1;
  double background_total = 0.0;
  int background_count = 0;
  const int left_first = std::max(
    0, run.first_column - kBackgroundGapPx - band_width_px);
  const int left_last = std::max(
    -1, run.first_column - kBackgroundGapPx - 1);
  for (int column = left_first; column <= left_last; ++column) {
    background_total += static_cast<double>(brightness[column]);
    ++background_count;
  }
  const int right_first = std::min(
    gray.cols, run.last_column + kBackgroundGapPx + 1);
  const int right_last = std::min(
    gray.cols - 1,
    run.last_column + kBackgroundGapPx + band_width_px);
  for (int column = right_first; column <= right_last; ++column) {
    background_total += static_cast<double>(brightness[column]);
    ++background_count;
  }
  if (center_count == 0 || background_count == 0) {
    return false;
  }
  const double center_mean = center_total / static_cast<double>(center_count);
  const double background_mean =
    background_total / static_cast<double>(background_count);
  return
    center_mean - background_mean >=
    static_cast<double>(config.minimum_local_contrast) &&
    background_mean <=
    static_cast<double>(config.maximum_local_background_brightness);
}

std::vector<Seed> findSeedCandidates(
  const cv::Mat & candidate_mask,
  const cv::Mat & gray,
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

  std::vector<Seed> candidates;
  // A single candidate is not chosen from its distance to the vehicle centre;
  // it is ranked later by how long a real sliding-window track it supports.
  for (
    int row = near_row;
    row >= far_row;
    row -= config.row_step_px)
  {
    const double x_m = xAtRow(config, row);
    const auto raw_runs = findRuns(
      candidate_mask, row, minimum_run_width_px, maximum_run_width_px);
    std::vector<double> laterals_m;
    laterals_m.reserve(raw_runs.size());
    for (const auto & run : raw_runs) {
      if (hasDarkLocalBackground(gray, row, run, config)) {
        laterals_m.push_back(yAtColumn(config, run.centerColumn()));
      }
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
        Seed pair;
        pair.valid = true;
        pair.x_m = x_m;
        pair.left = cv::Point2d(x_m, left_m);
        pair.right = cv::Point2d(x_m, right_m);
        pair.left_measured = true;
        pair.right_measured = true;
        candidates.push_back(pair);
      }
    }

    if (config.allow_single_lane) {
      for (const double lateral_m : laterals_m) {
        Seed as_left;
        as_left.valid = true;
        as_left.x_m = x_m;
        as_left.left = cv::Point2d(x_m, lateral_m);
        as_left.right = cv::Point2d(
          x_m, lateral_m - config.expected_lane_width_m);
        as_left.left_measured = true;
        candidates.push_back(as_left);

        Seed as_right;
        as_right.valid = true;
        as_right.x_m = x_m;
        as_right.left = cv::Point2d(
          x_m, lateral_m + config.expected_lane_width_m);
        as_right.right = cv::Point2d(x_m, lateral_m);
        as_right.right_measured = true;
        candidates.push_back(as_right);
      }
    }
  }
  return candidates;
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
  const double minimum_longitudinal_m =
    -0.25 * config.sliding_window_step_m;
  const int lateral_bin_count = std::max(
    1, static_cast<int>(std::ceil(
      2.0 * expanded_half_width_m / config.meter_per_pixel)) + 1);
  std::vector<LateralBin> bins(
    static_cast<std::size_t>(lateral_bin_count));
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
        longitudinal_m < minimum_longitudinal_m ||
        longitudinal_m > half_length_m ||
        std::abs(lateral_m) > expanded_half_width_m)
      {
        continue;
      }
      const int bin_index = std::clamp(
        static_cast<int>(std::floor(
          (lateral_m + expanded_half_width_m) /
          config.meter_per_pixel)),
        0, lateral_bin_count - 1);
      auto & bin = bins[static_cast<std::size_t>(bin_index)];
      const double weight =
        1.0 + static_cast<double>(brightness[column]) / 255.0;
      bin.weighted_sum += point * weight;
      bin.total_weight += weight;
      bin.total_brightness += static_cast<double>(brightness[column]);
      bin.minimum_lateral_m = std::min(
        bin.minimum_lateral_m, lateral_m);
      bin.maximum_lateral_m = std::max(
        bin.maximum_lateral_m, lateral_m);
      ++bin.pixel_count;
    }
  }

  const double distance_blend = appearanceBlend(config, prediction.x);
  const double maximum_mark_width_m =
    (1.0 - distance_blend) * config.tracked_lane_mark_width_near_m +
    distance_blend * config.tracked_lane_mark_width_far_m;
  const double lateral_gate_m =
    ((1.0 - distance_blend) * config.measurement_lateral_gate_near_m +
    distance_blend * config.measurement_lateral_gate_far_m) *
    (1.0 + 0.25 * static_cast<double>(missing_windows));

  int first_bin = 0;
  while (first_bin < lateral_bin_count) {
    while (
      first_bin < lateral_bin_count &&
      bins[static_cast<std::size_t>(first_bin)].pixel_count == 0)
    {
      ++first_bin;
    }
    if (first_bin >= lateral_bin_count) {
      break;
    }
    int last_bin = first_bin;
    while (
      last_bin + 1 < lateral_bin_count &&
      bins[static_cast<std::size_t>(last_bin + 1)].pixel_count > 0)
    {
      ++last_bin;
    }

    int pixel_count = 0;
    double total_weight = 0.0;
    double total_brightness = 0.0;
    cv::Point2d weighted_sum(0.0, 0.0);
    double minimum_lateral_m = std::numeric_limits<double>::infinity();
    double maximum_lateral_m = -std::numeric_limits<double>::infinity();
    for (int bin_index = first_bin; bin_index <= last_bin; ++bin_index) {
      const auto & bin = bins[static_cast<std::size_t>(bin_index)];
      pixel_count += bin.pixel_count;
      total_weight += bin.total_weight;
      total_brightness += bin.total_brightness;
      weighted_sum += bin.weighted_sum;
      minimum_lateral_m = std::min(
        minimum_lateral_m, bin.minimum_lateral_m);
      maximum_lateral_m = std::max(
        maximum_lateral_m, bin.maximum_lateral_m);
    }
    first_bin = last_bin + 1;

    if (
      pixel_count < config.minimum_window_pixel_count ||
      total_weight <= 0.0)
    {
      continue;
    }
    const double mark_width_m = std::max(
      config.meter_per_pixel,
      maximum_lateral_m - minimum_lateral_m + config.meter_per_pixel);
    if (
      mark_width_m < config.minimum_lane_mark_width_m ||
      mark_width_m > maximum_mark_width_m)
    {
      continue;
    }

    const cv::Point2d measured_point = weighted_sum * (1.0 / total_weight);
    const cv::Point2d prediction_error = measured_point - prediction;
    const double lateral_error_m = std::abs(dot(prediction_error, normal));
    if (lateral_error_m > lateral_gate_m) {
      continue;
    }

    const double ridge_center_lateral_m = dot(prediction_error, normal);
    const double background_inner_m =
      0.5 * mark_width_m + config.meter_per_pixel;
    const double background_outer_m =
      background_inner_m + config.local_background_band_m;
    double background_total = 0.0;
    int background_count = 0;
    for (int row = minimum_row; row <= maximum_row; ++row) {
      const auto * brightness = gray.ptr<std::uint8_t>(row);
      const double x_m = xAtRow(config, row);
      for (int column = minimum_column; column <= maximum_column; ++column) {
        const cv::Point2d point(x_m, yAtColumn(config, column));
        const cv::Point2d delta = point - prediction;
        const double longitudinal_m = dot(delta, tangent);
        if (
          longitudinal_m < minimum_longitudinal_m ||
          longitudinal_m > half_length_m)
        {
          continue;
        }
        const double distance_from_ridge_m = std::abs(
          dot(delta, normal) - ridge_center_lateral_m);
        if (
          distance_from_ridge_m >= background_inner_m &&
          distance_from_ridge_m <= background_outer_m)
        {
          background_total += static_cast<double>(brightness[column]);
          ++background_count;
        }
      }
    }
    if (background_count == 0) {
      continue;
    }
    const double center_brightness =
      total_brightness / static_cast<double>(pixel_count);
    const double background_brightness =
      background_total / static_cast<double>(background_count);
    const double local_contrast = center_brightness - background_brightness;
    if (
      local_contrast < static_cast<double>(config.minimum_local_contrast) ||
      background_brightness >
      static_cast<double>(config.maximum_local_background_brightness))
    {
      continue;
    }

    const double longitudinal_error_m = std::abs(
      dot(prediction_error, tangent));
    const double selection_score_m =
      lateral_error_m + 0.25 * longitudinal_error_m +
      0.10 * mark_width_m -
      0.0002 * std::min(local_contrast, 100.0);
    if (selection_score_m < result.selection_score_m) {
      result.valid = true;
      result.point = measured_point;
      result.pixel_count = pixel_count;
      result.selection_score_m = selection_score_m;
      result.local_contrast = local_contrast;
    }
  }
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

std::vector<cv::Point2d> trackBackwardFromSeed(
  const cv::Point2d & seed_point,
  const bool seed_is_measured,
  const bool left_lane,
  const cv::Mat & candidate_mask,
  const cv::Mat & gray,
  const BevLaneReconstructorConfig & config,
  std::vector<BevLaneReconstruction::SlidingWindow> * debug_windows)
{
  if (!seed_is_measured) {
    return {};
  }
  LaneTracker tracker;
  tracker.active = true;
  tracker.position = seed_point;
  tracker.tangent = cv::Point2d(-1.0, 0.0);
  const int maximum_steps = std::max(
    1, static_cast<int>(std::ceil(
      (seed_point.x - config.reconstruction_minimum_x_m) /
      config.sliding_window_step_m)));
  for (int step = 0; step < maximum_steps && tracker.active; ++step) {
    const cv::Point2d prediction =
      tracker.position + config.sliding_window_step_m * tracker.tangent;
    const double blend = farBlend(config, prediction.x);
    const double half_width_m =
      (1.0 - blend) * config.sliding_window_half_width_near_m +
      blend * config.sliding_window_half_width_far_m;
    const auto measurement = measureWindow(
      candidate_mask, gray, config, prediction, tracker.tangent,
      half_width_m, tracker.missing_windows);
    debug_windows->push_back(makeSlidingWindowDebug(
        prediction, tracker.tangent, half_width_m, tracker.missing_windows,
        config.sliding_window_length_m, config.sliding_window_step_m,
        left_lane, measurement.valid));
    updateTracker(&tracker, prediction, measurement, config);
    if (tracker.position.x < config.reconstruction_minimum_x_m - 0.10) {
      tracker.active = false;
    }
  }
  std::reverse(tracker.measured_points.begin(), tracker.measured_points.end());
  return tracker.measured_points;
}

struct SeedProbeScore
{
  int measured_points{0};
  double maximum_x_m{0.0};
};

SeedProbeScore probeSeed(
  const Seed & seed,
  const cv::Mat & candidate_mask,
  const cv::Mat & gray,
  const BevLaneReconstructorConfig & config)
{
  LaneTracker left;
  left.active = seed.left_measured;
  left.position = seed.left;
  LaneTracker right;
  right.active = seed.right_measured;
  right.position = seed.right;
  SeedProbeScore score;
  score.measured_points =
    static_cast<int>(seed.left_measured) + static_cast<int>(seed.right_measured);
  score.maximum_x_m = seed.x_m;
  const int maximum_steps = std::max(
    1, static_cast<int>(std::ceil(
      std::min(0.60, config.maximum_tracking_arc_length_m) /
      config.sliding_window_step_m)));
  for (int step = 0; step < maximum_steps; ++step) {
    if (!left.active && !right.active) {
      break;
    }
    const auto probeTracker = [&](LaneTracker * tracker) {
        if (!tracker->active) {
          return;
        }
        const cv::Point2d prediction =
          tracker->position + config.sliding_window_step_m * tracker->tangent;
        const double blend = farBlend(config, prediction.x);
        const double half_width_m =
          (1.0 - blend) * config.sliding_window_half_width_near_m +
          blend * config.sliding_window_half_width_far_m;
        const auto measurement = measureWindow(
          candidate_mask, gray, config, prediction, tracker->tangent,
          half_width_m, tracker->missing_windows);
        if (measurement.valid) {
          ++score.measured_points;
          score.maximum_x_m = std::max(score.maximum_x_m, measurement.point.x);
        }
        updateTracker(tracker, prediction, measurement, config);
      };
    probeTracker(&left);
    probeTracker(&right);
  }
  return score;
}

Seed selectSeedByTrackSupport(
  const std::vector<Seed> & candidates,
  const cv::Mat & candidate_mask,
  const cv::Mat & gray,
  const BevLaneReconstructorConfig & config)
{
  Seed selected_pair;
  double best_pair_seed_score = std::numeric_limits<double>::infinity();
  Seed selected_single;
  SeedProbeScore best_single_score;
  for (const auto & candidate : candidates) {
    const bool paired_candidate =
      candidate.left_measured && candidate.right_measured;
    if (paired_candidate) {
      const double center_m = 0.5 * (candidate.left.y + candidate.right.y);
      const double width_m = std::abs(candidate.left.y - candidate.right.y);
      const double seed_score =
        std::abs(center_m) +
        0.5 * std::abs(width_m - config.expected_lane_width_m) +
        (candidate.x_m - config.observation_minimum_x_m);
      if (seed_score < best_pair_seed_score) {
        selected_pair = candidate;
        best_pair_seed_score = seed_score;
      }
      continue;
    }
    const auto score = probeSeed(candidate, candidate_mask, gray, config);
    if (
      !selected_single.valid ||
      score.measured_points > best_single_score.measured_points ||
      (score.measured_points == best_single_score.measured_points &&
      score.maximum_x_m > best_single_score.maximum_x_m))
    {
      selected_single = candidate;
      best_single_score = score;
    }
  }
  if (selected_pair.valid) {
    return selected_pair;
  }
  return selected_single;
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

std::optional<double> lateralAtX(
  const std::vector<cv::Point2d> & points,
  const double x_m)
{
  if (points.empty()) {
    return std::nullopt;
  }
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const auto & first = points[index - 1U];
    const auto & second = points[index];
    if ((x_m - first.x) * (x_m - second.x) > 0.0) {
      continue;
    }
    const double delta_x_m = second.x - first.x;
    if (std::abs(delta_x_m) <= 1.0e-9) {
      return 0.5 * (first.y + second.y);
    }
    const double blend = (x_m - first.x) / delta_x_m;
    return (1.0 - blend) * first.y + blend * second.y;
  }
  return std::nullopt;
}

bool tracksAreTemporallyConsistent(
  const std::vector<cv::Point2d> & current,
  const std::vector<cv::Point2d> & previous,
  const BevLaneReconstructorConfig & config)
{
  if (current.size() < 2U || previous.size() < 2U) {
    return false;
  }
  const auto current_x = std::minmax_element(
    current.begin(), current.end(),
    [](const cv::Point2d & first, const cv::Point2d & second) {
      return first.x < second.x;
    });
  const auto previous_x = std::minmax_element(
    previous.begin(), previous.end(),
    [](const cv::Point2d & first, const cv::Point2d & second) {
      return first.x < second.x;
    });
  const double minimum_x_m = std::max({
      config.observation_minimum_x_m,
      current_x.first->x,
      previous_x.first->x});
  const double maximum_x_m = std::min({
      config.observation_maximum_x_m,
      current_x.second->x,
      previous_x.second->x});
  if (maximum_x_m - minimum_x_m < 0.12) {
    return false;
  }

  constexpr int kComparisonSamples = 7;
  std::vector<double> normalized_lateral_errors;
  normalized_lateral_errors.reserve(kComparisonSamples);
  for (int sample = 0; sample < kComparisonSamples; ++sample) {
    const double blend = static_cast<double>(sample) /
      static_cast<double>(kComparisonSamples - 1);
    const double x_m =
      (1.0 - blend) * minimum_x_m + blend * maximum_x_m;
    const auto current_y = lateralAtX(current, x_m);
    const auto previous_y = lateralAtX(previous, x_m);
    if (!current_y.has_value() || !previous_y.has_value()) {
      continue;
    }
    const double distance_blend = appearanceBlend(config, x_m);
    const double maximum_jump_m =
      (1.0 - distance_blend) *
      config.temporal_maximum_lateral_jump_near_m +
      distance_blend * config.temporal_maximum_lateral_jump_far_m;
    normalized_lateral_errors.push_back(
      std::abs(*current_y - *previous_y) / maximum_jump_m);
  }
  if (
    normalized_lateral_errors.size() < 3U ||
    median(normalized_lateral_errors) > 1.0)
  {
    return false;
  }

  const auto current_y_start = lateralAtX(current, minimum_x_m);
  const auto current_y_end = lateralAtX(current, maximum_x_m);
  const auto previous_y_start = lateralAtX(previous, minimum_x_m);
  const auto previous_y_end = lateralAtX(previous, maximum_x_m);
  if (
    !current_y_start.has_value() || !current_y_end.has_value() ||
    !previous_y_start.has_value() || !previous_y_end.has_value())
  {
    return false;
  }
  const cv::Point2d current_heading = normalized(cv::Point2d(
      maximum_x_m - minimum_x_m,
      *current_y_end - *current_y_start));
  const cv::Point2d previous_heading = normalized(cv::Point2d(
      maximum_x_m - minimum_x_m,
      *previous_y_end - *previous_y_start));
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  const double heading_change_deg = std::acos(std::clamp(
      dot(current_heading, previous_heading), -1.0, 1.0)) *
    kRadiansToDegrees;
  return heading_change_deg <= config.temporal_maximum_heading_jump_deg;
}

TemporalTrackResult stabilizeTrack(
  const std::vector<cv::Point2d> & current,
  std::vector<cv::Point2d> * accepted,
  std::vector<cv::Point2d> * pending,
  int * pending_frames,
  int * held_frames,
  const BevLaneReconstructorConfig & config)
{
  TemporalTrackResult result;
  if (!config.temporal_tracking_enabled) {
    *accepted = current;
    pending->clear();
    *pending_frames = 0;
    *held_frames = 0;
    result.points = current;
    result.from_current_frame = !current.empty();
    return result;
  }

  if (!current.empty()) {
    if (
      accepted->empty() ||
      tracksAreTemporallyConsistent(current, *accepted, config))
    {
      *accepted = current;
      pending->clear();
      *pending_frames = 0;
      *held_frames = 0;
      result.points = current;
      result.from_current_frame = true;
      return result;
    }

    if (
      !pending->empty() &&
      tracksAreTemporallyConsistent(current, *pending, config))
    {
      ++(*pending_frames);
    } else {
      *pending = current;
      *pending_frames = 1;
    }
    if (*pending_frames >= config.temporal_confirmation_frames) {
      *accepted = current;
      pending->clear();
      *pending_frames = 0;
      *held_frames = 0;
      result.points = current;
      result.from_current_frame = true;
      return result;
    }
  } else {
    pending->clear();
    *pending_frames = 0;
  }

  ++(*held_frames);
  if (!accepted->empty() && *held_frames <= config.temporal_hold_frames) {
    result.points = *accepted;
    result.held_previous_frame = true;
    return result;
  }
  accepted->clear();
  return result;
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

CorrespondingLaneResult buildCorrespondingLane(
  const std::vector<cv::Point2d> & reference,
  const double offset_sign,
  const double lane_width_m,
  const cv::Mat & candidate_mask,
  const cv::Mat & gray,
  const BevLaneReconstructorConfig & config)
{
  CorrespondingLaneResult result;
  result.points.reserve(reference.size());
  const double correspondence_half_width_m = 0.5 *
    (config.correspondence_maximum_width_m -
    config.correspondence_minimum_width_m);
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
    const cv::Point2d expected_point =
      reference[index] + offset_sign * lane_width_m * left_normal;
    const auto measurement = measureWindow(
      candidate_mask, gray, config, expected_point, tangent,
      correspondence_half_width_m, 0);
    bool actual_pixel_found = false;
    if (measurement.valid) {
      const cv::Point2d delta = measurement.point - reference[index];
      const double signed_width_m = offset_sign * dot(delta, left_normal);
      const double longitudinal_error_m = std::abs(dot(delta, tangent));
      actual_pixel_found =
        signed_width_m >= config.correspondence_minimum_width_m &&
        signed_width_m <= config.correspondence_maximum_width_m &&
        longitudinal_error_m <=
        config.correspondence_longitudinal_tolerance_m;
    }
    if (actual_pixel_found) {
      result.points.push_back(measurement.point);
    } else {
      result.points.push_back(expected_point);
      ++result.inferred_point_count;
    }
  }
  return result;
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
    config_.minimum_local_contrast < 0 ||
    config_.minimum_local_contrast > 255 ||
    config_.maximum_local_background_brightness < 0 ||
    config_.maximum_local_background_brightness > 255 ||
    config_.local_background_band_m <= 0.0 ||
    config_.tracked_lane_mark_width_near_m <
    config_.minimum_lane_mark_width_m ||
    config_.tracked_lane_mark_width_far_m <
    config_.tracked_lane_mark_width_near_m ||
    config_.measurement_lateral_gate_near_m <= 0.0 ||
    config_.measurement_lateral_gate_far_m <
    config_.measurement_lateral_gate_near_m ||
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
    config_.correspondence_minimum_width_m <= 0.0 ||
    config_.correspondence_maximum_width_m <=
    config_.correspondence_minimum_width_m ||
    config_.expected_lane_width_m < config_.correspondence_minimum_width_m ||
    config_.expected_lane_width_m > config_.correspondence_maximum_width_m ||
    config_.correspondence_longitudinal_tolerance_m <= 0.0 ||
    config_.temporal_maximum_lateral_jump_near_m <= 0.0 ||
    config_.temporal_maximum_lateral_jump_far_m <
    config_.temporal_maximum_lateral_jump_near_m ||
    config_.temporal_maximum_heading_jump_deg <= 0.0 ||
    config_.temporal_maximum_heading_jump_deg > 180.0 ||
    config_.temporal_confirmation_frames <= 0 ||
    config_.temporal_hold_frames < 0 ||
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
  const auto seed_candidates = findSeedCandidates(
    result.candidate_mask, gray, config_);
  const Seed seed = selectSeedByTrackSupport(
    seed_candidates, result.candidate_mask, gray, config_);
  std::vector<cv::Point2d> backward_left_points = trackBackwardFromSeed(
    seed.left, seed.left_measured, true, result.candidate_mask, gray,
    config_, &result.sliding_windows);
  std::vector<cv::Point2d> backward_right_points = trackBackwardFromSeed(
    seed.right, seed.right_measured, false, result.candidate_mask, gray,
    config_, &result.sliding_windows);

  LaneTracker left;
  left.active = seed.left_measured;
  left.position = seed.left;
  if (seed.left_measured) {
    left.measured_points.push_back(seed.left);
  }
  LaneTracker right;
  right.active = seed.right_measured;
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
      result.sliding_windows.push_back(makeSlidingWindowDebug(
          left_prediction, left.tangent, left_half_width_m,
          left.missing_windows, config_.sliding_window_length_m,
          config_.sliding_window_step_m, true, left_measurement.valid));
    }
    WindowMeasurement right_measurement;
    if (right.active) {
      right_measurement = measureWindow(
        result.candidate_mask, gray, config_,
        right_prediction, right.tangent,
        right_half_width_m, right.missing_windows);
      result.sliding_windows.push_back(makeSlidingWindowDebug(
          right_prediction, right.tangent, right_half_width_m,
          right.missing_windows, config_.sliding_window_length_m,
          config_.sliding_window_step_m, false, right_measurement.valid));
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

  backward_left_points.insert(
    backward_left_points.end(),
    left.measured_points.begin(), left.measured_points.end());
  backward_right_points.insert(
    backward_right_points.end(),
    right.measured_points.begin(), right.measured_points.end());
  result.left_measured_points = std::move(backward_left_points);
  result.right_measured_points = std::move(backward_right_points);
  result.measured_point_count = static_cast<int>(
    result.left_measured_points.size() + result.right_measured_points.size());
  const bool current_left_valid =
    static_cast<int>(result.left_measured_points.size()) >= config_.minimum_points;
  const bool current_right_valid =
    static_cast<int>(result.right_measured_points.size()) >= config_.minimum_points;
  const std::vector<cv::Point2d> empty_points;
  const TemporalTrackResult stable_left = stabilizeTrack(
    current_left_valid ? result.left_measured_points : empty_points,
    &accepted_left_points_, &pending_left_points_,
    &pending_left_frames_, &held_left_frames_, config_);
  const TemporalTrackResult stable_right = stabilizeTrack(
    current_right_valid ? result.right_measured_points : empty_points,
    &accepted_right_points_, &pending_right_points_,
    &pending_right_frames_, &held_right_frames_, config_);
  result.temporal_hold_used =
    stable_left.held_previous_frame || stable_right.held_previous_frame;

  if (
    stable_left.from_current_frame && stable_right.from_current_frame &&
    !measured_widths_m.empty())
  {
    const double current_width_m = median(measured_widths_m);
    if (
      current_width_m >= config_.correspondence_minimum_width_m &&
      current_width_m <= config_.correspondence_maximum_width_m)
    {
      accepted_lane_width_m_ = accepted_lane_width_m_ > 0.0 ?
        0.80 * accepted_lane_width_m_ + 0.20 * current_width_m :
        current_width_m;
    }
  }
  const double lane_width_m = std::clamp(
    accepted_lane_width_m_ > 0.0 ?
    accepted_lane_width_m_ : config_.expected_lane_width_m,
    config_.correspondence_minimum_width_m,
    config_.correspondence_maximum_width_m);
  result.measured_lane_width_m = lane_width_m;

  const bool left_valid =
    static_cast<int>(stable_left.points.size()) >= config_.minimum_points;
  const bool right_valid =
    static_cast<int>(stable_right.points.size()) >= config_.minimum_points;
  if (!left_valid && !right_valid) {
    return result;
  }

  std::vector<cv::Point2d> left_output;
  std::vector<cv::Point2d> right_output;
  if (left_valid) {
    left_output = smoothMeasuredPoints(
      stable_left.points, config_.measured_point_smoothing_weight);
  }
  if (right_valid) {
    right_output = smoothMeasuredPoints(
      stable_right.points, config_.measured_point_smoothing_weight);
  }

  if (
    config_.infer_partially_missing_lane &&
    ((left_valid && right_valid) || config_.allow_single_lane))
  {
    const auto maximumX = [](const std::vector<cv::Point2d> & points) {
        double maximum_x_m = 0.0;
        for (const auto & point : points) {
          maximum_x_m = std::max(maximum_x_m, point.x);
        }
        return maximum_x_m;
      };
    const bool use_left_reference = left_valid &&
      (!right_valid ||
      (stable_left.from_current_frame && !stable_right.from_current_frame) ||
      (stable_left.from_current_frame == stable_right.from_current_frame &&
      maximumX(left_output) >= maximumX(right_output)));
    if (use_left_reference) {
      auto corresponding = buildCorrespondingLane(
        left_output, -1.0, lane_width_m,
        result.candidate_mask, gray, config_);
      right_output = std::move(corresponding.points);
      result.inferred_point_count += corresponding.inferred_point_count;
    } else if (right_valid) {
      auto corresponding = buildCorrespondingLane(
        right_output, 1.0, lane_width_m,
        result.candidate_mask, gray, config_);
      left_output = std::move(corresponding.points);
      result.inferred_point_count += corresponding.inferred_point_count;
    }
  } else if (
    config_.infer_partially_missing_lane && left_valid && !right_valid &&
    config_.allow_single_lane)
  {
    right_output = offsetLane(left_output, -lane_width_m);
    result.inferred_point_count += static_cast<int>(right_output.size());
  } else if (
    config_.infer_partially_missing_lane && right_valid && !left_valid &&
    config_.allow_single_lane)
  {
    left_output = offsetLane(right_output, lane_width_m);
    result.inferred_point_count += static_cast<int>(left_output.size());
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
  accepted_left_points_.clear();
  accepted_right_points_.clear();
  pending_left_points_.clear();
  pending_right_points_.clear();
  pending_left_frames_ = 0;
  pending_right_frames_ = 0;
  held_left_frames_ = 0;
  held_right_frames_ = 0;
  accepted_lane_width_m_ = 0.0;
}

}  // namespace bev_processor
