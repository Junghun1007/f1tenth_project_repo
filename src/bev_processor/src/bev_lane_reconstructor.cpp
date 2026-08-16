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
  double implied_center_abs_m{0.0};
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

struct LanePairGeometry
{
  bool valid{false};
  double width_m{0.0};
  int sample_count{0};
  cv::Point2d representative_left;
  cv::Point2d representative_right;
};

struct LanePairSample
{
  double distance_m{0.0};
  cv::Point2d left;
  cv::Point2d right;
};

LanePairGeometry measureLanePairGeometry(
  const std::vector<cv::Point2d> & left,
  const std::vector<cv::Point2d> & right,
  const BevLaneReconstructorConfig & config)
{
  LanePairGeometry geometry;
  if (left.size() < 2U || right.size() < 2U) {
    return geometry;
  }

  std::vector<LanePairSample> samples;
  const auto appendNormalProjections = [&samples](
      const std::vector<cv::Point2d> & query,
      const std::vector<cv::Point2d> & reference,
      const bool query_is_left) {
      for (const auto & query_point : query) {
        for (std::size_t index = 1U; index < reference.size(); ++index) {
          const cv::Point2d segment = reference[index] - reference[index - 1U];
          const double segment_length_squared = dot(segment, segment);
          if (segment_length_squared <= 1.0e-9) {
            continue;
          }
          const double projection_blend = dot(
            query_point - reference[index - 1U], segment) /
            segment_length_squared;
          // Endpoint clamping would turn non-overlapping track ends into a
          // false lane-width sample. Only an actual perpendicular foot on the
          // measured segment is used.
          if (projection_blend < 0.0 || projection_blend > 1.0) {
            continue;
          }
          const cv::Point2d projection =
            reference[index - 1U] + projection_blend * segment;
          const cv::Point2d tangent = normalized(segment);
          const cv::Point2d left_normal(-tangent.y, tangent.x);
          const cv::Point2d left_point = query_is_left ? query_point : projection;
          const cv::Point2d right_point = query_is_left ? projection : query_point;
          // Both measured tracks are ordered from near to far. This signed
          // test rejects two seeds from the same marking and crossed pairs.
          if (dot(left_point - right_point, left_normal) <= 0.0) {
            continue;
          }
          samples.push_back(
            {norm(left_point - right_point), left_point, right_point});
        }
      }
    };
  appendNormalProjections(left, right, true);
  appendNormalProjections(right, left, false);
  if (samples.size() < 3U) {
    return geometry;
  }

  std::vector<double> distances_m;
  distances_m.reserve(samples.size());
  for (const auto & sample : samples) {
    distances_m.push_back(sample.distance_m);
  }
  geometry.width_m = median(std::move(distances_m));
  geometry.sample_count = static_cast<int>(samples.size());
  const double minimum_width_m =
    config.expected_lane_width_m - config.lane_width_tolerance_m;
  const double maximum_width_m =
    config.expected_lane_width_m + config.lane_width_tolerance_m;
  if (
    geometry.width_m < minimum_width_m ||
    geometry.width_m > maximum_width_m)
  {
    return geometry;
  }

  const auto representative = std::min_element(
    samples.begin(), samples.end(),
    [&geometry](const LanePairSample & first, const LanePairSample & second) {
      const double first_width_error =
        std::abs(first.distance_m - geometry.width_m);
      const double second_width_error =
        std::abs(second.distance_m - geometry.width_m);
      if (std::abs(first_width_error - second_width_error) > 1.0e-9) {
        return first_width_error < second_width_error;
      }
      return
        0.5 * (first.left.x + first.right.x) <
        0.5 * (second.left.x + second.right.x);
    });
  geometry.representative_left = representative->left;
  geometry.representative_right = representative->right;
  geometry.valid = true;
  return geometry;
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
    static_cast<int>(std::lround(
      (config.y_max_m + config.output_lateral_margin_m - metric_point.y) /
      config.meter_per_pixel - 0.5)),
    rowAtX(config, metric_point.x));
}

bool imagePointInside(
  const cv::Mat & image,
  const cv::Point & point)
{
  return
    point.x >= 0 && point.x < image.cols &&
    point.y >= 0 && point.y < image.rows;
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
        pair.implied_center_abs_m = std::abs(center_m);
        candidates.push_back(pair);
      }
    }

    if (config.allow_single_lane) {
      for (const double lateral_m : laterals_m) {
        const double as_left_center_m =
          lateral_m - 0.5 * config.expected_lane_width_m;
        Seed as_left;
        as_left.valid = true;
        as_left.x_m = x_m;
        as_left.left = cv::Point2d(x_m, lateral_m);
        as_left.right = cv::Point2d(
          x_m, lateral_m - config.expected_lane_width_m);
        as_left.left_measured = true;
        as_left.implied_center_abs_m = std::abs(as_left_center_m);
        if (
          as_left.implied_center_abs_m <=
          config.single_lane_initial_tolerance_m)
        {
          candidates.push_back(as_left);
        }

        const double as_right_center_m =
          lateral_m + 0.5 * config.expected_lane_width_m;
        Seed as_right;
        as_right.valid = true;
        as_right.x_m = x_m;
        as_right.left = cv::Point2d(
          x_m, lateral_m + config.expected_lane_width_m);
        as_right.right = cv::Point2d(x_m, lateral_m);
        as_right.right_measured = true;
        as_right.implied_center_abs_m = std::abs(as_right_center_m);
        if (
          as_right.implied_center_abs_m <=
          config.single_lane_initial_tolerance_m)
        {
          candidates.push_back(as_right);
        }
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
  std::vector<cv::Point2d> points;
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
  if (seed.left_measured) {
    left.measured_points.push_back(seed.left);
  }
  LaneTracker right;
  right.active = seed.right_measured;
  right.position = seed.right;
  if (seed.right_measured) {
    right.measured_points.push_back(seed.right);
  }
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
  score.points = seed.left_measured ?
    std::move(left.measured_points) : std::move(right.measured_points);
  return score;
}

struct ProbedSingleSeed
{
  Seed seed;
  SeedProbeScore score;
  double identity_error_m{std::numeric_limits<double>::infinity()};
};

bool betterProbe(
  const ProbedSingleSeed & first,
  const ProbedSingleSeed & second)
{
  if (first.score.measured_points != second.score.measured_points) {
    return first.score.measured_points > second.score.measured_points;
  }
  if (
    std::abs(first.score.maximum_x_m - second.score.maximum_x_m) > 1.0e-9)
  {
    return first.score.maximum_x_m > second.score.maximum_x_m;
  }
  return first.identity_error_m < second.identity_error_m;
}

std::vector<ProbedSingleSeed> selectDiverseProbes(
  std::vector<ProbedSingleSeed> probes,
  const std::size_t maximum_count)
{
  constexpr double kDuplicateTrackDistanceM = 0.04;
  std::sort(probes.begin(), probes.end(), betterProbe);
  std::vector<ProbedSingleSeed> selected;
  selected.reserve(std::min(maximum_count, probes.size()));
  for (auto & probe : probes) {
    bool duplicates_selected_track = false;
    for (const auto & existing : selected) {
      double minimum_distance_m = std::numeric_limits<double>::infinity();
      for (const auto & point : probe.score.points) {
        for (const auto & existing_point : existing.score.points) {
          minimum_distance_m = std::min(
            minimum_distance_m, norm(point - existing_point));
        }
      }
      if (minimum_distance_m <= kDuplicateTrackDistanceM) {
        duplicates_selected_track = true;
        break;
      }
    }
    if (duplicates_selected_track) {
      continue;
    }
    selected.push_back(std::move(probe));
    if (selected.size() >= maximum_count) {
      break;
    }
  }
  return selected;
}

double seedIdentityError(
  const Seed & seed,
  const std::vector<cv::Point2d> & previous_left,
  const std::vector<cv::Point2d> & previous_right)
{
  const auto pointError = [](
      const cv::Point2d & point,
      const std::vector<cv::Point2d> & previous) {
      if (previous.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      const auto closest = std::min_element(
        previous.begin(), previous.end(),
        [&point](const cv::Point2d & first, const cv::Point2d & second) {
          return std::abs(first.x - point.x) < std::abs(second.x - point.x);
        });
      return std::abs(closest->y - point.y);
    };
  double total_error_m = 0.0;
  int comparison_count = 0;
  if (seed.left_measured) {
    const double error_m = pointError(seed.left, previous_left);
    if (std::isfinite(error_m)) {
      total_error_m += error_m;
      ++comparison_count;
    }
  }
  if (seed.right_measured) {
    const double error_m = pointError(seed.right, previous_right);
    if (std::isfinite(error_m)) {
      total_error_m += error_m;
      ++comparison_count;
    }
  }
  return comparison_count > 0 ?
         total_error_m / static_cast<double>(comparison_count) :
         seed.implied_center_abs_m;
}

Seed selectSeedByTrackSupport(
  const std::vector<Seed> & candidates,
  const cv::Mat & candidate_mask,
  const cv::Mat & gray,
  const BevLaneReconstructorConfig & config,
  const std::vector<cv::Point2d> & previous_left,
  const std::vector<cv::Point2d> & previous_right)
{
  Seed selected_pair;
  double best_pair_seed_score = std::numeric_limits<double>::infinity();
  Seed selected_single;
  SeedProbeScore best_single_score;
  double best_single_identity_error_m =
    std::numeric_limits<double>::infinity();
  std::vector<ProbedSingleSeed> left_probes;
  std::vector<ProbedSingleSeed> right_probes;
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
    const double identity_error_m = seedIdentityError(
      candidate, previous_left, previous_right);
    ProbedSingleSeed probed{candidate, score, identity_error_m};
    if (candidate.left_measured) {
      left_probes.push_back(probed);
    } else {
      right_probes.push_back(probed);
    }
    if (
      !selected_single.valid ||
      score.measured_points > best_single_score.measured_points ||
      (score.measured_points == best_single_score.measured_points &&
      score.maximum_x_m > best_single_score.maximum_x_m + 1.0e-9) ||
      (score.measured_points == best_single_score.measured_points &&
      std::abs(score.maximum_x_m - best_single_score.maximum_x_m) <= 1.0e-9 &&
      identity_error_m < best_single_identity_error_m))
    {
      selected_single = candidate;
      best_single_score = score;
      best_single_identity_error_m = identity_error_m;
    }
  }
  if (selected_pair.valid) {
    return selected_pair;
  }

  // Same-X lateral separation grows on a diagonal or tight curve even when
  // the physical lane width measured along the normal is correct. Probe the
  // two visible markings independently, then pair them by perpendicular
  // polyline distance so even a short real counterpart contributes to the
  // centerline estimate without being replaced by a synthetic boundary.
  constexpr std::size_t kMaximumDiverseProbesPerSide = 8U;
  left_probes = selectDiverseProbes(
    std::move(left_probes), kMaximumDiverseProbesPerSide);
  right_probes = selectDiverseProbes(
    std::move(right_probes), kMaximumDiverseProbesPerSide);
  Seed recovered_pair;
  int best_minimum_support = -1;
  int best_total_support = -1;
  double best_geometry_error_m = std::numeric_limits<double>::infinity();
  for (const auto & left_probe : left_probes) {
    for (const auto & right_probe : right_probes) {
      const int left_support = left_probe.score.measured_points;
      const int right_support = right_probe.score.measured_points;
      if (
        std::max(left_support, right_support) < config.minimum_points ||
        std::min(left_support, right_support) <
        config.minimum_counterpart_points)
      {
        continue;
      }
      const LanePairGeometry geometry = measureLanePairGeometry(
        left_probe.score.points, right_probe.score.points, config);
      if (!geometry.valid) {
        continue;
      }
      const double center_abs_m = std::abs(
        0.5 * (geometry.representative_left.y +
        geometry.representative_right.y));
      if (center_abs_m > config.initial_center_tolerance_m) {
        continue;
      }
      const int minimum_support = std::min(left_support, right_support);
      const int total_support = left_support + right_support;
      const double geometry_error_m =
        std::abs(geometry.width_m - config.expected_lane_width_m) +
        0.25 * center_abs_m;
      if (
        !recovered_pair.valid ||
        minimum_support > best_minimum_support ||
        (minimum_support == best_minimum_support &&
        total_support > best_total_support) ||
        (minimum_support == best_minimum_support &&
        total_support == best_total_support &&
        geometry_error_m < best_geometry_error_m))
      {
        recovered_pair.valid = true;
        recovered_pair.x_m = 0.5 * (
          geometry.representative_left.x + geometry.representative_right.x);
        recovered_pair.left = geometry.representative_left;
        recovered_pair.right = geometry.representative_right;
        recovered_pair.left_measured = true;
        recovered_pair.right_measured = true;
        recovered_pair.implied_center_abs_m = center_abs_m;
        best_minimum_support = minimum_support;
        best_total_support = total_support;
        best_geometry_error_m = geometry_error_m;
      }
    }
  }
  if (recovered_pair.valid) {
    return recovered_pair;
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
  const double offset_m,
  const BevLaneReconstructorConfig & config)
{
  if (reference.empty()) {
    return {};
  }
  constexpr double kPi = 3.14159265358979323846;
  const auto angleDifference = [](const double first, const double second) {
      double difference = first - second;
      while (difference > kPi) {
        difference -= 2.0 * kPi;
      }
      while (difference < -kPi) {
        difference += 2.0 * kPi;
      }
      return difference;
    };
  const double maximum_heading_step_rad =
    config.centerline_maximum_heading_step_deg * kPi / 180.0;
  const double maximum_curvature_per_m = std::min(
    config.centerline_maximum_curvature_per_m,
    0.90 / std::max(std::abs(offset_m), config.meter_per_pixel));

  std::vector<double> arc_length_m(reference.size(), 0.0);
  for (std::size_t index = 1U; index < reference.size(); ++index) {
    arc_length_m[index] = arc_length_m[index - 1U] +
      norm(reference[index] - reference[index - 1U]);
  }

  // Estimate every tangent from a metric neighbourhood rather than from one
  // adjacent point. This prevents one noisy sample from rotating the full
  // lane-width offset by a large angle.
  std::vector<double> tangent_heading_rad(reference.size(), 0.0);
  const double half_window_m = 0.5 * config.centerline_tangent_window_m;
  for (std::size_t index = 0U; index < reference.size(); ++index) {
    std::size_t first = index;
    while (
      first > 0U &&
      arc_length_m[index] - arc_length_m[first] < half_window_m)
    {
      --first;
    }
    std::size_t last = index;
    while (
      last + 1U < reference.size() &&
      arc_length_m[last] - arc_length_m[index] < half_window_m)
    {
      ++last;
    }
    cv::Point2d tangent = reference[last] - reference[first];
    if (norm(tangent) <= 1.0e-9 && index > 0U) {
      tangent = reference[index] - reference[index - 1U];
    }
    tangent = normalized(tangent);
    double heading_rad = std::atan2(tangent.y, tangent.x);
    if (index > 0U) {
      const double step_m = std::max(
        config.meter_per_pixel,
        arc_length_m[index] - arc_length_m[index - 1U]);
      const double maximum_change_rad = std::min(
        maximum_heading_step_rad,
        maximum_curvature_per_m * step_m);
      const double change_rad = std::clamp(
        angleDifference(heading_rad, tangent_heading_rad[index - 1U]),
        -maximum_change_rad, maximum_change_rad);
      heading_rad = tangent_heading_rad[index - 1U] + change_rad;
    }
    tangent_heading_rad[index] = heading_rad;
  }

  std::vector<cv::Point2d> offset;
  offset.reserve(reference.size());
  for (std::size_t index = 0; index < reference.size(); ++index) {
    const cv::Point2d tangent(
      std::cos(tangent_heading_rad[index]),
      std::sin(tangent_heading_rad[index]));
    const cv::Point2d left_normal(-tangent.y, tangent.x);
    offset.push_back(reference[index] + offset_m * left_normal);
  }

  return offset;
}

std::vector<cv::Point2d> translateLaneLaterally(
  const std::vector<cv::Point2d> & reference,
  const double offset_m)
{
  std::vector<cv::Point2d> offset = reference;
  for (auto & point : offset) {
    point.y += offset_m;
  }
  return offset;
}

struct OffsetLaneResult
{
  std::vector<cv::Point2d> points;
  bool normal_offset_truncated{false};
  std::size_t reference_first_index{0U};
};

OffsetLaneResult retainLongestSafeNormalOffsetSegment(
  const std::vector<cv::Point2d> & reference,
  const std::vector<cv::Point2d> & offset,
  const BevLaneReconstructorConfig & config,
  const int minimum_required_points)
{
  if (reference.size() != offset.size() || offset.size() < 2U) {
    return {{}, !offset.empty(), 0U};
  }

  std::size_t best_first = 0U;
  std::size_t best_last = 1U;
  double best_arc_length_m = 0.0;
  std::size_t current_first = 0U;
  double current_arc_length_m = 0.0;
  const auto keepCurrentSegment = [&](const std::size_t last) {
      if (
        last - current_first > best_last - best_first ||
        (last - current_first == best_last - best_first &&
        current_arc_length_m > best_arc_length_m))
      {
        best_first = current_first;
        best_last = last;
        best_arc_length_m = current_arc_length_m;
      }
    };

  for (std::size_t index = 1U; index < offset.size(); ++index) {
    const cv::Point2d reference_segment =
      reference[index] - reference[index - 1U];
    const cv::Point2d offset_segment =
      offset[index] - offset[index - 1U];
    const double reference_length_m = norm(reference_segment);
    const double offset_length_m = norm(offset_segment);
    const bool same_curve_direction =
      reference_length_m >= config.meter_per_pixel &&
      offset_length_m >= config.meter_per_pixel &&
      dot(reference_segment, offset_segment) > 0.0;
    const bool forward_in_vehicle_frame =
      offset[index].x + config.meter_per_pixel >= offset[index - 1U].x;
    if (!same_curve_direction || !forward_in_vehicle_frame) {
      keepCurrentSegment(index);
      current_first = index;
      current_arc_length_m = 0.0;
      continue;
    }
    current_arc_length_m += offset_length_m;
  }
  keepCurrentSegment(offset.size());

  if (
    best_last - best_first <
    static_cast<std::size_t>(minimum_required_points))
  {
    return {{}, true, 0U};
  }
  return {
    std::vector<cv::Point2d>(
      offset.begin() + static_cast<std::ptrdiff_t>(best_first),
      offset.begin() + static_cast<std::ptrdiff_t>(best_last)),
    best_first != 0U || best_last != offset.size(),
    best_first};
}

OffsetLaneResult offsetFromReference(
  const std::vector<cv::Point2d> & reference,
  const double offset_m,
  const BevLaneReconstructorConfig & config,
  const int minimum_required_points)
{
  if (config.centerline_preserve_reference_shape) {
    return {translateLaneLaterally(reference, offset_m), false, 0U};
  }

  // Never replace a normal offset with a same-X lateral translation: that
  // preserves the measured radius and creates a physically false lane on a
  // tight curve. Parallel curves can become singular when the requested
  // offset exceeds the local radius, so retain only the longest segment that
  // still advances with the measured curve. If it is too short, emit no
  // offset path instead of drawing one at the wrong position.
  return retainLongestSafeNormalOffsetSegment(
    reference, offsetLane(reference, offset_m, config), config,
    minimum_required_points);
}

double polylineArcLength(const std::vector<cv::Point2d> & points)
{
  double length_m = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    length_m += norm(points[index] - points[index - 1U]);
  }
  return length_m;
}

struct BoundaryProjection
{
  bool valid{false};
  cv::Point2d point;
  double distance_m{std::numeric_limits<double>::infinity()};
};

BoundaryProjection closestBoundaryProjection(
  const cv::Point2d & query,
  const std::vector<cv::Point2d> & reference)
{
  BoundaryProjection best;
  for (std::size_t index = 1U; index < reference.size(); ++index) {
    const cv::Point2d segment = reference[index] - reference[index - 1U];
    const double segment_length_squared = dot(segment, segment);
    if (segment_length_squared <= 1.0e-9) {
      continue;
    }
    const double blend = dot(query - reference[index - 1U], segment) /
      segment_length_squared;
    // A clamped endpoint is not a true correspondence and can incorrectly
    // pair the non-overlapping ends of two measured tracks.
    if (blend < 0.0 || blend > 1.0) {
      continue;
    }
    const cv::Point2d projection =
      reference[index - 1U] + blend * segment;
    const double distance_m = norm(query - projection);
    if (distance_m < best.distance_m) {
      best = {true, projection, distance_m};
    }
  }
  return best;
}

std::vector<cv::Point2d> centerlineFromMeasuredPair(
  const std::vector<cv::Point2d> & left,
  const std::vector<cv::Point2d> & right,
  const OffsetLaneResult & center_from_left,
  const OffsetLaneResult & center_from_right,
  const BevLaneReconstructorConfig & config,
  bool * normal_offset_truncated)
{
  if (left.size() < 2U || right.size() < 2U) {
    return {};
  }

  const bool left_is_primary =
    polylineArcLength(left) >= polylineArcLength(right);
  const auto & primary = left_is_primary ? left : right;
  const auto & secondary = left_is_primary ? right : left;
  const auto & fallback_center = left_is_primary ?
    center_from_left : center_from_right;
  *normal_offset_truncated = fallback_center.normal_offset_truncated;
  const double width_margin_m = 2.0 * config.meter_per_pixel;
  const double minimum_width_m = std::max(
    config.meter_per_pixel,
    config.expected_lane_width_m - config.lane_width_tolerance_m -
    width_margin_m);
  const double maximum_width_m =
    config.expected_lane_width_m + config.lane_width_tolerance_m +
    width_margin_m;

  std::vector<std::optional<cv::Point2d>> direct_midpoints(primary.size());
  std::size_t direct_midpoint_count = 0U;
  for (std::size_t index = 0U; index < primary.size(); ++index) {
    const auto & point = primary[index];
    const BoundaryProjection projection = closestBoundaryProjection(
      point, secondary);
    if (
      projection.valid &&
      projection.distance_m >= minimum_width_m &&
      projection.distance_m <= maximum_width_m)
    {
      direct_midpoints[index] = 0.5 * (point + projection.point);
      ++direct_midpoint_count;
    }
  }
  if (
    direct_midpoint_count <
    static_cast<std::size_t>(config.minimum_counterpart_points))
  {
    return {};
  }

  // Follow the longer measured boundary through portions where its real
  // counterpart is unavailable. Wherever both measurements have a valid
  // perpendicular correspondence, the actual midpoint takes precedence.
  // Because both candidates share the longer boundary's sample order, this
  // also fills brief correspondence holes without reordering a tight curve.
  std::vector<cv::Point2d> centerline;
  centerline.reserve(primary.size());
  const std::size_t fallback_first =
    fallback_center.reference_first_index;
  const std::size_t fallback_last =
    fallback_first + fallback_center.points.size();
  for (std::size_t index = 0U; index < primary.size(); ++index) {
    if (direct_midpoints[index].has_value()) {
      centerline.push_back(*direct_midpoints[index]);
    } else if (index >= fallback_first && index < fallback_last) {
      centerline.push_back(fallback_center.points[index - fallback_first]);
    }
  }
  return smoothMeasuredPoints(
    centerline, config.centerline_midpoint_smoothing_weight);
}

std::vector<cv::Point2d> smoothCenterlineAgainstPrevious(
  const std::vector<cv::Point2d> & current,
  const std::vector<cv::Point2d> & previous,
  const double current_weight)
{
  if (current.empty() || previous.size() < 2U || current_weight >= 1.0) {
    return current;
  }
  std::vector<cv::Point2d> smoothed = current;
  for (auto & point : smoothed) {
    const auto previous_y = lateralAtX(previous, point.x);
    if (previous_y.has_value()) {
      point.y = current_weight * point.y +
        (1.0 - current_weight) * *previous_y;
    }
  }
  return smoothed;
}

double centerlineLateralCorrection(
  const std::vector<cv::Point2d> & reference,
  const std::vector<cv::Point2d> & current)
{
  std::vector<double> differences_m;
  differences_m.reserve(current.size());
  for (const auto & point : current) {
    const auto reference_y = lateralAtX(reference, point.x);
    if (reference_y.has_value()) {
      differences_m.push_back(*reference_y - point.y);
    }
  }
  return median(std::move(differences_m));
}

std::optional<double> centerlineMedianAbsoluteLateralError(
  const std::vector<cv::Point2d> & reference,
  const std::vector<cv::Point2d> & candidate)
{
  std::vector<double> errors_m;
  errors_m.reserve(candidate.size());
  for (const auto & point : candidate) {
    const auto reference_y = lateralAtX(reference, point.x);
    if (reference_y.has_value()) {
      errors_m.push_back(std::abs(*reference_y - point.y));
    }
  }
  if (errors_m.size() < 3U) {
    return std::nullopt;
  }
  return median(std::move(errors_m));
}

void translateCenterlineLaterally(
  std::vector<cv::Point2d> * points,
  const double correction_m)
{
  for (auto & point : *points) {
    point.y += correction_m;
  }
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
    if (!imagePointInside(*mask, pixel)) {
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
    config_.y_max_m <= config_.y_min_m ||
    !std::isfinite(config_.output_lateral_margin_m) ||
    config_.output_lateral_margin_m < 0.0 ||
    config_.meter_per_pixel <= 0.0 ||
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
    config_.minimum_counterpart_points < 2 ||
    config_.minimum_counterpart_points > config_.minimum_points ||
    !std::isfinite(config_.centerline_midpoint_smoothing_weight) ||
    config_.centerline_midpoint_smoothing_weight <= 0.0 ||
    config_.centerline_midpoint_smoothing_weight > 1.0 ||
    !std::isfinite(config_.centerline_temporal_current_weight) ||
    config_.centerline_temporal_current_weight <= 0.0 ||
    config_.centerline_temporal_current_weight > 1.0 ||
    !std::isfinite(config_.centerline_transition_maximum_correction_m) ||
    config_.centerline_transition_maximum_correction_m < 0.0 ||
    !std::isfinite(config_.centerline_transition_correction_decay) ||
    config_.centerline_transition_correction_decay < 0.0 ||
    config_.centerline_transition_correction_decay >= 1.0 ||
    !std::isfinite(config_.centerline_tangent_window_m) ||
    config_.centerline_tangent_window_m <= 0.0 ||
    !std::isfinite(config_.centerline_maximum_curvature_per_m) ||
    config_.centerline_maximum_curvature_per_m <= 0.0 ||
    !std::isfinite(config_.centerline_maximum_heading_step_deg) ||
    config_.centerline_maximum_heading_step_deg <= 0.0 ||
    config_.centerline_maximum_heading_step_deg > 180.0 ||
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

  const int output_margin_px = static_cast<int>(std::llround(
      config_.output_lateral_margin_m / config_.meter_per_pixel));
  const int output_width = config_.image_width + 2 * output_margin_px;
  result.left_reconstructed_mask = cv::Mat::zeros(
    config_.image_height, output_width, CV_8UC1);
  result.right_reconstructed_mask = cv::Mat::zeros(
    config_.image_height, output_width, CV_8UC1);
  result.centerline_reconstructed_mask = cv::Mat::zeros(
    config_.image_height, output_width, CV_8UC1);
  result.reconstructed_mask = cv::Mat::zeros(
    config_.image_height, output_width, CV_8UC1);
  const auto seed_candidates = findSeedCandidates(
    result.candidate_mask, gray, config_);
  const Seed seed = selectSeedByTrackSupport(
    seed_candidates, result.candidate_mask, gray, config_,
    accepted_left_points_, accepted_right_points_);
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
  const int current_left_point_count = static_cast<int>(
    result.left_measured_points.size());
  const int current_right_point_count = static_cast<int>(
    result.right_measured_points.size());
  const LanePairGeometry current_pair_geometry = measureLanePairGeometry(
    result.left_measured_points, result.right_measured_points, config_);
  const bool short_counterpart_pair_valid =
    current_pair_geometry.valid &&
    std::max(current_left_point_count, current_right_point_count) >=
    config_.minimum_points &&
    std::min(current_left_point_count, current_right_point_count) >=
    config_.minimum_counterpart_points;
  const bool current_left_valid =
    current_left_point_count >= config_.minimum_points ||
    short_counterpart_pair_valid;
  const bool current_right_valid =
    current_right_point_count >= config_.minimum_points ||
    short_counterpart_pair_valid;
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
    (current_pair_geometry.valid || !measured_widths_m.empty()))
  {
    const double current_width_m = current_pair_geometry.valid ?
      current_pair_geometry.width_m : median(measured_widths_m);
    if (
      current_width_m >=
      config_.expected_lane_width_m - config_.lane_width_tolerance_m &&
      current_width_m <=
      config_.expected_lane_width_m + config_.lane_width_tolerance_m)
    {
      accepted_lane_width_m_ = accepted_lane_width_m_ > 0.0 ?
        0.80 * accepted_lane_width_m_ + 0.20 * current_width_m :
        current_width_m;
    }
  }
  const double lane_width_m = std::clamp(
    accepted_lane_width_m_ > 0.0 ?
    accepted_lane_width_m_ : config_.expected_lane_width_m,
    config_.expected_lane_width_m - config_.lane_width_tolerance_m,
    config_.expected_lane_width_m + config_.lane_width_tolerance_m);
  result.measured_lane_width_m = lane_width_m;

  const int stable_left_point_count = static_cast<int>(stable_left.points.size());
  const int stable_right_point_count = static_cast<int>(stable_right.points.size());
  const LanePairGeometry stable_pair_geometry = measureLanePairGeometry(
    stable_left.points, stable_right.points, config_);
  const bool stable_short_counterpart_pair_valid =
    stable_pair_geometry.valid &&
    std::max(stable_left_point_count, stable_right_point_count) >=
    config_.minimum_points &&
    std::min(stable_left_point_count, stable_right_point_count) >=
    config_.minimum_counterpart_points;
  const bool left_valid =
    stable_left_point_count >= config_.minimum_points ||
    stable_short_counterpart_pair_valid;
  const bool right_valid =
    stable_right_point_count >= config_.minimum_points ||
    stable_short_counterpart_pair_valid;
  if (!left_valid && !right_valid) {
    previous_centerline_points_.clear();
    previous_centerline_from_pair_ = false;
    single_boundary_transition_correction_m_ = 0.0;
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

  const double center_offset_m = 0.5 * config_.expected_lane_width_m;
  OffsetLaneResult center_from_left;
  OffsetLaneResult center_from_right;
  const int centerline_minimum_points = left_valid && right_valid ?
    config_.minimum_counterpart_points : config_.minimum_points;
  if (left_valid) {
    center_from_left = offsetFromReference(
      left_output, -center_offset_m, config_, centerline_minimum_points);
  }
  if (right_valid) {
    center_from_right = offsetFromReference(
      right_output, center_offset_m, config_, centerline_minimum_points);
  }

  std::vector<cv::Point2d> centerline_output;
  bool centerline_from_pair = false;
  bool pair_centerline_normal_offset_truncated = false;
  if (left_valid && right_valid) {
    centerline_output = centerlineFromMeasuredPair(
      left_output, right_output, center_from_left, center_from_right,
      config_, &pair_centerline_normal_offset_truncated);
    centerline_from_pair = !centerline_output.empty();
    if (centerline_from_pair && previous_centerline_from_pair_) {
      centerline_output = smoothCenterlineAgainstPrevious(
        centerline_output, previous_centerline_points_,
        config_.centerline_temporal_current_weight);
    }
    // A direct midpoint needs actual overlap. If the measured pair has too
    // little overlap, use one safe half-width estimate and report it as a
    // single-boundary centerline instead of averaging unrelated points.
    if (!centerline_from_pair) {
      const bool use_left = polylineArcLength(center_from_left.points) >=
        polylineArcLength(center_from_right.points);
      centerline_output = use_left ?
        center_from_left.points : center_from_right.points;
      result.centerline_from_single_boundary = !centerline_output.empty();
    }
  } else if (
    config_.centerline_from_single_boundary_enabled &&
    config_.allow_single_lane)
  {
    centerline_output = left_valid ?
      center_from_left.points : center_from_right.points;
    // The detected side label can momentarily flip even though the physical
    // boundary cannot cross the vehicle in one frame. Compare both normal
    // offset directions with the previous centerline and keep the continuous
    // candidate. This protects a right-only track that is briefly labelled
    // left (and vice versa) without holding an old path.
    if (!previous_centerline_points_.empty()) {
      const auto & reference = left_valid ? left_output : right_output;
      const double alternative_offset_m = left_valid ?
        center_offset_m : -center_offset_m;
      OffsetLaneResult alternative = offsetFromReference(
        reference, alternative_offset_m, config_, config_.minimum_points);
      const auto selected_error = centerlineMedianAbsoluteLateralError(
        previous_centerline_points_, centerline_output);
      const auto alternative_error = centerlineMedianAbsoluteLateralError(
        previous_centerline_points_, alternative.points);
      if (
        alternative_error.has_value() &&
        (!selected_error.has_value() ||
        *alternative_error + config_.meter_per_pixel < *selected_error))
      {
        centerline_output = std::move(alternative.points);
        if (left_valid) {
          center_from_left.normal_offset_truncated =
            alternative.normal_offset_truncated;
        } else {
          center_from_right.normal_offset_truncated =
            alternative.normal_offset_truncated;
        }
      }
    }
    result.centerline_from_single_boundary = true;
  }

  if (result.centerline_from_single_boundary && !centerline_output.empty()) {
    if (previous_centerline_from_pair_) {
      single_boundary_transition_correction_m_ = std::clamp(
        centerlineLateralCorrection(
          previous_centerline_points_, centerline_output),
        -config_.centerline_transition_maximum_correction_m,
        config_.centerline_transition_maximum_correction_m);
    } else {
      single_boundary_transition_correction_m_ *=
        config_.centerline_transition_correction_decay;
    }
    translateCenterlineLaterally(
      &centerline_output, single_boundary_transition_correction_m_);
  } else {
    single_boundary_transition_correction_m_ = 0.0;
  }
  if (!centerline_output.empty()) {
    previous_centerline_points_ = centerline_output;
    previous_centerline_from_pair_ = centerline_from_pair;
  } else {
    previous_centerline_points_.clear();
    previous_centerline_from_pair_ = false;
  }
  result.centerline_normal_offset_truncated =
    (centerline_from_pair && pair_centerline_normal_offset_truncated) ||
    (result.centerline_from_single_boundary &&
    (center_from_left.normal_offset_truncated ||
    center_from_right.normal_offset_truncated));
  result.centerline_point_count = static_cast<int>(centerline_output.size());

  left_output = extendByTangent(std::move(left_output), config_);
  right_output = extendByTangent(std::move(right_output), config_);
  centerline_output = extendByTangent(std::move(centerline_output), config_);
  drawMeasuredLane(
    left_output, config_, &result.left_reconstructed_mask);
  drawMeasuredLane(
    right_output, config_, &result.right_reconstructed_mask);
  result.reconstructed_maximum_x_m = drawMeasuredLane(
    centerline_output, config_, &result.centerline_reconstructed_mask);
  result.centerline_reconstructed_mask.copyTo(result.reconstructed_mask);
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
  previous_centerline_points_.clear();
  previous_centerline_from_pair_ = false;
  single_boundary_transition_correction_m_ = 0.0;
}

}  // namespace bev_processor
