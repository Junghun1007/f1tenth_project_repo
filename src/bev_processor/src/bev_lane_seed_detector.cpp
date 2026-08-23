#include "bev_processor/bev_lane_seed_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace bev_processor
{

namespace
{

struct SeedRun
{
  int row{0};
  int first_column{0};
  int last_column{0};
  double centroid_column{0.0};
  double mean_response{0.0};
};

struct SeedTrack
{
  std::vector<SeedRun> runs;
  double response_sum{0.0};
};

struct RunEvidence
{
  SeedRun run;
  double bilateral_contrast{0.0};
  double background_asymmetry{0.0};
  int contrast_relaxation_steps{0};
  bool contrast_valid{false};
};

struct TrackCandidate
{
  bool valid{false};
  std::vector<RunEvidence> evidence;
  cv::Point2d seed_point;
  double arc_length_px{0.0};
  double mean_contrast{0.0};
  double mean_response{0.0};
  double score{0.0};
};

bool isOddPositive(const int value)
{
  return value > 0 && value % 2 == 1;
}

void validateConfig(const BevLaneSeedDetectorConfig & config)
{
  const auto finiteNonNegative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  if (config.image_width <= 0 || config.image_height <= 0) {
    throw std::invalid_argument("lane seed image dimensions must be positive");
  }
  if (
    !finiteNonNegative(config.roi_bottom_exclusion_ratio) ||
    config.roi_bottom_exclusion_ratio >= 1.0 ||
    !std::isfinite(config.roi_height_ratio) ||
    config.roi_height_ratio <= 0.0 || config.roi_height_ratio > 1.0 ||
    config.roi_bottom_exclusion_ratio + config.roi_height_ratio > 1.0)
  {
    throw std::invalid_argument("lane seed ROI ratios are invalid");
  }
  if (
    config.minimum_top_hat_response < 1 ||
    config.minimum_top_hat_response > 255 ||
    config.minimum_run_width_px < 1 ||
    config.maximum_run_width_px < config.minimum_run_width_px)
  {
    throw std::invalid_argument("lane seed response or run-width range is invalid");
  }
  if (
    !finiteNonNegative(config.maximum_lateral_step_px) ||
    config.maximum_gap_rows < 0 || config.background_gap_px < 0 ||
    config.background_band_width_px < 1)
  {
    throw std::invalid_argument("lane seed search dimensions are invalid");
  }
  if (
    !finiteNonNegative(config.minimum_bilateral_contrast) ||
    !finiteNonNegative(config.maximum_background_asymmetry) ||
    !finiteNonNegative(config.minimum_track_arc_length_px) ||
    !finiteNonNegative(config.contrast_score_weight) ||
    !std::isfinite(config.contrast_relaxation_step) ||
    config.contrast_relaxation_step <= 0.0 ||
    config.contrast_relaxation_retry_count < 0)
  {
    throw std::invalid_argument("lane seed contrast parameters are invalid");
  }
  if (
    !isOddPositive(config.slope_median_window) ||
    !std::isfinite(config.maximum_slope_change_px_per_row) ||
    config.maximum_slope_change_px_per_row <= 0.0)
  {
    throw std::invalid_argument("lane seed slope parameters are invalid");
  }
  if (
    !finiteNonNegative(config.minimum_pair_distance_px) ||
    !finiteNonNegative(config.maximum_pair_distance_px) ||
    config.maximum_pair_distance_px < config.minimum_pair_distance_px)
  {
    throw std::invalid_argument("lane seed pair-distance range is invalid");
  }
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  return values.size() % 2 == 0 ?
         0.5 * (values[middle - 1] + values[middle]) : values[middle];
}

std::vector<SeedRun> findRuns(
  const cv::Mat & response,
  const int row,
  const BevLaneSeedDetectorConfig & config)
{
  std::vector<SeedRun> runs;
  runs.reserve(12);
  const auto * values = response.ptr<std::uint8_t>(row);
  int column = 0;
  while (column < response.cols) {
    while (
      column < response.cols &&
      values[column] < config.minimum_top_hat_response)
    {
      ++column;
    }
    if (column >= response.cols) {
      break;
    }
    const int first = column;
    double response_sum = 0.0;
    double weighted_column_sum = 0.0;
    while (
      column < response.cols &&
      values[column] >= config.minimum_top_hat_response)
    {
      const double weight = static_cast<double>(values[column]);
      response_sum += weight;
      weighted_column_sum += weight * static_cast<double>(column);
      ++column;
    }
    const int last = column - 1;
    const int width = last - first + 1;
    if (
      width < config.minimum_run_width_px ||
      width > config.maximum_run_width_px || response_sum <= 0.0)
    {
      continue;
    }
    runs.push_back(SeedRun{
      row, first, last, weighted_column_sum / response_sum,
      response_sum / static_cast<double>(width)});
  }
  return runs;
}

std::vector<SeedTrack> buildTracks(
  const cv::Mat & response,
  const int roi_top,
  const int roi_bottom,
  const BevLaneSeedDetectorConfig & config)
{
  std::vector<SeedTrack> tracks;
  tracks.reserve(32);
  for (int row = roi_bottom - 1; row >= roi_top; --row) {
    std::vector<SeedRun> runs = findRuns(response, row, config);
    std::sort(
      runs.begin(), runs.end(),
      [](const SeedRun & left, const SeedRun & right) {
        return left.mean_response > right.mean_response;
      });
    std::vector<bool> used(tracks.size(), false);
    for (const SeedRun & run : runs) {
      std::size_t best_index = tracks.size();
      double best_distance = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < tracks.size(); ++index) {
        if (used[index] || tracks[index].runs.empty()) {
          continue;
        }
        const SeedRun & previous = tracks[index].runs.back();
        const int row_gap = previous.row - run.row;
        if (row_gap <= 0 || row_gap > config.maximum_gap_rows + 1) {
          continue;
        }
        const double lateral_distance = std::abs(
          run.centroid_column - previous.centroid_column);
        if (
          lateral_distance <= config.maximum_lateral_step_px * row_gap &&
          lateral_distance < best_distance)
        {
          best_index = index;
          best_distance = lateral_distance;
        }
      }
      if (best_index == tracks.size()) {
        SeedTrack track;
        track.runs.reserve(static_cast<std::size_t>(roi_bottom - roi_top));
        track.runs.push_back(run);
        track.response_sum = run.mean_response;
        tracks.push_back(std::move(track));
        used.push_back(true);
      } else {
        tracks[best_index].runs.push_back(run);
        tracks[best_index].response_sum += run.mean_response;
        used[best_index] = true;
      }
    }
  }
  return tracks;
}

double trackArcLength(const std::vector<SeedRun> & runs)
{
  double length = 0.0;
  for (std::size_t index = 1; index < runs.size(); ++index) {
    length += std::hypot(
      runs[index].centroid_column - runs[index - 1].centroid_column,
      static_cast<double>(runs[index].row - runs[index - 1].row));
  }
  return length;
}

std::vector<double> medianSmoothedColumns(
  const SeedTrack & track,
  const int window)
{
  std::vector<double> result;
  result.reserve(track.runs.size());
  const int half = window / 2;
  for (std::size_t index = 0; index < track.runs.size(); ++index) {
    const std::size_t first = index > static_cast<std::size_t>(half) ?
      index - static_cast<std::size_t>(half) : 0;
    const std::size_t last = std::min(
      track.runs.size() - 1, index + static_cast<std::size_t>(half));
    std::vector<double> values;
    values.reserve(last - first + 1);
    for (std::size_t sample = first; sample <= last; ++sample) {
      values.push_back(track.runs[sample].centroid_column);
    }
    result.push_back(median(std::move(values)));
  }
  return result;
}

std::vector<SeedTrack> splitAbruptSlopeChanges(
  const std::vector<SeedTrack> & tracks,
  const BevLaneSeedDetectorConfig & config,
  std::vector<cv::Point2d> * break_points)
{
  if (!config.slope_filter_enabled) {
    return tracks;
  }
  std::vector<SeedTrack> result;
  result.reserve(tracks.size());
  for (const SeedTrack & track : tracks) {
    if (track.runs.size() < 3) {
      result.push_back(track);
      continue;
    }
    const std::vector<double> columns = medianSmoothedColumns(
      track, config.slope_median_window);
    const bool show_break_points =
      trackArcLength(track.runs) >= config.minimum_track_arc_length_px;
    std::vector<double> slopes;
    slopes.reserve(track.runs.size() - 1);
    for (std::size_t index = 1; index < track.runs.size(); ++index) {
      const int row_gap = track.runs[index - 1].row - track.runs[index].row;
      slopes.push_back(
        (columns[index] - columns[index - 1]) /
        static_cast<double>(row_gap));
    }
    std::vector<std::size_t> splits;
    for (std::size_t index = 1; index < slopes.size(); ++index) {
      if (
        std::abs(slopes[index] - slopes[index - 1]) >
        config.maximum_slope_change_px_per_row)
      {
        const std::size_t split = index + 1;
        splits.push_back(split);
        if (break_points != nullptr && show_break_points) {
          break_points->emplace_back(
            track.runs[split].centroid_column,
            static_cast<double>(track.runs[split].row));
        }
        ++index;
      }
    }
    std::size_t first = 0;
    const auto appendSegment = [&track, &result](
        const std::size_t begin, const std::size_t end) {
        if (end <= begin) {
          return;
        }
        SeedTrack segment;
        segment.runs.insert(
          segment.runs.end(),
          track.runs.begin() + static_cast<std::ptrdiff_t>(begin),
          track.runs.begin() + static_cast<std::ptrdiff_t>(end));
        for (const SeedRun & run : segment.runs) {
          segment.response_sum += run.mean_response;
        }
        result.push_back(std::move(segment));
      };
    for (const std::size_t split : splits) {
      appendSegment(first, split);
      first = split;
    }
    appendSegment(first, track.runs.size());
  }
  return result;
}

bool samplePixel(
  const cv::Mat & image,
  const cv::Point2d & center,
  const cv::Point2d & direction,
  const double distance,
  double * value)
{
  if (value == nullptr) {
    return false;
  }
  const int column = static_cast<int>(std::lround(
      center.x + distance * direction.x));
  const int row = static_cast<int>(std::lround(
      center.y + distance * direction.y));
  if (column < 0 || column >= image.cols || row < 0 || row >= image.rows) {
    return false;
  }
  *value = static_cast<double>(image.at<std::uint8_t>(row, column));
  return true;
}

cv::Point2d trackTangent(const SeedTrack & track, const std::size_t index)
{
  cv::Point2d tangent;
  if (index == 0) {
    tangent = cv::Point2d(
      track.runs[1].centroid_column - track.runs[0].centroid_column,
      static_cast<double>(track.runs[1].row - track.runs[0].row));
  } else if (index + 1 == track.runs.size()) {
    tangent = cv::Point2d(
      track.runs[index].centroid_column -
      track.runs[index - 1].centroid_column,
      static_cast<double>(
        track.runs[index].row - track.runs[index - 1].row));
  } else {
    tangent = cv::Point2d(
      track.runs[index + 1].centroid_column -
      track.runs[index - 1].centroid_column,
      static_cast<double>(
        track.runs[index + 1].row - track.runs[index - 1].row));
  }
  const double length = std::hypot(tangent.x, tangent.y);
  return length > 1.0e-9 ? tangent * (1.0 / length) : cv::Point2d(0.0, -1.0);
}

int ridgeExtent(
  const cv::Mat & response,
  const cv::Point2d & center,
  const cv::Point2d & normal,
  const int direction,
  const BevLaneSeedDetectorConfig & config)
{
  int extent = 0;
  for (int step = 1; step <= config.maximum_run_width_px; ++step) {
    double value = 0.0;
    if (
      !samplePixel(response, center, normal, direction * step, &value) ||
      value < config.minimum_top_hat_response)
    {
      break;
    }
    extent = step;
  }
  return extent;
}

RunEvidence evaluateRun(
  const SeedTrack & track,
  const std::size_t index,
  const cv::Mat & gray,
  const cv::Mat & response,
  const BevLaneSeedDetectorConfig & config)
{
  RunEvidence evidence;
  evidence.run = track.runs[index];
  if (track.runs.size() < 2) {
    return evidence;
  }
  const cv::Point2d center(
    evidence.run.centroid_column, static_cast<double>(evidence.run.row));
  const cv::Point2d tangent = trackTangent(track, index);
  cv::Point2d normal(-tangent.y, tangent.x);
  if (normal.x < 0.0) {
    normal *= -1.0;
  }
  const int negative_extent = ridgeExtent(
    response, center, normal, -1, config);
  const int positive_extent = ridgeExtent(
    response, center, normal, 1, config);

  std::vector<double> center_values;
  center_values.reserve(static_cast<std::size_t>(
    negative_extent + positive_extent + 1));
  for (int offset = -negative_extent; offset <= positive_extent; ++offset) {
    double value = 0.0;
    if (samplePixel(gray, center, normal, offset, &value)) {
      center_values.push_back(value);
    }
  }
  std::vector<double> negative_background;
  std::vector<double> positive_background;
  negative_background.reserve(
    static_cast<std::size_t>(config.background_band_width_px));
  positive_background.reserve(
    static_cast<std::size_t>(config.background_band_width_px));
  const int negative_first =
    negative_extent + config.background_gap_px + 1;
  const int positive_first =
    positive_extent + config.background_gap_px + 1;
  for (int band_index = 0;
    band_index < config.background_band_width_px; ++band_index)
  {
    double value = 0.0;
    if (samplePixel(
        gray, center, normal, -(negative_first + band_index), &value))
    {
      negative_background.push_back(value);
    }
    if (samplePixel(
        gray, center, normal, positive_first + band_index, &value))
    {
      positive_background.push_back(value);
    }
  }
  if (
    center_values.empty() || negative_background.empty() ||
    positive_background.empty())
  {
    return evidence;
  }
  const double center_brightness = median(std::move(center_values));
  const double negative_brightness = median(std::move(negative_background));
  const double positive_brightness = median(std::move(positive_background));
  evidence.bilateral_contrast = std::min(
    center_brightness - negative_brightness,
    center_brightness - positive_brightness);
  evidence.background_asymmetry = std::abs(
    negative_brightness - positive_brightness);

  double required_contrast = config.minimum_bilateral_contrast;
  const int retry_count = config.contrast_relaxation_enabled ?
    config.contrast_relaxation_retry_count : 0;
  bool contrast_passed = false;
  for (int attempt = 0; attempt <= retry_count; ++attempt) {
    if (evidence.bilateral_contrast >= required_contrast) {
      evidence.contrast_relaxation_steps = attempt;
      contrast_passed = true;
      break;
    }
    required_contrast = std::max(
      0.0, required_contrast - config.contrast_relaxation_step);
  }
  evidence.contrast_valid = contrast_passed &&
    evidence.background_asymmetry <= config.maximum_background_asymmetry;
  return evidence;
}

TrackCandidate evaluateTrack(
  const SeedTrack & track,
  const cv::Mat & gray,
  const cv::Mat & response,
  const BevLaneSeedDetectorConfig & config,
  std::vector<cv::Point2d> * contrast_points,
  std::vector<cv::Point2d> * relaxed_points)
{
  TrackCandidate best;
  if (track.runs.size() < 2) {
    return best;
  }
  std::vector<RunEvidence> current;
  current.reserve(track.runs.size());
  const auto finishSegment = [&best, &config](
      std::vector<RunEvidence> * segment) {
      if (segment == nullptr || segment->size() < 2) {
        if (segment != nullptr) {
          segment->clear();
        }
        return;
      }
      std::vector<SeedRun> runs;
      runs.reserve(segment->size());
      double contrast_sum = 0.0;
      double response_sum = 0.0;
      for (const RunEvidence & evidence : *segment) {
        runs.push_back(evidence.run);
        contrast_sum += evidence.bilateral_contrast;
        response_sum += evidence.run.mean_response;
      }
      const double arc_length = trackArcLength(runs);
      const double mean_contrast = contrast_sum / segment->size();
      const double mean_response = response_sum / segment->size();
      const double score = arc_length +
        config.contrast_score_weight * mean_contrast +
        0.05 * mean_response;
      if (!best.valid || score > best.score) {
        best.valid = true;
        best.evidence = *segment;
        best.seed_point = cv::Point2d(
          segment->front().run.centroid_column,
          static_cast<double>(segment->front().run.row));
        best.arc_length_px = arc_length;
        best.mean_contrast = mean_contrast;
        best.mean_response = mean_response;
        best.score = score;
      }
      segment->clear();
    };

  for (std::size_t index = 0; index < track.runs.size(); ++index) {
    RunEvidence evidence = evaluateRun(track, index, gray, response, config);
    if (!evidence.contrast_valid) {
      finishSegment(&current);
      continue;
    }
    if (contrast_points != nullptr) {
      contrast_points->emplace_back(
        evidence.run.centroid_column, static_cast<double>(evidence.run.row));
    }
    if (relaxed_points != nullptr && evidence.contrast_relaxation_steps > 0) {
      relaxed_points->emplace_back(
        evidence.run.centroid_column, static_cast<double>(evidence.run.row));
    }
    if (!current.empty()) {
      const SeedRun & previous = current.back().run;
      const int row_gap = previous.row - evidence.run.row;
      const double lateral = std::abs(
        previous.centroid_column - evidence.run.centroid_column);
      if (
        row_gap <= 0 || row_gap > config.maximum_gap_rows + 1 ||
        lateral > config.maximum_lateral_step_px * row_gap)
      {
        finishSegment(&current);
      }
    }
    current.push_back(std::move(evidence));
  }
  finishSegment(&current);
  if (best.valid && best.arc_length_px < config.minimum_track_arc_length_px) {
    best = TrackCandidate{};
  }
  return best;
}

bool candidateColumnAtRow(
  const TrackCandidate & candidate,
  const int row,
  double * column)
{
  if (
    column == nullptr || candidate.evidence.empty() ||
    row > candidate.evidence.front().run.row ||
    row < candidate.evidence.back().run.row)
  {
    return false;
  }
  for (std::size_t index = 0; index < candidate.evidence.size(); ++index) {
    const SeedRun & current = candidate.evidence[index].run;
    if (current.row == row) {
      *column = current.centroid_column;
      return true;
    }
    if (index + 1 >= candidate.evidence.size()) {
      continue;
    }
    const SeedRun & next = candidate.evidence[index + 1].run;
    if (current.row > row && row > next.row) {
      const double ratio = static_cast<double>(current.row - row) /
        static_cast<double>(current.row - next.row);
      *column = current.centroid_column +
        ratio * (next.centroid_column - current.centroid_column);
      return true;
    }
  }
  return false;
}

bool sameRowPairDistance(
  const TrackCandidate & left,
  const TrackCandidate & right,
  double * distance,
  int * sample_count,
  int * inlier_count)
{
  if (
    distance == nullptr || sample_count == nullptr || inlier_count == nullptr ||
    left.evidence.empty() || right.evidence.empty())
  {
    return false;
  }
  const int overlap_bottom = std::min(
    left.evidence.front().run.row, right.evidence.front().run.row);
  const int overlap_top = std::max(
    left.evidence.back().run.row, right.evidence.back().run.row);
  if (overlap_bottom < overlap_top) {
    return false;
  }
  std::vector<double> distances;
  distances.reserve(static_cast<std::size_t>(overlap_bottom - overlap_top + 1));
  for (int row = overlap_top; row <= overlap_bottom; ++row) {
    double left_column = 0.0;
    double right_column = 0.0;
    if (
      candidateColumnAtRow(left, row, &left_column) &&
      candidateColumnAtRow(right, row, &right_column))
    {
      distances.push_back(std::abs(right_column - left_column));
    }
  }
  if (distances.empty()) {
    return false;
  }
  const double median_distance = median(distances);
  std::vector<double> deviations;
  deviations.reserve(distances.size());
  for (const double value : distances) {
    deviations.push_back(std::abs(value - median_distance));
  }
  const double maximum_deviation = std::max(3.0, 3.0 * median(deviations));
  double sum = 0.0;
  int inliers = 0;
  for (const double value : distances) {
    if (std::abs(value - median_distance) <= maximum_deviation) {
      sum += value;
      ++inliers;
    }
  }
  if (inliers == 0) {
    return false;
  }
  *distance = sum / static_cast<double>(inliers);
  *sample_count = static_cast<int>(distances.size());
  *inlier_count = inliers;
  return true;
}

BevLaneSeed toPublicSeed(const TrackCandidate & candidate)
{
  BevLaneSeed seed;
  seed.valid = candidate.valid;
  seed.image_point = candidate.seed_point;
  seed.arc_length_px = candidate.arc_length_px;
  seed.mean_bilateral_contrast = candidate.mean_contrast;
  seed.score = candidate.score;
  seed.support_points.reserve(candidate.evidence.size());
  for (const RunEvidence & evidence : candidate.evidence) {
    seed.support_points.emplace_back(
      evidence.run.centroid_column, static_cast<double>(evidence.run.row));
  }
  return seed;
}

void drawCandidate(
  cv::Mat * image,
  const TrackCandidate & candidate,
  const cv::Scalar & color)
{
  if (image == nullptr || !candidate.valid) {
    return;
  }
  std::vector<cv::Point> points;
  points.reserve(candidate.evidence.size());
  for (const RunEvidence & evidence : candidate.evidence) {
    cv::line(
      *image,
      cv::Point(evidence.run.first_column, evidence.run.row),
      cv::Point(evidence.run.last_column, evidence.run.row),
      color, 1, cv::LINE_8);
    points.emplace_back(
      static_cast<int>(std::lround(evidence.run.centroid_column)),
      evidence.run.row);
  }
  if (points.size() >= 2) {
    cv::polylines(*image, points, false, color, 1, cv::LINE_AA);
  }
  const cv::Point seed(
    static_cast<int>(std::lround(candidate.seed_point.x)),
    static_cast<int>(std::lround(candidate.seed_point.y)));
  cv::drawMarker(
    *image, seed, color, cv::MARKER_CROSS, 7, 1, cv::LINE_AA);
  cv::circle(*image, seed, 2, color, cv::FILLED, cv::LINE_AA);
}

void drawMaskCandidate(cv::Mat * mask, const TrackCandidate & candidate)
{
  if (mask == nullptr || !candidate.valid) {
    return;
  }
  std::vector<cv::Point> points;
  points.reserve(candidate.evidence.size());
  for (const RunEvidence & evidence : candidate.evidence) {
    points.emplace_back(
      static_cast<int>(std::lround(evidence.run.centroid_column)),
      evidence.run.row);
  }
  if (points.size() >= 2) {
    cv::polylines(*mask, points, false, cv::Scalar(255), 1, cv::LINE_8);
  } else if (!points.empty()) {
    cv::circle(*mask, points.front(), 1, cv::Scalar(255), cv::FILLED);
  }
}

}  // namespace

BevLaneSeedDetector::BevLaneSeedDetector(BevLaneSeedDetectorConfig config)
: config_(std::move(config))
{
  validateConfig(config_);
}

BevLaneSeedDetection BevLaneSeedDetector::detect(
  const cv::Mat & gray,
  const cv::Mat & enhanced_top_hat,
  const bool create_preview) const
{
  if (
    gray.type() != CV_8UC1 || enhanced_top_hat.type() != CV_8UC1 ||
    gray.size() != enhanced_top_hat.size() ||
    gray.cols != config_.image_width || gray.rows != config_.image_height)
  {
    throw std::invalid_argument(
            "lane seed detector expects matching configured MONO8 images");
  }

  BevLaneSeedDetection result;
  const int excluded_rows = static_cast<int>(std::lround(
      enhanced_top_hat.rows * config_.roi_bottom_exclusion_ratio));
  const int roi_height = std::max(
    1, static_cast<int>(std::lround(
      enhanced_top_hat.rows * config_.roi_height_ratio)));
  result.roi_bottom_row = std::clamp(
    enhanced_top_hat.rows - excluded_rows, 1, enhanced_top_hat.rows);
  result.roi_top_row = std::max(0, result.roi_bottom_row - roi_height);

  std::vector<SeedTrack> tracks = buildTracks(
    enhanced_top_hat, result.roi_top_row, result.roi_bottom_row, config_);
  std::vector<cv::Point2d> slope_break_points;
  tracks = splitAbruptSlopeChanges(tracks, config_, &slope_break_points);
  result.slope_break_count = static_cast<int>(slope_break_points.size());

  std::vector<cv::Point2d> contrast_points;
  std::vector<cv::Point2d> relaxed_points;
  std::vector<TrackCandidate> candidates;
  candidates.reserve(tracks.size());
  BevLaneSeedDetectorConfig strict_config = config_;
  strict_config.contrast_relaxation_enabled = false;
  for (const SeedTrack & track : tracks) {
    const TrackCandidate strict_candidate = evaluateTrack(
      track, gray, enhanced_top_hat, strict_config, nullptr, nullptr);
    if (!strict_candidate.valid) {
      continue;
    }
    TrackCandidate candidate = evaluateTrack(
      track, gray, enhanced_top_hat, config_,
      &contrast_points, &relaxed_points);
    if (candidate.valid) {
      candidates.push_back(std::move(candidate));
    }
  }
  result.accepted_track_count = static_cast<int>(candidates.size());
  result.strict_evidence_count = static_cast<int>(contrast_points.size()) -
    static_cast<int>(relaxed_points.size());
  result.relaxed_evidence_count = static_cast<int>(relaxed_points.size());

  TrackCandidate selected_left;
  TrackCandidate selected_right;
  const double center_column =
    0.5 * static_cast<double>(enhanced_top_hat.cols - 1);
  double best_pair_score = -std::numeric_limits<double>::infinity();
  for (const TrackCandidate & left : candidates) {
    if (left.seed_point.x >= center_column) {
      continue;
    }
    for (const TrackCandidate & right : candidates) {
      if (right.seed_point.x < center_column) {
        continue;
      }
      double distance = 0.0;
      int samples = 0;
      int inliers = 0;
      if (!sameRowPairDistance(
          left, right, &distance, &samples, &inliers))
      {
        continue;
      }
      if (
        distance < config_.minimum_pair_distance_px ||
        distance > config_.maximum_pair_distance_px)
      {
        continue;
      }
      const double target = 0.5 * (
        config_.minimum_pair_distance_px + config_.maximum_pair_distance_px);
      const double score = left.score + right.score -
        0.5 * std::abs(distance - target);
      if (score > best_pair_score) {
        best_pair_score = score;
        selected_left = left;
        selected_right = right;
        result.pair_valid = true;
        result.pair_distance_px = distance;
        result.pair_distance_samples = samples;
        result.pair_distance_inliers = inliers;
      }
    }
  }
  if (!result.pair_valid && !candidates.empty()) {
    const auto strongest = std::max_element(
      candidates.begin(), candidates.end(),
      [](const TrackCandidate & left, const TrackCandidate & right) {
        return left.score < right.score;
      });
    if (strongest->seed_point.x < center_column) {
      selected_left = *strongest;
    } else {
      selected_right = *strongest;
    }
  }

  result.left = toPublicSeed(selected_left);
  result.right = toPublicSeed(selected_right);
  result.seed_mask = cv::Mat::zeros(
    enhanced_top_hat.size(), CV_8UC1);
  drawMaskCandidate(&result.seed_mask, selected_left);
  drawMaskCandidate(&result.seed_mask, selected_right);

  if (create_preview) {
    cv::cvtColor(enhanced_top_hat, result.preview, cv::COLOR_GRAY2BGR);
    for (const cv::Point2d & point : contrast_points) {
      cv::circle(
        result.preview,
        cv::Point(
          static_cast<int>(std::lround(point.x)),
          static_cast<int>(std::lround(point.y))),
        1, cv::Scalar(80, 120, 80), cv::FILLED, cv::LINE_8);
    }
    for (const cv::Point2d & point : relaxed_points) {
      cv::circle(
        result.preview,
        cv::Point(
          static_cast<int>(std::lround(point.x)),
          static_cast<int>(std::lround(point.y))),
        1, cv::Scalar(0, 165, 255), cv::FILLED, cv::LINE_8);
    }
    cv::line(
      result.preview, cv::Point(0, result.roi_top_row),
      cv::Point(result.preview.cols - 1, result.roi_top_row),
      cv::Scalar(0, 255, 255), 1, cv::LINE_8);
    cv::line(
      result.preview, cv::Point(0, result.roi_bottom_row - 1),
      cv::Point(result.preview.cols - 1, result.roi_bottom_row - 1),
      cv::Scalar(0, 255, 255), 1, cv::LINE_8);
    drawCandidate(&result.preview, selected_left, cv::Scalar(255, 255, 0));
    drawCandidate(&result.preview, selected_right, cv::Scalar(255, 0, 255));
    for (const cv::Point2d & point : slope_break_points) {
      cv::drawMarker(
        result.preview,
        cv::Point(
          static_cast<int>(std::lround(point.x)),
          static_cast<int>(std::lround(point.y))),
        cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS, 7, 1,
        cv::LINE_AA);
    }
  }
  return result;
}

}  // namespace bev_processor
