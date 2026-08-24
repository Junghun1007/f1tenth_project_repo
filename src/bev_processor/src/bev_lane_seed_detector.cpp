#include "bev_processor/bev_lane_seed_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
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
  bool transposed{false};
  double bilateral_contrast{0.0};
  double background_asymmetry{0.0};
  int contrast_relaxation_steps{0};
  bool local_samples_valid{false};
  bool contrast_valid{false};
};

struct TrackCandidate
{
  bool valid{false};
  bool has_row_support{false};
  bool has_column_support{false};
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
  if (
    !finiteNonNegative(
      config.cross_direction_merge_maximum_endpoint_distance_px) ||
    !std::isfinite(
      config.cross_direction_merge_minimum_connector_support_ratio) ||
    config.cross_direction_merge_minimum_connector_support_ratio < 0.0 ||
    config.cross_direction_merge_minimum_connector_support_ratio > 1.0 ||
    !std::isfinite(
      config.cross_direction_merge_maximum_turn_angle_deg) ||
    config.cross_direction_merge_maximum_turn_angle_deg <= 0.0 ||
    config.cross_direction_merge_maximum_turn_angle_deg > 180.0)
  {
    throw std::invalid_argument("lane seed cross-direction merge settings are invalid");
  }
  if (
    config.temporal_side_lock_reset_frames < 1 ||
    !std::isfinite(config.temporal_side_reacquire_maximum_distance_px) ||
    config.temporal_side_reacquire_maximum_distance_px <= 0.0)
  {
    throw std::invalid_argument("lane seed temporal side-lock settings are invalid");
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
  const int first_search_column,
  const int last_search_column,
  const BevLaneSeedDetectorConfig & config)
{
  std::vector<SeedRun> runs;
  runs.reserve(12);
  const auto * values = response.ptr<std::uint8_t>(row);
  int column = first_search_column;
  while (column < last_search_column) {
    while (
      column < last_search_column &&
      values[column] < config.minimum_top_hat_response)
    {
      ++column;
    }
    if (column >= last_search_column) {
      break;
    }
    const int first = column;
    double response_sum = 0.0;
    double weighted_column_sum = 0.0;
    while (
      column < last_search_column &&
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
  const int first_scan_row,
  const int last_scan_row,
  const int first_search_column,
  const int last_search_column,
  const BevLaneSeedDetectorConfig & config)
{
  std::vector<SeedTrack> tracks;
  tracks.reserve(32);
  for (int row = last_scan_row - 1; row >= first_scan_row; --row) {
    std::vector<SeedRun> runs = findRuns(
      response, row, first_search_column, last_search_column, config);
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
        track.runs.reserve(
          static_cast<std::size_t>(last_scan_row - first_scan_row));
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
  evidence.local_samples_valid = true;
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

  int pending_contrast_gap = 0;
  for (std::size_t index = 0; index < track.runs.size(); ++index) {
    RunEvidence evidence = evaluateRun(track, index, gray, response, config);
    if (!evidence.contrast_valid) {
      const bool contrast_only_failure = evidence.local_samples_valid &&
        evidence.background_asymmetry <= config.maximum_background_asymmetry;
      if (
        contrast_only_failure && !current.empty() &&
        pending_contrast_gap < config.maximum_gap_rows)
      {
        ++pending_contrast_gap;
        continue;
      }
      finishSegment(&current);
      pending_contrast_gap = 0;
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
    pending_contrast_gap = 0;
  }
  finishSegment(&current);
  if (best.valid && best.arc_length_px < config.minimum_track_arc_length_px) {
    best = TrackCandidate{};
  }
  return best;
}

cv::Point2d scanPointToOriginal(
  const cv::Point2d & point,
  const bool transposed)
{
  return transposed ? cv::Point2d(point.y, point.x) : point;
}

cv::Point2d evidenceCenterOriginal(
  const RunEvidence & evidence)
{
  return scanPointToOriginal(
    cv::Point2d(
      evidence.run.centroid_column,
      static_cast<double>(evidence.run.row)),
    evidence.transposed);
}

cv::Point2d candidateSeedOriginal(const TrackCandidate & candidate)
{
  if (candidate.evidence.empty()) {
    return cv::Point2d();
  }
  cv::Point2d seed = evidenceCenterOriginal(candidate.evidence.front());
  for (const RunEvidence & evidence : candidate.evidence) {
    const cv::Point2d point = evidenceCenterOriginal(evidence);
    if (point.y > seed.y) {
      seed = point;
    }
  }
  return seed;
}

struct CandidateExtraction
{
  std::vector<TrackCandidate> candidates;
  std::vector<cv::Point2d> contrast_points;
  std::vector<cv::Point2d> relaxed_points;
  std::vector<cv::Point2d> slope_break_points;
};

CandidateExtraction extractCandidates(
  const cv::Mat & gray,
  const cv::Mat & response,
  const int first_scan_row,
  const int last_scan_row,
  const int first_search_column,
  const int last_search_column,
  const BevLaneSeedDetectorConfig & config,
  const bool transposed)
{
  CandidateExtraction extraction;
  std::vector<SeedTrack> tracks = buildTracks(
    response, first_scan_row, last_scan_row,
    first_search_column, last_search_column, config);
  std::vector<cv::Point2d> scan_slope_break_points;
  tracks = splitAbruptSlopeChanges(
    tracks, config, &scan_slope_break_points);

  BevLaneSeedDetectorConfig strict_config = config;
  strict_config.contrast_relaxation_enabled = false;
  for (const SeedTrack & track : tracks) {
    const TrackCandidate strict_candidate = evaluateTrack(
      track, gray, response, strict_config, nullptr, nullptr);
    if (!strict_candidate.valid) {
      continue;
    }
    std::vector<cv::Point2d> scan_contrast_points;
    std::vector<cv::Point2d> scan_relaxed_points;
    TrackCandidate candidate = evaluateTrack(
      track, gray, response, config,
      &scan_contrast_points, &scan_relaxed_points);
    if (!candidate.valid) {
      continue;
    }
    candidate.has_row_support = !transposed;
    candidate.has_column_support = transposed;
    for (RunEvidence & evidence : candidate.evidence) {
      evidence.transposed = transposed;
    }
    candidate.seed_point = candidateSeedOriginal(candidate);
    extraction.candidates.push_back(std::move(candidate));
    for (const cv::Point2d & point : scan_contrast_points) {
      extraction.contrast_points.push_back(
        scanPointToOriginal(point, transposed));
    }
    for (const cv::Point2d & point : scan_relaxed_points) {
      extraction.relaxed_points.push_back(
        scanPointToOriginal(point, transposed));
    }
  }
  for (const cv::Point2d & point : scan_slope_break_points) {
    extraction.slope_break_points.push_back(
      scanPointToOriginal(point, transposed));
  }
  return extraction;
}

double candidateArcLengthOriginal(const std::vector<RunEvidence> & evidence)
{
  double length = 0.0;
  for (std::size_t index = 1; index < evidence.size(); ++index) {
    length += cv::norm(
      evidenceCenterOriginal(evidence[index]) -
      evidenceCenterOriginal(evidence[index - 1]));
  }
  return length;
}

double connectorSupportRatio(
  const cv::Mat & response,
  const cv::Point2d & first,
  const cv::Point2d & second,
  const BevLaneSeedDetectorConfig & config)
{
  const int samples = std::max(
    2, static_cast<int>(std::ceil(cv::norm(second - first))) + 1);
  int supported = 0;
  int inside = 0;
  for (int index = 0; index < samples; ++index) {
    const double ratio = static_cast<double>(index) /
      static_cast<double>(samples - 1);
    const cv::Point2d point = first + ratio * (second - first);
    const int column = static_cast<int>(std::lround(point.x));
    const int row = static_cast<int>(std::lround(point.y));
    if (column < 0 || column >= response.cols || row < 0 || row >= response.rows) {
      continue;
    }
    ++inside;
    if (response.at<std::uint8_t>(row, column) >=
      config.minimum_top_hat_response)
    {
      ++supported;
    }
  }
  return inside > 0 ?
         static_cast<double>(supported) / static_cast<double>(inside) : 0.0;
}

bool perpendicularRunsOverlap(
  const RunEvidence & first,
  const RunEvidence & second,
  cv::Point2d * intersection)
{
  if (first.transposed == second.transposed || intersection == nullptr) {
    return false;
  }
  const RunEvidence & row_evidence = first.transposed ? second : first;
  const RunEvidence & column_evidence = first.transposed ? first : second;
  const double column = static_cast<double>(column_evidence.run.row);
  const double row = static_cast<double>(row_evidence.run.row);
  if (
    column < row_evidence.run.first_column ||
    column > row_evidence.run.last_column ||
    row < column_evidence.run.first_column ||
    row > column_evidence.run.last_column)
  {
    return false;
  }
  *intersection = cv::Point2d(column, row);
  return true;
}

std::vector<RunEvidence> retainedArm(
  const std::vector<RunEvidence> & evidence,
  const std::size_t joint_index,
  const bool toward_front,
  const bool joint_last)
{
  std::vector<RunEvidence> arm;
  if (toward_front) {
    arm.insert(
      arm.end(), evidence.begin(),
      evidence.begin() + static_cast<std::ptrdiff_t>(joint_index + 1));
  } else {
    arm.insert(
      arm.end(),
      evidence.begin() + static_cast<std::ptrdiff_t>(joint_index),
      evidence.end());
  }
  if ((joint_last && !toward_front) || (!joint_last && toward_front)) {
    std::reverse(arm.begin(), arm.end());
  }
  return arm;
}

struct MergeProposal
{
  bool valid{false};
  std::size_t first_index{0};
  std::size_t second_index{0};
  std::size_t first_joint_index{0};
  std::size_t second_joint_index{0};
  bool first_arm_toward_front{false};
  bool second_arm_toward_front{false};
  bool overlap_joint{false};
  cv::Point2d joint_point;
  double endpoint_distance_px{0.0};
  double connector_support_ratio{0.0};
  double turn_angle_deg{0.0};
  double retained_arc_length_px{0.0};
  double quality{-std::numeric_limits<double>::infinity()};
};

MergeProposal evaluateMergeProposal(
  const TrackCandidate & first,
  const TrackCandidate & second,
  const cv::Mat & response,
  const BevLaneSeedDetectorConfig & config)
{
  if (
    !first.valid || !second.valid || first.evidence.size() < 2 ||
    second.evidence.size() < 2)
  {
    return MergeProposal{};
  }

  MergeProposal best;
  const auto evaluate_joint = [&first, &second, &response, &config, &best](
      const std::size_t first_index,
      const std::size_t second_index,
      const cv::Point2d & joint_point,
      const bool overlap_joint) {
      const cv::Point2d first_joint = evidenceCenterOriginal(
        first.evidence[first_index]);
      const cv::Point2d second_joint = evidenceCenterOriginal(
        second.evidence[second_index]);
      const double joint_distance = cv::norm(second_joint - first_joint);
      if (
        !overlap_joint && joint_distance >
        config.cross_direction_merge_maximum_endpoint_distance_px)
      {
        return;
      }
      const double support_ratio = connectorSupportRatio(
        response, first_joint, second_joint, config);
      if (
        support_ratio <
        config.cross_direction_merge_minimum_connector_support_ratio)
      {
        return;
      }
      for (int first_front = 0; first_front <= 1; ++first_front) {
        const bool first_toward_front = first_front != 0;
        const std::vector<RunEvidence> first_arm = retainedArm(
          first.evidence, first_index, first_toward_front, true);
        if (first_arm.size() < 2) {
          continue;
        }
        for (int second_front = 0; second_front <= 1; ++second_front) {
          const bool second_toward_front = second_front != 0;
          const std::vector<RunEvidence> second_arm = retainedArm(
            second.evidence, second_index, second_toward_front, false);
          if (second_arm.size() < 2) {
            continue;
          }
          const std::size_t first_tangent_index = first_arm.size() > 3 ?
            first_arm.size() - 4 : 0;
          const std::size_t second_tangent_index = second_arm.size() > 3 ? 3 :
            second_arm.size() - 1;
          const cv::Point2d incoming =
            evidenceCenterOriginal(first_arm.back()) -
            evidenceCenterOriginal(first_arm[first_tangent_index]);
          const cv::Point2d outgoing =
            evidenceCenterOriginal(second_arm[second_tangent_index]) -
            evidenceCenterOriginal(second_arm.front());
          const double incoming_length = cv::norm(incoming);
          const double outgoing_length = cv::norm(outgoing);
          if (incoming_length <= 1.0e-9 || outgoing_length <= 1.0e-9) {
            continue;
          }
          const double cosine = std::clamp(
            incoming.dot(outgoing) /
            (incoming_length * outgoing_length), -1.0, 1.0);
          const double turn_angle = std::acos(cosine) * 180.0 / CV_PI;
          if (
            turn_angle >
            config.cross_direction_merge_maximum_turn_angle_deg)
          {
            continue;
          }
          const double retained_arc_length =
            candidateArcLengthOriginal(first_arm) + joint_distance +
            candidateArcLengthOriginal(second_arm);
          const double quality = retained_arc_length + 5.0 * support_ratio -
            joint_distance - 0.10 * turn_angle;
          if (!best.valid || quality > best.quality) {
            best.valid = true;
            best.first_joint_index = first_index;
            best.second_joint_index = second_index;
            best.first_arm_toward_front = first_toward_front;
            best.second_arm_toward_front = second_toward_front;
            best.overlap_joint = overlap_joint;
            best.joint_point = joint_point;
            best.endpoint_distance_px = joint_distance;
            best.connector_support_ratio = support_ratio;
            best.turn_angle_deg = turn_angle;
            best.retained_arc_length_px = retained_arc_length;
            best.quality = quality;
          }
        }
      }
    };

  const std::size_t first_last = first.evidence.size() - 1;
  const std::size_t second_last = second.evidence.size() - 1;
  for (const std::size_t first_index : {std::size_t{0}, first_last}) {
    for (const std::size_t second_index : {std::size_t{0}, second_last}) {
      const cv::Point2d first_point = evidenceCenterOriginal(
        first.evidence[first_index]);
      const cv::Point2d second_point = evidenceCenterOriginal(
        second.evidence[second_index]);
      evaluate_joint(
        first_index, second_index, 0.5 * (first_point + second_point), false);
    }
  }
  for (std::size_t first_index = 0;
    first_index < first.evidence.size(); ++first_index)
  {
    for (std::size_t second_index = 0;
      second_index < second.evidence.size(); ++second_index)
    {
      cv::Point2d intersection;
      if (perpendicularRunsOverlap(
          first.evidence[first_index], second.evidence[second_index],
          &intersection))
      {
        evaluate_joint(first_index, second_index, intersection, true);
      }
    }
  }
  return best;
}

TrackCandidate mergeCandidates(
  const TrackCandidate & first,
  const TrackCandidate & second,
  const MergeProposal & proposal,
  const BevLaneSeedDetectorConfig & config)
{
  const std::vector<RunEvidence> first_evidence = retainedArm(
    first.evidence, proposal.first_joint_index,
    proposal.first_arm_toward_front, true);
  const std::vector<RunEvidence> second_evidence = retainedArm(
    second.evidence, proposal.second_joint_index,
    proposal.second_arm_toward_front, false);
  TrackCandidate merged;
  merged.valid = true;
  merged.has_row_support =
    first.has_row_support || second.has_row_support;
  merged.has_column_support =
    first.has_column_support || second.has_column_support;
  merged.evidence.reserve(first_evidence.size() + second_evidence.size());
  merged.evidence.insert(
    merged.evidence.end(), first_evidence.begin(), first_evidence.end());
  merged.evidence.insert(
    merged.evidence.end(), second_evidence.begin(), second_evidence.end());
  double contrast_sum = 0.0;
  double response_sum = 0.0;
  for (const RunEvidence & evidence : merged.evidence) {
    contrast_sum += evidence.bilateral_contrast;
    response_sum += evidence.run.mean_response;
  }
  merged.arc_length_px = candidateArcLengthOriginal(merged.evidence);
  merged.mean_contrast = contrast_sum / merged.evidence.size();
  merged.mean_response = response_sum / merged.evidence.size();
  merged.score = merged.arc_length_px +
    config.contrast_score_weight * merged.mean_contrast +
    0.05 * merged.mean_response;
  merged.seed_point = candidateSeedOriginal(merged);
  return merged;
}

int mergeTouchingCandidates(
  std::vector<TrackCandidate> * candidates,
  const cv::Mat & response,
  const BevLaneSeedDetectorConfig & config,
  std::vector<cv::Point2d> * merge_points)
{
  if (
    candidates == nullptr || !config.cross_direction_merge_enabled)
  {
    return 0;
  }
  int merge_count = 0;
  while (true) {
    MergeProposal best;
    double best_quality = -std::numeric_limits<double>::infinity();
    for (std::size_t first_index = 0;
      first_index < candidates->size(); ++first_index)
    {
      for (std::size_t second_index = first_index + 1;
        second_index < candidates->size(); ++second_index)
      {
        MergeProposal proposal = evaluateMergeProposal(
          (*candidates)[first_index], (*candidates)[second_index],
          response, config);
        if (!proposal.valid) {
          continue;
        }
        if (proposal.quality > best_quality) {
          best_quality = proposal.quality;
          best = proposal;
          best.first_index = first_index;
          best.second_index = second_index;
        }
      }
    }
    if (!best.valid) {
      break;
    }
    TrackCandidate merged = mergeCandidates(
      (*candidates)[best.first_index], (*candidates)[best.second_index],
      best, config);
    (*candidates)[best.first_index] = std::move(merged);
    candidates->erase(
      candidates->begin() + static_cast<std::ptrdiff_t>(best.second_index));
    if (merge_points != nullptr) {
      merge_points->push_back(best.joint_point);
    }
    ++merge_count;
  }
  return merge_count;
}

double temporalTrackDistance(
  const TrackCandidate & candidate,
  const BevLaneSeed & remembered)
{
  if (!candidate.valid || !remembered.valid) {
    return std::numeric_limits<double>::infinity();
  }
  std::vector<double> distances;
  distances.reserve(candidate.evidence.size());
  for (const RunEvidence & evidence : candidate.evidence) {
    const cv::Point2d point = evidenceCenterOriginal(evidence);
    double nearest = std::numeric_limits<double>::infinity();
    for (const cv::Point2d & previous : remembered.support_points) {
      nearest = std::min(nearest, cv::norm(point - previous));
    }
    distances.push_back(nearest);
  }
  if (!distances.empty()) {
    return median(std::move(distances));
  }
  return cv::norm(candidate.seed_point - remembered.image_point);
}

bool orientationIndependentPairDistance(
  const TrackCandidate & first,
  const TrackCandidate & second,
  double * distance,
  int * sample_count,
  int * inlier_count)
{
  if (
    distance == nullptr || sample_count == nullptr || inlier_count == nullptr ||
    first.evidence.empty() || second.evidence.empty())
  {
    return false;
  }
  std::vector<double> distances;
  distances.reserve(first.evidence.size() + second.evidence.size());
  const auto append_nearest_distances = [&distances](
      const TrackCandidate & source, const TrackCandidate & target) {
      for (const RunEvidence & source_evidence : source.evidence) {
        const cv::Point2d source_point = evidenceCenterOriginal(source_evidence);
        double nearest = std::numeric_limits<double>::infinity();
        for (const RunEvidence & target_evidence : target.evidence) {
          const cv::Point2d target_point = evidenceCenterOriginal(target_evidence);
          nearest = std::min(nearest, cv::norm(source_point - target_point));
        }
        if (std::isfinite(nearest)) {
          distances.push_back(nearest);
        }
      }
    };
  append_nearest_distances(first, second);
  append_nearest_distances(second, first);
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
  seed.image_point = candidateSeedOriginal(candidate);
  seed.arc_length_px = candidate.arc_length_px;
  seed.mean_bilateral_contrast = candidate.mean_contrast;
  seed.score = candidate.score;
  seed.support_points.reserve(candidate.evidence.size());
  for (const RunEvidence & evidence : candidate.evidence) {
    seed.support_points.push_back(evidenceCenterOriginal(evidence));
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
    if (evidence.transposed) {
      cv::line(
        *image,
        cv::Point(evidence.run.row, evidence.run.first_column),
        cv::Point(evidence.run.row, evidence.run.last_column),
        color, 1, cv::LINE_8);
    } else {
      cv::line(
        *image,
        cv::Point(evidence.run.first_column, evidence.run.row),
        cv::Point(evidence.run.last_column, evidence.run.row),
        color, 1, cv::LINE_8);
    }
    const cv::Point2d point = evidenceCenterOriginal(evidence);
    points.emplace_back(
      static_cast<int>(std::lround(point.x)),
      static_cast<int>(std::lround(point.y)));
  }
  if (points.size() >= 2) {
    cv::polylines(*image, points, false, color, 1, cv::LINE_AA);
  }
  const cv::Point2d seed_point = candidateSeedOriginal(candidate);
  const cv::Point seed(
    static_cast<int>(std::lround(seed_point.x)),
    static_cast<int>(std::lround(seed_point.y)));
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
    const cv::Point2d point = evidenceCenterOriginal(evidence);
    points.emplace_back(
      static_cast<int>(std::lround(point.x)),
      static_cast<int>(std::lround(point.y)));
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
  const bool create_preview)
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

  CandidateExtraction row_extraction = extractCandidates(
    gray, enhanced_top_hat,
    result.roi_top_row, result.roi_bottom_row,
    0, enhanced_top_hat.cols, config_, false);
  std::vector<TrackCandidate> candidates =
    std::move(row_extraction.candidates);
  std::vector<cv::Point2d> contrast_points =
    std::move(row_extraction.contrast_points);
  std::vector<cv::Point2d> relaxed_points =
    std::move(row_extraction.relaxed_points);
  std::vector<cv::Point2d> slope_break_points =
    std::move(row_extraction.slope_break_points);
  result.accepted_row_track_count = static_cast<int>(candidates.size());

  // Evaluate columns on every frame, using the exact same run, continuity,
  // arc-length, local-contrast and slope gates as the row direction.
  if (config_.column_tracking_enabled) {
    cv::Mat transposed_gray;
    cv::Mat transposed_response;
    cv::transpose(gray, transposed_gray);
    cv::transpose(enhanced_top_hat, transposed_response);
    CandidateExtraction column_extraction = extractCandidates(
      transposed_gray, transposed_response,
      0, transposed_response.rows,
      result.roi_top_row, result.roi_bottom_row,
      config_, true);
    result.accepted_column_track_count = static_cast<int>(
      column_extraction.candidates.size());
    candidates.insert(
      candidates.end(),
      std::make_move_iterator(column_extraction.candidates.begin()),
      std::make_move_iterator(column_extraction.candidates.end()));
    contrast_points.insert(
      contrast_points.end(),
      column_extraction.contrast_points.begin(),
      column_extraction.contrast_points.end());
    relaxed_points.insert(
      relaxed_points.end(),
      column_extraction.relaxed_points.begin(),
      column_extraction.relaxed_points.end());
    slope_break_points.insert(
      slope_break_points.end(),
      column_extraction.slope_break_points.begin(),
      column_extraction.slope_break_points.end());
    result.column_tracking_used = true;
  }
  std::vector<cv::Point2d> merge_points;
  result.merged_track_count = mergeTouchingCandidates(
    &candidates, enhanced_top_hat, config_, &merge_points);
  result.accepted_track_count = static_cast<int>(candidates.size());
  result.strict_evidence_count = static_cast<int>(contrast_points.size()) -
    static_cast<int>(relaxed_points.size());
  result.relaxed_evidence_count = static_cast<int>(relaxed_points.size());
  result.slope_break_count = static_cast<int>(slope_break_points.size());

  TrackCandidate selected_left;
  TrackCandidate selected_right;
  const double center_column =
    0.5 * static_cast<double>(enhanced_top_hat.cols - 1);
  double best_pair_score = -std::numeric_limits<double>::infinity();
  TrackCandidate current_pair_left;
  TrackCandidate current_pair_right;
  for (std::size_t first_index = 0;
    first_index < candidates.size(); ++first_index)
  {
    for (std::size_t second_index = first_index + 1;
      second_index < candidates.size(); ++second_index)
    {
      const TrackCandidate & first = candidates[first_index];
      const TrackCandidate & second = candidates[second_index];
      const TrackCandidate & left =
        first.seed_point.x < second.seed_point.x ? first : second;
      const TrackCandidate & right =
        first.seed_point.x < second.seed_point.x ? second : first;
      // The first lock must be anchored by one seed on each side of the BEV
      // center. Once roles are known, a valid-width pair may move entirely to
      // one screen half without changing its physical left/right ordering.
      const bool require_centered_pair =
        !config_.temporal_side_lock_enabled || !side_lock_initialized_;
      const bool pair_has_column_track =
        left.has_column_support || right.has_column_support;
      if (
        require_centered_pair && !pair_has_column_track &&
        (left.seed_point.x >= center_column ||
        right.seed_point.x < center_column))
      {
        continue;
      }
      double distance = 0.0;
      int samples = 0;
      int inliers = 0;
      if (!orientationIndependentPairDistance(
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
        current_pair_left = left;
        current_pair_right = right;
        result.pair_valid = true;
        result.pair_distance_px = distance;
        result.pair_distance_samples = samples;
        result.pair_distance_inliers = inliers;
      }
    }
  }

  if (!config_.temporal_side_lock_enabled) {
    if (result.pair_valid) {
      selected_left = current_pair_left;
      selected_right = current_pair_right;
    } else if (!candidates.empty()) {
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
  } else if (!side_lock_initialized_) {
    // A single track cannot establish its physical side. Initialise the lock
    // only from a geometrically valid pair, ordered by image column.
    if (result.pair_valid) {
      selected_left = current_pair_left;
      selected_right = current_pair_right;
      side_lock_initialized_ = true;
    }
  } else if (result.pair_valid) {
    selected_left = current_pair_left;
    selected_right = current_pair_right;
  } else {
    const double maximum_distance =
      config_.temporal_side_reacquire_maximum_distance_px;
    std::size_t best_left_index = candidates.size();
    std::size_t best_right_index = candidates.size();
    double best_assignment_cost = std::numeric_limits<double>::infinity();

    // Prefer a two-sided temporal assignment when two distinct candidates
    // match the remembered physical left/right curves.
    for (std::size_t left_index = 0;
      left_index < candidates.size(); ++left_index)
    {
      const double left_distance = temporalTrackDistance(
        candidates[left_index], remembered_left_);
      if (left_distance > maximum_distance) {
        continue;
      }
      for (std::size_t right_index = 0;
        right_index < candidates.size(); ++right_index)
      {
        if (left_index == right_index) {
          continue;
        }
        const double right_distance = temporalTrackDistance(
          candidates[right_index], remembered_right_);
        if (right_distance > maximum_distance) {
          continue;
        }
        const double cost = left_distance + right_distance - 0.001 * (
          candidates[left_index].score + candidates[right_index].score);
        if (cost < best_assignment_cost) {
          best_assignment_cost = cost;
          best_left_index = left_index;
          best_right_index = right_index;
        }
      }
    }

    if (
      best_left_index != candidates.size() &&
      best_right_index != candidates.size())
    {
      selected_left = candidates[best_left_index];
      selected_right = candidates[best_right_index];
      result.temporal_labeling_used = true;
    } else {
      // With one visible lane, keep the role that was visible in the previous
      // frame. If both or neither were visible, use the closer remembered
      // curve. This also preserves the role after a short detection dropout.
      const bool prefer_left =
        (previous_left_visible_ && !previous_right_visible_) ||
        (!previous_left_visible_ && !previous_right_visible_ &&
        remembered_single_side_ < 0);
      const bool prefer_right =
        (previous_right_visible_ && !previous_left_visible_) ||
        (!previous_left_visible_ && !previous_right_visible_ &&
        remembered_single_side_ > 0);
      std::size_t best_index = candidates.size();
      bool assign_left = false;
      double best_cost = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < candidates.size(); ++index) {
        const double left_distance = temporalTrackDistance(
          candidates[index], remembered_left_);
        const double right_distance = temporalTrackDistance(
          candidates[index], remembered_right_);
        if (
          !prefer_right && left_distance <= maximum_distance &&
          left_distance < best_cost)
        {
          best_cost = left_distance;
          best_index = index;
          assign_left = true;
        }
        if (
          !prefer_left && right_distance <= maximum_distance &&
          right_distance < best_cost)
        {
          best_cost = right_distance;
          best_index = index;
          assign_left = false;
        }
      }
      if (best_index != candidates.size()) {
        if (assign_left) {
          selected_left = candidates[best_index];
        } else {
          selected_right = candidates[best_index];
        }
        result.temporal_labeling_used = true;
      }
    }
  }

  result.left = toPublicSeed(selected_left);
  result.right = toPublicSeed(selected_right);
  if (config_.temporal_side_lock_enabled && side_lock_initialized_) {
    previous_left_visible_ = result.left.valid;
    previous_right_visible_ = result.right.valid;
    if (result.left.valid && !result.right.valid) {
      remembered_single_side_ = -1;
    } else if (result.right.valid && !result.left.valid) {
      remembered_single_side_ = 1;
    } else if (result.left.valid && result.right.valid) {
      remembered_single_side_ = 0;
    }
    if (result.left.valid) {
      remembered_left_ = result.left;
    }
    if (result.right.valid) {
      remembered_right_ = result.right;
    }
    if (result.left.valid || result.right.valid) {
      both_sides_missing_frames_ = 0;
    } else {
      ++both_sides_missing_frames_;
      if (
        both_sides_missing_frames_ >=
        config_.temporal_side_lock_reset_frames)
      {
        side_lock_initialized_ = false;
        previous_left_visible_ = false;
        previous_right_visible_ = false;
        remembered_single_side_ = 0;
        both_sides_missing_frames_ = 0;
        remembered_left_ = BevLaneSeed{};
        remembered_right_ = BevLaneSeed{};
        result.side_lock_reset = true;
      }
    }
  }
  result.side_lock_initialized = side_lock_initialized_;
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
    for (const cv::Point2d & point : merge_points) {
      cv::circle(
        result.preview,
        cv::Point(
          static_cast<int>(std::lround(point.x)),
          static_cast<int>(std::lround(point.y))),
        3, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
    if (config_.temporal_side_lock_enabled) {
      std::string lock_text = !result.side_lock_initialized ?
        "WAIT L+R" : result.temporal_labeling_used ?
        "SIDE LOCK:T" : "SIDE LOCK";
      if (result.column_tracking_used) {
        lock_text += ":RC";
      }
      cv::putText(
        result.preview, lock_text, cv::Point(2, 12),
        cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(0, 0, 0),
        2, cv::LINE_AA);
      cv::putText(
        result.preview, lock_text, cv::Point(2, 12),
        cv::FONT_HERSHEY_SIMPLEX, 0.30, cv::Scalar(255, 255, 255),
        1, cv::LINE_AA);
    }
  }
  return result;
}

}  // namespace bev_processor
