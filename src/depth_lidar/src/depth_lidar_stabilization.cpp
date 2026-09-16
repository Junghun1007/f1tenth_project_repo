#include "depth_lidar/depth_lidar_stabilization.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace depth_lidar
{
bool validateStabilizationConfig(const StabilizationConfig & c, std::string & reason)
{
  if (c.window_frames < 1 || c.window_frames > 16 || c.confirm_hits < 1
      || c.confirm_hits > c.window_frames || !std::isfinite(c.hold_sec)
      || c.hold_sec < 0.0 || c.hold_sec > 0.5 || !std::isfinite(c.match_distance_m)
      || c.match_distance_m <= 0.0 || c.match_distance_m > 1.0
      || !std::isfinite(c.max_frame_gap_sec) || c.max_frame_gap_sec <= 0.0
      || c.max_frame_gap_sec > 1.0 || c.max_frame_gap_sec < c.hold_sec) {
    reason = "stabilization requires 1 <= hits <= window <= 16, hold [0,0.5], "
      "match_distance (0,1], frame_gap (0,1] >= hold";
    return false;
  }
  reason.clear();
  return true;
}

void ClusterStabilizer::clear()
{
  tracks_.clear();
  have_time_ = false;
  previous_time_sec_ = 0.0;
}

DetectionResult ClusterStabilizer::snapshot(double time_sec, const StabilizationConfig & c) const
{
  DetectionResult result;
  // Expire display/output even if the device stops delivering frames. This
  // read-only clock update must not count as a new observation or missed frame.
  for (const auto & track : tracks_) {
    const double age = time_sec - track.last_seen_sec;
    if (!track.confirmed || age < 0.0 || age > c.hold_sec) { continue; }
    auto circle = track.circle;
    circle.observation_age_sec = age;
    result.obstacles.push_back(circle);
  }
  return result;
}

DetectionResult ClusterStabilizer::update(const DetectionResult & raw, double time_sec,
  const StabilizationConfig & c)
{
  std::string reason;
  if (!std::isfinite(time_sec) || !validateStabilizationConfig(c, reason)) {
    throw std::invalid_argument("invalid stabilization input: " + reason);
  }
  if (!c.enabled) { clear(); return raw; }
  if (have_time_ && time_sec <= previous_time_sec_) {
    // Duplicate/out-of-order images cannot provide a second confirmation.
    clear();
    DetectionResult empty;
    empty.roi = raw.roi;
    return empty;
  }
  if (have_time_ && time_sec - previous_time_sec_ > c.max_frame_gap_sec) { clear(); }
  previous_time_sec_ = time_sec;
  have_time_ = true;
  const auto history_mask = (std::uint32_t{1} << c.window_frames) - 1U;
  for (auto & track : tracks_) { track.history = (track.history << 1U) & history_mask; }

  // Each raw cluster owns a contiguous run of points from detectForeground.
  std::vector<std::size_t> offsets(raw.obstacles.size() + 1, 0U);
  for (std::size_t i = 0; i < raw.obstacles.size(); ++i) {
    if (raw.obstacles[i].support_points > raw.points.size() - offsets[i]) {
      throw std::invalid_argument("cluster support does not match raw points");
    }
    offsets[i + 1] = offsets[i] + raw.obstacles[i].support_points;
  }
  if (offsets.back() != raw.points.size()) {
    throw std::invalid_argument("unassigned raw cluster points");
  }
  struct Pair { double squared_distance; std::size_t track; std::size_t detection; };
  std::vector<Pair> pairs;
  const double max_distance_squared = c.match_distance_m * c.match_distance_m;
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    for (std::size_t d = 0; d < raw.obstacles.size(); ++d) {
      const double dx = tracks_[t].circle.forward_m - raw.obstacles[d].forward_m;
      const double dy = tracks_[t].circle.left_m - raw.obstacles[d].left_m;
      const double distance = dx * dx + dy * dy;
      if (distance <= max_distance_squared) { pairs.push_back({distance, t, d}); }
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) {
    if (a.squared_distance != b.squared_distance) { return a.squared_distance < b.squared_distance; }
    if (a.track != b.track) { return a.track < b.track; }
    return a.detection < b.detection;
  });
  const auto no_detection = raw.obstacles.size();
  std::vector<std::size_t> matched(tracks_.size(), no_detection);
  std::vector<bool> used(raw.obstacles.size(), false);
  for (const auto & pair : pairs) {
    if (matched[pair.track] != no_detection || used[pair.detection]) { continue; }
    matched[pair.track] = pair.detection;
    used[pair.detection] = true;
    auto & track = tracks_[pair.track];
    track.circle = raw.obstacles[pair.detection];
    track.history |= 1U;
    track.last_seen_sec = time_sec;
  }
  for (std::size_t d = 0; d < raw.obstacles.size(); ++d) {
    if (used[d]) { continue; }
    tracks_.push_back({raw.obstacles[d], 1U, time_sec, false});
    matched.push_back(d);
  }

  DetectionResult result;
  result.roi = raw.roi;
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    auto & track = tracks_[t];
    int hits = 0;
    for (auto bits = track.history; bits; bits >>= 1U) { hits += static_cast<int>(bits & 1U); }
    if (matched[t] != no_detection && hits >= c.confirm_hits) { track.confirmed = true; }
    const double age = time_sec - track.last_seen_sec;
    if (!track.confirmed || age > c.hold_sec) { continue; }
    auto circle = track.circle;
    circle.observation_age_sec = age;
    result.obstacles.push_back(circle);
    if (matched[t] != no_detection) {
      const auto d = matched[t];
      result.points.insert(result.points.end(), raw.points.begin() + offsets[d], raw.points.begin() + offsets[d + 1]);
    }
  }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & track) {
    return time_sec - track.last_seen_sec > c.hold_sec || (!track.confirmed && track.history == 0U);
  }), tracks_.end());
  return result;
}
} // namespace depth_lidar
