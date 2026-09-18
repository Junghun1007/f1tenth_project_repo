#pragma once

#include "point_cloud/obstacle_clusters.hpp"

#include <map>
#include <tuple>

namespace point_cloud
{
struct TrackingOptions
{
  bool enabled{true};
  double match_distance_m{0.15};
  double hold_sec{0.20};
  int min_hits{2};
};

inline void validateTracking(const TrackingOptions & o)
{
  if (!std::isfinite(o.match_distance_m) || o.match_distance_m <= 0 ||
    o.match_distance_m > 0.5 || !std::isfinite(o.hold_sec) || o.hold_sec < 0.02 ||
    o.hold_sec > 0.5 || o.min_hits < 1 || o.min_hits > 10)
  {
    throw std::invalid_argument("tracking: match_distance_m=(0,0.5], hold_sec=0.02..0.5, min_hits=1..10");
  }
}

struct TrackedClusters
{
  ClusterResult clusters;  // IDs are persistent here, unlike per-frame detections.
  std::vector<float> observation_age_sec;  // One age per original point.
  std::vector<bool> held;  // True: last seen in an earlier depth frame.
  std::size_t live_tracks{0}, held_tracks{0};
};

// Short visual dropout hold in vehicle coordinates; no ego-motion compensation
// or predicted points. Each matched observation REPLACES the track's old points.
class ClusterTracker
{
  struct Detection
  {
    std::vector<float> xyz;
    std::array<double, 3> center{};
  };
  struct Track
  {
    Detection detection;
    std::uint32_t id;
    double last_seen;
    int hits;
  };
  std::vector<Track> tracks_;
  TrackingOptions options_;
  double latest_stamp_{-std::numeric_limits<double>::infinity()};
  std::uint32_t next_id_{1};
  static constexpr std::size_t max_tracks = 128;

  std::uint32_t allocateId()
  {
    for (;;) {
      const auto id = next_id_++;
      if (id != 0 && std::none_of(tracks_.begin(), tracks_.end(),
        [id](const Track & t) {return t.id == id;})) {return id;}
    }
  }

public:
  void clear()
  {
    tracks_.clear();
    latest_stamp_ = -std::numeric_limits<double>::infinity();
  }

  // Call even while no new frames arrive. Returns true if output may have changed.
  bool expire(double now)
  {
    const auto count = tracks_.size();
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track & t) {
      return now - t.last_seen >= options_.hold_sec;
    }), tracks_.end());
    return tracks_.size() != count;
  }

  void update(const ClusterResult & input, double stamp, const TrackingOptions & o)
  {
    validateTracking(o);
    if (!std::isfinite(stamp)) {throw std::invalid_argument("tracking: nonfinite timestamp");}
    if (std::tie(o.enabled, o.match_distance_m, o.hold_sec, o.min_hits) !=
      std::tie(options_.enabled, options_.match_distance_m, options_.hold_sec, options_.min_hits) ||
      stamp < latest_stamp_) {clear();}
    options_ = o;
    if (!o.enabled) {clear(); return;}
    if (stamp == latest_stamp_) {return;}  // Replayed frames cannot confirm a track.
    expire(stamp);
    latest_stamp_ = stamp;

    std::map<std::uint32_t, Detection> groups;
    for (std::size_t i = 0; i < input.ids.size(); ++i) {
      const auto * p = input.points.xyz.data() + i * 3;
      auto & d = groups[input.ids[i]];
      d.xyz.insert(d.xyz.end(), p, p + 3);
      for (int axis = 0; axis < 3; ++axis) {d.center[axis] += p[axis];}
    }
    std::vector<Detection> detections;
    for (auto & entry : groups) {
      auto & d = entry.second;
      for (auto & value : d.center) {value /= d.xyz.size() / 3;}
      detections.push_back(std::move(d));
    }
    // Bound association and retained history in pathological noisy scenes.
    std::stable_sort(detections.begin(), detections.end(), [](const Detection & a, const Detection & b) {
      return a.xyz.size() > b.xyz.size();
    });
    if (detections.size() > max_tracks) {detections.resize(max_tracks);}
    struct Edge {double squared_distance; std::size_t track, detection;};
    std::vector<Edge> edges;
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
      for (std::size_t d = 0; d < detections.size(); ++d) {
        double distance = 0;
        for (int axis = 0; axis < 3; ++axis) {
          const double delta = tracks_[t].detection.center[axis] - detections[d].center[axis];
          distance += delta * delta;
        }
        if (distance <= o.match_distance_m * o.match_distance_m) {edges.push_back({distance, t, d});}
      }
    }
    std::sort(edges.begin(), edges.end(), [](const Edge & a, const Edge & b) {
      return std::tie(a.squared_distance, a.track, a.detection) <
             std::tie(b.squared_distance, b.track, b.detection);
    });
    std::vector<bool> used_tracks(tracks_.size()), used_detections(detections.size());
    for (const auto & edge : edges) {
      if (used_tracks[edge.track] || used_detections[edge.detection]) {continue;}
      auto & track = tracks_[edge.track];
      track.detection = std::move(detections[edge.detection]);
      track.last_seen = stamp;
      track.hits = std::min(track.hits + 1, o.min_hits);
      used_tracks[edge.track] = used_detections[edge.detection] = true;
    }
    for (std::size_t d = 0; d < detections.size() && tracks_.size() < max_tracks; ++d) {
      if (!used_detections[d]) {
        const auto id = allocateId();
        tracks_.push_back({std::move(detections[d]), id, stamp, 1});
      }
    }
  }

  TrackedClusters snapshot(double now)
  {
    expire(now);
    TrackedClusters out;
    for (const auto & track : tracks_) {
      if (track.hits < options_.min_hits) {continue;}
      const auto count = track.detection.xyz.size() / 3;
      const bool held = track.last_seen < latest_stamp_;
      out.live_tracks += !held;
      out.held_tracks += held;
      out.clusters.points.xyz.insert(out.clusters.points.xyz.end(),
        track.detection.xyz.begin(), track.detection.xyz.end());
      out.clusters.ids.insert(out.clusters.ids.end(), count, track.id);
      out.observation_age_sec.insert(out.observation_age_sec.end(), count,
        static_cast<float>(std::max(0.0, now - track.last_seen)));
      out.held.insert(out.held.end(), count, held);
    }
    out.clusters.accepted = out.live_tracks + out.held_tracks;
    out.clusters.points.width = out.clusters.ids.size();
    out.clusters.points.height = 1;
    out.clusters.points.valid_points = out.clusters.ids.size();
    return out;
  }
};
}  // namespace point_cloud
