#include "point_cloud/cluster_tracker.hpp"

#include <iostream>
#include <string>

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}
point_cloud::ClusterResult detections(std::initializer_list<float> centers)
{
  point_cloud::ClusterResult out;
  for (const auto x : centers) {
    ++out.accepted;
    for (int i = 0; i < 8; ++i) {
      out.points.xyz.insert(out.points.xyz.end(), {x + 0.002f * i, 0.0f, 0.1f});
      out.ids.push_back(out.accepted);
    }
  }
  out.points.width = out.points.valid_points = out.ids.size();
  out.points.height = 1;
  return out;
}
std::uint32_t idNear(const point_cloud::TrackedClusters & out, float x)
{
  for (std::size_t i = 0; i < out.clusters.ids.size(); ++i) {
    if (std::abs(out.clusters.points.xyz[3*i] - x) < 0.02f) {return out.clusters.ids[i];}
  }
  return 0;
}
}

int main()
{
  using namespace point_cloud;
  ClusterTracker tracker;
  TrackingOptions o;
  tracker.update(detections({1, 2}), 1.0, o);
  check(tracker.snapshot(1.0).clusters.accepted == 0, "One-frame noise admitted");
  tracker.update(detections({1, 2}), 1.0, o);
  check(tracker.snapshot(1.0).clusters.accepted == 0, "Duplicate frame confirmed track");
  tracker.update(detections({2.01f, 1.01f, 3}), 1.02, o);
  auto out = tracker.snapshot(1.02);
  check(out.live_tracks == 2 && out.held_tracks == 0, "Confirmation or transient rejection failed");
  const auto first = idNear(out, 1.01f), second = idNear(out, 2.01f);
  check(first != 0 && second != 0 && first != second, "Distinct IDs missing");
  check(out.clusters.points.valid_points == 16, "Track points accumulated");
  tracker.update(detections({2.02f}), 1.04, o);
  out = tracker.snapshot(1.04);
  check(out.live_tracks == 1 && out.held_tracks == 1, "Short individual dropout blinked");
  check(idNear(out, 1.01f) == first && idNear(out, 2.02f) == second, "Frame label changed identity");
  for (std::size_t i = 0; i < out.clusters.ids.size(); ++i) {
    check(out.held[i] == (out.clusters.ids[i] == first), "Held point provenance wrong");
    check(std::abs(out.observation_age_sec[i] - (out.held[i] ? 0.02f : 0.0f)) < 1e-6,
      "Observation age wrong");
  }
  tracker.update(detections({}), 1.08, o);
  out = tracker.snapshot(1.08);
  check(out.held_tracks == 2 && out.live_tracks == 0, "Whole-frame dropout blinked");
  tracker.update(detections({1.04f, 2.04f}), 1.12, o);
  out = tracker.snapshot(1.12);
  check(out.live_tracks == 2 && idNear(out, 1.04f) == first && idNear(out, 2.04f) == second,
    "Reappearance reset confirmed identity");
  check(out.clusters.points.valid_points == 16, "Old points retained after new observation");
  tracker.update(detections({}), 1.30, o);
  check(tracker.snapshot(1.30).held_tracks == 2, "Hold too short");
  check(tracker.expire(1.321), "No-frame wall clock did not expire tracks");
  check(tracker.snapshot(1.321).clusters.ids.empty(), "Dropouts extended last-seen lifetime");
  tracker.update(detections({1}), 1.34, o);
  check(tracker.snapshot(1.34).clusters.accepted == 0, "Expired track stayed confirmed");
  tracker.clear();
  check(tracker.snapshot(1.34).clusters.ids.empty(), "Clear left stale points");

  // Sporadic detections within the hold window may confirm; empty frames are not hits.
  tracker.update(detections({1}), 2.0, o);
  tracker.update(detections({}), 2.04, o);
  check(tracker.snapshot(2.04).clusters.accepted == 0, "Empty frame confirmed tentative track");
  tracker.update(detections({1.02f}), 2.10, o);
  check(tracker.snapshot(2.10).live_tracks == 1, "Intermittent real observation never confirmed");
  o.match_distance_m = 0.1;
  tracker.update(detections({1.02f}), 2.12, o);
  check(tracker.snapshot(2.12).clusters.accepted == 0, "Changed settings inherited history");
  tracker.update(detections({1.02f}), 1.0, o);
  check(tracker.snapshot(1.0).clusters.accepted == 0, "Clock reversal inherited history");
  o.enabled = false;
  tracker.update(detections({1.02f}), 1.02, o);
  check(tracker.snapshot(1.02).clusters.ids.empty(), "Disable retained tracks");
  o.enabled = true;
  tracker.update(detections({1.02f}), 1.04, o);
  check(tracker.snapshot(1.04).clusters.accepted == 0, "Re-enable inherited hits");

  // Nearby detections must never share an ID, including after one detection splits.
  o.min_hits = 1;
  tracker.update(detections({1}), 3.0, o);
  tracker.update(detections({1.01f, 1.08f}), 3.02, o);
  out = tracker.snapshot(3.02);
  check(out.live_tracks == 2 && idNear(out, 1.01f) != idNear(out, 1.08f), "Association is not one-to-one");
  tracker.clear();
  tracker.update(detections({1}), 4.0, o);
  const auto near_id = idNear(tracker.snapshot(4.0), 1);
  tracker.update(detections({2}), 4.02, o);
  out = tracker.snapshot(4.02);
  check(out.live_tracks == 1 && out.held_tracks == 1 && idNear(out, 2) != near_id,
    "Out-of-gate detection reused ID");
  tracker.clear();
  tracker.update(detections({1}), 5.0, o);
  auto high = detections({1});
  for (std::size_t i = 2; i < high.points.xyz.size(); i += 3) {high.points.xyz[i] += 0.5f;}
  tracker.update(high, 5.02, o);
  check(tracker.snapshot(5.02).clusters.accepted == 2, "Association ignored height separation");

  // Bounded association under many disconnected candidates.
  tracker.clear();
  ClusterResult noisy;
  for (std::uint32_t i = 1; i <= 160; ++i) {
    noisy.points.xyz.insert(noisy.points.xyz.end(), {static_cast<float>(i), 0, 0.1f});
    noisy.ids.push_back(i);
  }
  noisy.points.width = noisy.points.valid_points = noisy.ids.size();
  noisy.points.height = 1;
  noisy.accepted = 160;
  tracker.update(noisy, 6.0, o);
  check(tracker.snapshot(6.0).live_tracks == 128, "Track budget exceeded");

  for (const double hold : {0.0, 0.6, std::numeric_limits<double>::quiet_NaN()}) {
    auto invalid = o;
    invalid.hold_sec = hold;
    bool rejected = false;
    try {validateTracking(invalid);} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "Invalid hold accepted");
  }
  std::cout << "Cluster tracking tests passed\n";
}
