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
      || c.hold_sec < 0.0 || c.hold_sec > 0.5
      || !std::isfinite(c.max_frame_gap_sec) || c.max_frame_gap_sec <= 0.0
      || c.max_frame_gap_sec > 1.0 || c.max_frame_gap_sec < c.hold_sec) {
    reason = "stabilization requires 1 <= hits <= window <= 16, hold [0,0.5], frame_gap (0,1] >= hold";
    return false;
  }
  reason.clear();
  return true;
}

void GridStabilizer::clear()
{
  states_.clear();
  have_time_ = false;
  previous_time_sec_ = 0.0;
  next_expiry_sec_ = std::numeric_limits<double>::infinity();
}

DetectionResult GridStabilizer::render(double time_sec, const StabilizationConfig & c)
{
  auto result = emptyGrid(grid_, cluster_);
  result.roi = roi_;
  next_expiry_sec_ = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < states_.size(); ++i) {
    auto & state = states_[i];
    if (time_sec - state.last_seen_sec > c.hold_sec) {
      state.confirmed = false;
      state.support_points = 0;
      // An idle clock expires occupancy, not frame-counted evidence. Keeping
      // that evidence also lets confirm_hits > 1 work when hold_sec is zero.
      continue;
    }
    if (!state.confirmed) { continue; }
    result.cells[i] = {state.support_points, std::max(0.0, time_sec - state.last_seen_sec)};
    next_expiry_sec_ = std::min(next_expiry_sec_, state.last_seen_sec + c.hold_sec);
  }
  buildGridClusters(result);
  return result;
}

DetectionResult GridStabilizer::snapshot(double time_sec, const StabilizationConfig & c)
{
  // No history shift or extra hit from the idle clock.
  return render(time_sec, c);
}

DetectionResult GridStabilizer::update(const DetectionResult & raw, double time_sec,
  const StabilizationConfig & c)
{
  std::string reason;
  if (!std::isfinite(time_sec) || !validateStabilizationConfig(c, reason)) {
    throw std::invalid_argument("invalid stabilization input: " + reason);
  }
  if (!c.enabled) {
    clear();
    auto result = raw;
    buildGridClusters(result);
    return result;
  }
  const bool layout_changed = grid_.resolution_m != raw.grid.resolution_m
    || grid_.x_min_m != raw.grid.x_min_m || grid_.x_max_m != raw.grid.x_max_m
    || grid_.y_min_m != raw.grid.y_min_m || grid_.y_max_m != raw.grid.y_max_m;
  if (layout_changed || states_.size() != raw.cells.size()
      || (have_time_ && (time_sec <= previous_time_sec_
        || time_sec - previous_time_sec_ > c.max_frame_gap_sec))) { clear(); }
  grid_ = raw.grid;
  cluster_ = raw.cluster;
  roi_ = raw.roi;
  if (states_.empty()) { states_.resize(raw.cells.size()); }
  have_time_ = true;
  previous_time_sec_ = time_sec;
  const auto history_mask = (std::uint32_t{1} << c.window_frames) - 1U;
  for (std::size_t i = 0; i < states_.size(); ++i) {
    auto & state = states_[i];
    // An expired cell needs a fresh hit plus enough recent frame evidence.
    if (time_sec - state.last_seen_sec > c.hold_sec) { state.confirmed = false; }
    state.history = (state.history << 1U) & history_mask;
    if (raw.cells[i].support_points) {
      state.history |= 1U;
      state.support_points = raw.cells[i].support_points;
      state.last_seen_sec = time_sec;
      int hits = 0;
      for (auto bits = state.history; bits; bits >>= 1U) { hits += static_cast<int>(bits & 1U); }
      if (hits >= c.confirm_hits) { state.confirmed = true; }
    }
  }
  return render(time_sec, c);
}
} // namespace depth_lidar
