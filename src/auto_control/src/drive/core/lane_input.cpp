#include "auto_control/drive/core/lane_input.hpp"
#include "auto_control/drive/core/cone_memory.hpp"
#include "auto_control/drive/core/path_utils.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace auto_control::drive {

namespace {
constexpr double kMaximumSegmentGap = 0.25;
constexpr double kMinimumLaneWidth = 0.50;
constexpr double kWidthStationTolerance = 0.08;
constexpr double kBoundaryStationSpacing = 0.04;
constexpr double kSameBoundarySpread = 0.08;
constexpr double kReturnProgress = 0.02;

const std::vector<Vec2> & reference_center(const LaneObservation & lane) {
  return lane.center.size() >= 4 ? lane.center : lane.path;
}

std::vector<Vec2> boundary_stations(const std::vector<Vec2> & points, const FrenetFrame & center) {
  std::vector<Vec2> stations;
  stations.reserve(points.size());
  for (const auto & point : points) {
    if (std::isfinite(point.x) && std::isfinite(point.y)) stations.push_back(project_sl(point, center));
  }
  std::sort(stations.begin(), stations.end(), [](const Vec2 & a, const Vec2 & b) { return a.x < b.x; });
  return stations;
}

// Reject competing lines on one side and measure left-to-right width at the
// same longitudinal station, even when the vehicle is angled to the lane.
bool boundaries_consistent(const LaneObservation & lane, bool require_pair = false) {
  const auto & points = reference_center(lane);
  if (points.size() < 4) return false;
  const auto center = frenet_frame(points);
  const auto left = boundary_stations(lane.left, center);
  const auto right = boundary_stations(lane.right, center);
  for (const auto * side : {&left, &right}) {
    for (std::size_t i = 0; i < side->size(); ++i) {
      for (std::size_t j = i + 1; j < side->size() && (*side)[j].x - (*side)[i].x <= kBoundaryStationSpacing; ++j) {
        if (std::abs((*side)[i].y - (*side)[j].y) > kSameBoundarySpread) return false;
      }
    }
  }
  bool paired = false;
  for (const auto & l : left) {
    for (auto r = std::lower_bound(right.begin(), right.end(), l.x - kWidthStationTolerance,
           [](const Vec2 & p, double s) { return p.x < s; });
         r != right.end() && r->x <= l.x + kWidthStationTolerance; ++r) {
      if (l.y - r->y + 1e-6 < kMinimumLaneWidth) return false;
      paired = true;
    }
  }
  return paired || (!require_pair && (left.empty() || right.empty()));
}

std::vector<Vec2> innermost_boundary(const LaneObservation & lane, const Pose2 & pose,
                                     const FrenetFrame & road, const Pose2 & road_pose, int side,
                                     bool assign_single = false) {
  std::map<double, std::pair<double, Vec2>> nearest;
  // Select at most one candidate per station on the requested side. Farther
  // outward markings lose to the line closest to the lane interior.
  for (const auto * points : {&lane.left, &lane.right}) {
    for (const auto & point : *points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
      const auto sl = project_sl(to_local(to_world(point, pose), road_pose), road);
      const double lateral = side * sl.y;
      if (!assign_single && lateral <= 0.0) continue;
      const double station = std::round(sl.x / kBoundaryStationSpacing);
      auto [it, inserted] = nearest.try_emplace(station, lateral, point);
      if (!inserted && lateral < it->second.first) it->second = {lateral, point};
    }
  }
  std::vector<Vec2> ordered;
  ordered.reserve(nearest.size());
  std::size_t start = 0, best_start = 0, best_end = 0;
  double length = 0.0, best_length = 0.0;
  for (const auto & entry : nearest) {
    const auto & point = entry.second.second;
    if (!ordered.empty()) {
      const double gap = std::hypot(point.x - ordered.back().x, point.y - ordered.back().y);
      if (gap > kMaximumSegmentGap) { start = ordered.size(); length = 0.0; }
      else length += gap;
    }
    ordered.push_back(point);
    if (length > best_length) {
      best_start = start;
      best_end = ordered.size();
      best_length = length;
    }
  }
  if (best_end - best_start < 4) return {};
  return {ordered.begin() + best_start, ordered.begin() + best_end};
}

void center_from_boundary(LaneObservation & lane, int side, double half_width) {
  const auto & boundary = side > 0 ? lane.left : lane.right;
  const auto line = frenet_frame(boundary);
  lane.path.clear();
  for (std::size_t i = 0; i < boundary.size(); ++i) {
    lane.path.push_back({boundary[i].x - side * half_width * line.norm[i].x,
                         boundary[i].y - side * half_width * line.norm[i].y});
  }
  lane.center = lane.path;
  lane.valid = lane.path.size() >= 4;
}
}  // namespace

bool LaneDepartureCheck::accept(LaneObservation & lane, const std::optional<Pose2> & pose,
                                 bool avoiding, double lane_half_width, double vehicle_half_width, double wheelbase) {
  if (!pose || !std::isfinite(pose->x) || !std::isfinite(pose->y) || !std::isfinite(pose->yaw) ||
      !std::isfinite(lane.capture_time) || lane.capture_time <= last_capture_) return false;
  last_capture_ = lane.capture_time;
  bool restored_pair = false;
  if (road_ && (avoiding || departure_side_ != 0)) {
    const auto road = frenet_frame(road_->center);
    const auto front = project_sl(to_local({pose->x, pose->y}, road_->pose), road);
    const auto rear = project_sl(to_local(to_world({-wheelbase, 0.0}, *pose), road_->pose), road);
    const double outer_axle = std::abs(front.y) >= std::abs(rear.y) ? front.y : rear.y;
    const bool inside = std::abs(outer_axle) + vehicle_half_width <= lane_half_width;
    if (departure_side_ == 0 && !inside) departure_side_ = outer_axle > 0.0 ? 1 : -1;
    if (departure_side_ != 0) {
      const double outward = departure_side_ * front.y;
      peak_outward_m_ = std::max(peak_outward_m_, outward);
      returning_ = returning_ || inside || peak_outward_m_ - outward >= kReturnProgress;
      const bool left_first = departure_side_ > 0;
      const bool assign_single = boundary_lost_ && returning_ && (lane.left.empty() != lane.right.empty());
      const auto recovered = recovered_road_ ? std::optional<FrenetFrame>{frenet_frame(recovered_road_->center)} : std::nullopt;
      const auto & boundary_frame = recovered ? *recovered : road;
      const auto & boundary_pose = recovered_road_ ? recovered_road_->pose : road_->pose;
      auto first = innermost_boundary(lane, *pose, boundary_frame, boundary_pose, departure_side_, assign_single);
      if (first.empty()) {
        boundary_lost_ = true;
        first_recovered_ = false;
        return false;
      }
      if (boundary_lost_ && !returning_) return false;
      auto opposite = inside && first_recovered_ ?
        innermost_boundary(lane, *pose, boundary_frame, boundary_pose, -departure_side_) : std::vector<Vec2>{};
      (left_first ? lane.left : lane.right) = std::move(first);
      (left_first ? lane.right : lane.left) = std::move(opposite);
      center_from_boundary(lane, departure_side_, lane_half_width);
      restored_pair = boundaries_consistent(lane, true);
      if (!restored_pair) (left_first ? lane.right : lane.left).clear();
    }
  }
  const auto finite = [](const Vec2 & p) { return std::isfinite(p.x) && std::isfinite(p.y); };
  if (!lane.valid || !std::all_of(lane.path.begin(), lane.path.end(), finite) ||
      !std::all_of(lane.center.begin(), lane.center.end(), finite) || !boundaries_consistent(lane)) return false;
  if (restored_pair) {
    departure_side_ = 0;
    peak_outward_m_ = 0.0;
    boundary_lost_ = returning_ = first_recovered_ = false;
    recovered_road_.reset();
  } else if (departure_side_ != 0) {
    first_recovered_ = true;
    if (returning_) recovered_road_ = Road{reference_center(lane), *pose};
  }
  if (!road_ || (!avoiding && departure_side_ == 0) || restored_pair) road_ = Road{reference_center(lane), *pose};
  return true;
}

LaneObservation lane_from_pixels(const LanePixels & in, double t) {
  LaneObservation out;
  out.capture_time = t;
  if (in.source_width <= 0 || !(in.bev_width_m > 0) || !(in.bev_height_m > 0)) return out;
  const double mpp = in.bev_width_m / in.source_width;
  const double x_max = in.bev_height_m, y_max = in.bev_width_m / 2.0;
  auto conv = [&](const std::vector<LanePixels::P> & src, std::vector<Vec2> & dst) {
    dst.reserve(src.size());
    for (const auto & p : src) dst.push_back({x_max - (p.y + 0.5) * mpp, y_max - (p.x - in.padding_left + 0.5) * mpp});
  };
  if (in.centerline_valid) conv(in.centerline, out.path);
  conv(in.reference, out.center);
  conv(in.left, out.left);
  conv(in.right, out.right);
  for (auto * side : {&out.left, &out.right})
    std::sort(side->begin(), side->end(), [](const Vec2 & a, const Vec2 & b) { return a.x < b.x; });
  out.stop_line_m = std::isfinite(in.stop_line_m) ? in.stop_line_m : std::numeric_limits<double>::quiet_NaN();
  out.valid = out.path.size() >= 4;
  return out;
}

}  // namespace auto_control::drive
