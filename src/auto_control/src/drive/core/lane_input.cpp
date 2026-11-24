#include "auto_control/drive/core/lane_input.hpp"
#include "auto_control/drive/core/cone_memory.hpp"
#include "auto_control/drive/core/path_utils.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace auto_control::drive {

namespace {
constexpr double kMaximumSegmentGap = 0.25;
constexpr double kMinimumCenterOverlap = 0.30;
constexpr double kMinimumBoundaryOverlap = 0.18;
constexpr double kMaximumLateralDifference = 0.12;
constexpr double kMinimumHeadingCosine = 0.906307787;  // 25 degrees
constexpr double kMinimumMatchFraction = 0.75;
constexpr double kMaximumConfirmationGap = 0.15;
constexpr double kMinimumLaneWidth = 0.50;
constexpr double kWidthStationTolerance = 0.08;
constexpr double kBoundaryStationSpacing = 0.04;
constexpr double kSameBoundarySpread = 0.08;
constexpr int kConfirmationFrames = 3;

const std::vector<Vec2> & reference_center(const LaneObservation & lane) {
  return lane.center.size() >= 4 ? lane.center : lane.path;
}

struct Segment {
  Vec2 start;
  Vec2 direction;
  double length;
};

std::vector<Segment> segments(const std::vector<Vec2> & points) {
  std::vector<Segment> out;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto start = points[i - 1];
    const Vec2 delta{points[i].x - start.x, points[i].y - start.y};
    const double length = std::hypot(delta.x, delta.y);
    if (length >= 1e-5 && length <= kMaximumSegmentGap)
      out.push_back({start, {delta.x / length, delta.y / length}, length});
  }
  return out;
}

double observed_span(const std::vector<Vec2> & points) {
  double span = 0.0;
  for (const auto & segment : segments(points)) span += segment.length;
  return span;
}

bool matches(const std::vector<Vec2> & points, const Pose2 & pose,
             const std::vector<Vec2> & reference, const Pose2 & reference_pose, double minimum_overlap) {
  const auto old_segments = segments(reference);
  if (old_segments.empty()) return false;
  const double yaw = pose.yaw - reference_pose.yaw;
  const double c = std::cos(yaw), s = std::sin(yaw);
  double overlap = 0.0, matched = 0.0;
  for (const auto & segment : segments(points)) {
    const Vec2 mid{segment.start.x + segment.direction.x * segment.length / 2,
                   segment.start.y + segment.direction.y * segment.length / 2};
    const Vec2 q = to_local(to_world(mid, pose), reference_pose);
    const Vec2 direction{c * segment.direction.x - s * segment.direction.y,
                         s * segment.direction.x + c * segment.direction.y};
    double nearest = std::numeric_limits<double>::infinity(), alignment = 0.0;
    for (const auto & old : old_segments) {
      const Vec2 delta{q.x - old.start.x, q.y - old.start.y};
      const double along = delta.x * old.direction.x + delta.y * old.direction.y;
      if (along < 0.0 || along > old.length) continue;  // Compare only where the old line was observed.
      const double lateral = std::abs(delta.x * old.direction.y - delta.y * old.direction.x);
      if (lateral < nearest) {
        nearest = lateral;
        alignment = direction.x * old.direction.x + direction.y * old.direction.y;
      }
    }
    if (!std::isfinite(nearest)) continue;
    overlap += segment.length;
    if (nearest <= kMaximumLateralDifference && alignment >= kMinimumHeadingCosine)
      matched += segment.length;
  }
  return matched >= minimum_overlap && matched >= kMinimumMatchFraction * overlap;
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
  return paired || !require_pair;
}

std::vector<Vec2> recover_boundary(const LaneObservation & lane, const Pose2 & pose,
                                   const std::vector<Vec2> & old, const Pose2 & old_pose) {
  if (old.size() < 2) return {};
  const auto line = frenet_frame(old);
  std::map<int, std::pair<double, Vec2>> nearest;
  // Model labels may flip when reacquiring the same physical boundary.
  for (const auto * points : {&lane.left, &lane.right}) {
    for (const auto & point : *points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
      const auto sl = project_sl(to_local(to_world(point, pose), old_pose), line);
      const double lateral = std::abs(sl.y);
      if (sl.x < 0.0 || sl.x > line.arc.back() || lateral > kMaximumLateralDifference) continue;
      const int station = static_cast<int>(std::lround(sl.x / kBoundaryStationSpacing));
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
  if (best_end - best_start < 4 || best_length < kMinimumBoundaryOverlap) return {};
  std::vector<Vec2> best(ordered.begin() + best_start, ordered.begin() + best_end);
  if (!matches(best, pose, old, old_pose, kMinimumBoundaryOverlap)) return {};
  return best;
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

bool LaneContinuity::accept(LaneObservation & lane, const std::optional<Pose2> & pose,
                            bool avoiding, double lane_half_width, double vehicle_half_width, double wheelbase) {
  if (avoiding && reference_) holding_ = true;
  const auto finite = [](const Vec2 & p) { return std::isfinite(p.x) && std::isfinite(p.y); };
  if (!pose || !std::isfinite(lane.capture_time) || lane.capture_time <= last_capture_ ||
      !std::all_of(lane.path.begin(), lane.path.end(), finite) ||
      !std::all_of(lane.center.begin(), lane.center.end(), finite)) {
    matching_frames_ = 0;
    return false;
  }
  if (lane.capture_time - last_capture_ > kMaximumConfirmationGap) matching_frames_ = 0;
  last_capture_ = lane.capture_time;
  if (holding_) {
    const auto & old = *reference_;
    const auto & old_center = reference_center(old.lane);
    const auto old_line = frenet_frame(old_center);
    const auto front = project_sl(to_local(to_world({0.0, 0.0}, *pose), old.pose), old_line);
    const auto rear = project_sl(to_local(to_world({-wheelbase, 0.0}, *pose), old.pose), old_line);
    const double outer_axle = std::abs(front.y) >= std::abs(rear.y) ? front.y : rear.y;
    const bool inside = std::abs(outer_axle) + vehicle_half_width <= lane_half_width;
    if (departure_side_ == 0 && !inside) departure_side_ = outer_axle > 0.0 ? 1 : -1;

    bool agreed = false;
    if (departure_side_ != 0) {
      const bool left_first = departure_side_ > 0;
      auto first = recover_boundary(lane, *pose, left_first ? old.lane.left : old.lane.right, old.pose);
      if (!first.empty()) {
        auto opposite = inside ? recover_boundary(lane, *pose,
          left_first ? old.lane.right : old.lane.left, old.pose) : std::vector<Vec2>{};
        if (!lane.valid || !matches(reference_center(lane), *pose, old_center, old.pose, kMinimumCenterOverlap))
          lane.stop_line_m = std::numeric_limits<double>::quiet_NaN();
        (left_first ? lane.left : lane.right) = std::move(first);
        (left_first ? lane.right : lane.left) = std::move(opposite);
        center_from_boundary(lane, departure_side_, lane_half_width);
        if (!boundaries_consistent(lane, true)) (left_first ? lane.right : lane.left).clear();
        agreed = matches(lane.path, *pose, old_center, old.pose, kMinimumCenterOverlap);
      }
    } else if (lane.valid && observed_span(lane.path) >= kMinimumCenterOverlap &&
               matches(reference_center(lane), *pose, old_center, old.pose, kMinimumCenterOverlap)) {
      const bool left = matches(lane.left, *pose, old.lane.left, old.pose, kMinimumBoundaryOverlap);
      const bool right = matches(lane.right, *pose, old.lane.right, old.pose, kMinimumBoundaryOverlap);
      agreed = (left || right) && boundaries_consistent(lane);
    }
    if (!agreed) {
      matching_frames_ = 0;
      return false;
    }
    matching_frames_ = std::min(matching_frames_ + 1, kConfirmationFrames);
    if (matching_frames_ < kConfirmationFrames) return false;
    if (avoiding || !inside) return true;
    holding_ = false;
    departure_side_ = 0;
    matching_frames_ = 0;
  }
  if (!lane.valid || !boundaries_consistent(lane)) return false;
  if (observed_span(reference_center(lane)) >= kMinimumCenterOverlap &&
      (observed_span(lane.left) >= kMinimumBoundaryOverlap || observed_span(lane.right) >= kMinimumBoundaryOverlap))
    reference_ = Reference{lane, *pose};
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
