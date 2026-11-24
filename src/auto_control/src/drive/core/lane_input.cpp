#include "auto_control/drive/core/lane_input.hpp"
#include "auto_control/drive/core/cone_memory.hpp"
#include "auto_control/drive/core/path_utils.hpp"
#include <algorithm>
#include <cmath>

namespace auto_control::drive {

namespace {
constexpr double kMaximumSegmentGap = 0.25;
constexpr double kMinimumCenterOverlap = 0.30;
constexpr double kMinimumBoundaryOverlap = 0.18;
constexpr double kMaximumLateralDifference = 0.12;
constexpr double kMinimumHeadingCosine = 0.906307787;  // 25 degrees
constexpr double kMinimumMatchFraction = 0.75;
constexpr double kMaximumConfirmationGap = 0.15;
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
}  // namespace

bool LaneContinuity::accept(const LaneObservation & lane, const std::optional<Pose2> & pose,
                            bool avoiding, double inside_half_width, double wheelbase) {
  if (avoiding && reference_) holding_ = true;
  const auto finite = [](const Vec2 & p) { return std::isfinite(p.x) && std::isfinite(p.y); };
  if (!lane.valid || !pose || !std::isfinite(lane.capture_time) || lane.capture_time <= last_capture_ ||
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
    if (observed_span(lane.path) < kMinimumCenterOverlap ||
        !matches(reference_center(lane), *pose, old_center, old.pose, kMinimumCenterOverlap) ||
        !(matches(lane.left, *pose, old.lane.left, old.pose, kMinimumBoundaryOverlap) ||
          matches(lane.right, *pose, old.lane.right, old.pose, kMinimumBoundaryOverlap))) {
      matching_frames_ = 0;
      return false;
    }
    matching_frames_ = std::min(matching_frames_ + 1, kConfirmationFrames);
    if (matching_frames_ < kConfirmationFrames) return false;
    if (avoiding) return true;

    const auto old_line = frenet_frame(old_center);
    const auto front = project_sl(to_local(to_world({0.0, 0.0}, *pose), old.pose), old_line);
    const auto rear = project_sl(to_local(to_world({-wheelbase, 0.0}, *pose), old.pose), old_line);
    if (std::max(std::abs(front.y), std::abs(rear.y)) > inside_half_width) return true;
    holding_ = false;
    matching_frames_ = 0;
  }
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
