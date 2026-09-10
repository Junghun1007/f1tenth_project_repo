#include "line_detactor/centerline.hpp"
#include "line_detactor/lane_connector.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <utility>
#include <opencv2/imgproc.hpp>

namespace line_detactor
{
namespace
{
using Point = cv::Point2d;
using Path = std::vector<Point>;
constexpr double pi = 3.14159265358979323846;
double norm(const Point & p) {return std::hypot(p.x, p.y);}
Point unit(const Point & p) {return p * (1.0 / std::max(norm(p), 1.0e-9));}
double radians(double degrees) {return degrees * pi / 180.0;}

std::vector<double> arc_lengths(const Path & p)
{
  std::vector<double> arc(p.size(), 0.0);
  for (std::size_t i = 1; i < p.size(); ++i) {arc[i] = arc[i - 1] + norm(p[i] - p[i - 1]);}
  return arc;
}
Path resample(const Path & p, double step)
{
  if (p.size() < 2U) {return p;}
  const auto arc = arc_lengths(p);
  const int count = std::max(2, static_cast<int>(std::ceil(arc.back() / step)) + 1);
  Path out;
  std::size_t j = 1U;
  for (int i = 0; i < count; ++i) {
    const double s = arc.back() * i / (count - 1);
    while (j + 1U < p.size() && arc[j] < s) {++j;}
    const double t = (s - arc[j - 1]) / std::max(arc[j] - arc[j - 1], 1.0e-9);
    out.push_back(p[j - 1] + (p[j] - p[j - 1]) * t);
  }
  return out;
}
Path gaussian(const Path & p, double sigma_samples)
{
  if (sigma_samples <= 0.0 || p.size() < 2U) {return p;}
  Path out(p.size());
  const int radius = std::min(static_cast<int>(p.size()),
    static_cast<int>(std::ceil(3.0 * sigma_samples)));
  for (int i = 0; i < static_cast<int>(p.size()); ++i) {
    double total = 0.0;
    for (int d = -radius; d <= radius; ++d) {
      const int j = std::clamp(i + d, 0, static_cast<int>(p.size()) - 1);
      const double w = std::exp(-0.5 * d * d / (sigma_samples * sigma_samples));
      out[i] += p[j] * w;
      total += w;
    }
    out[i] *= 1.0 / total;
  }
  return out;
}
Path tangents(const Path & p, int half_window)
{
  Path result;
  for (int i = 0; i < static_cast<int>(p.size()); ++i) {
    result.push_back(unit(p[std::min(i + half_window, static_cast<int>(p.size()) - 1)] -
      p[std::max(0, i - half_window)]));
  }
  return result;
}

// Metric spatial bins keep clearance and graph-neighbor queries local.
class SpatialIndex
{
public:
  SpatialIndex(const Path & points, double cell) : points_(points), cell_(cell)
  {
    for (int i = 0; i < static_cast<int>(points_.size()); ++i) {bins_[key(points_[i])].push_back(i);}
  }
  std::vector<int> nearby(const Point & p, double radius) const
  {
    std::vector<int> ids;
    const auto lo = key(p - Point(radius, radius));
    const auto hi = key(p + Point(radius, radius));
    for (int y = lo.second; y <= hi.second; ++y) {
      for (int x = lo.first; x <= hi.first; ++x) {
        const auto found = bins_.find({x, y});
        if (found == bins_.end()) {continue;}
        for (int id : found->second) {
          if (norm(points_[id] - p) <= radius) {ids.push_back(id);}
        }
      }
    }
    return ids;
  }
  bool clear(const Point & p, double radius) const
  {
    const auto lo = key(p - Point(radius, radius));
    const auto hi = key(p + Point(radius, radius));
    for (int y = lo.second; y <= hi.second; ++y) {
      for (int x = lo.first; x <= hi.first; ++x) {
        const auto found = bins_.find({x, y});
        if (found == bins_.end()) {continue;}
        for (int id : found->second) {
          if (norm(points_[id] - p) <= radius) {return false;}
        }
      }
    }
    return true;
  }
private:
  std::pair<int, int> key(const Point & p) const
  {
    return {static_cast<int>(std::floor(p.x / cell_)), static_cast<int>(std::floor(p.y / cell_))};
  }
  const Path & points_;
  double cell_;
  std::map<std::pair<int, int>, std::vector<int>> bins_;
};

struct Geometry
{
  double width;
  double height;
  double margin;
  double clearance;
  double check_step;
  const SpatialIndex & boundary;
  bool point_ok(const Point & p) const
  {
    return std::isfinite(p.x) && std::isfinite(p.y) && p.x >= -margin &&
           p.x <= width + margin && p.y >= 0.0 && p.y < height && boundary.clear(p, clearance);
  }
  bool segment_ok(const Point & a, const Point & b) const
  {
    const int steps = std::max(1, static_cast<int>(std::ceil(norm(b - a) / check_step)));
    for (int i = 0; i <= steps; ++i) {
      if (!point_ok(a + (b - a) * (static_cast<double>(i) / steps))) {return false;}
    }
    return true;
  }
  bool path_ok(const Path & p) const
  {
    for (std::size_t i = 1; i < p.size(); ++i) {
      if (!segment_ok(p[i - 1], p[i])) {return false;}
    }
    return p.size() >= 2U;
  }
};

Path bounded_smooth(const Path & original, const Path & proposal, double max_shift,
  const Geometry & geometry)
{
  double largest = 0.0;
  for (std::size_t i = 0; i < original.size(); ++i) {
    largest = std::max(largest, norm(proposal[i] - original[i]));
  }
  double factor = std::min(1.0, max_shift / std::max(largest, 1.0e-9));
  for (int attempt = 0; attempt < 9; ++attempt, factor *= 0.5) {
    Path out = original;
    for (std::size_t i = 1; i + 1U < out.size(); ++i) {
      out[i] += (proposal[i] - original[i]) * factor;
    }
    bool forward = true;
    for (std::size_t i = 1; i < out.size(); ++i) {
      if ((out[i] - out[i - 1]).dot(original[i] - original[i - 1]) <= 0.0) {forward = false;}
    }
    if (forward && geometry.path_ok(out)) {return out;}
  }
  return original;
}

// Local quadratic least squares on metric arc coordinates: Savitzky-Golay style
// smoothing without adding Python/SciPy to the ROS runtime.
Path quadratic(const Path & p, double window_m)
{
  const auto arc = arc_lengths(p);
  Path out = p;
  const double half = window_m / 2.0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    const double middle = std::clamp(arc[i], std::min(half, arc.back() / 2.0),
      std::max(arc.back() - half, arc.back() / 2.0));
    cv::Matx33d a = cv::Matx33d::zeros();
    cv::Vec3d bx(0, 0, 0), by(0, 0, 0);
    int count = 0;
    for (std::size_t j = 0; j < p.size(); ++j) {
      if (std::abs(arc[j] - middle) > half) {continue;}
      const double t = (arc[j] - arc[i]) / half;
      const cv::Vec3d row(1.0, t, t * t);
      for (int r = 0; r < 3; ++r) {
        bx[r] += row[r] * p[j].x;
        by[r] += row[r] * p[j].y;
        for (int c = 0; c < 3; ++c) {a(r, c) += row[r] * row[c];}
      }
      ++count;
    }
    cv::Vec3d x, y;
    if (count >= 5 && cv::solve(a, bx, x, cv::DECOMP_SVD) && cv::solve(a, by, y, cv::DECOMP_SVD)) {
      out[i] = Point(x[0], y[0]);
    }
  }
  return out;
}
Path smooth_centerline(const Path & path, const Geometry & geometry, const CenterlineConfig & cfg)
{
  if (!cfg.smoothing_enabled || cfg.smoothing_strength == 0.0 || path.size() < 7U) {return path;}
  const double ds = arc_lengths(path).back() / (path.size() - 1U);
  auto local = gaussian(path, cfg.smoothing_sigma_m / ds);
  for (std::size_t i = 0; i < path.size(); ++i) {
    local[i] = path[i] + (local[i] - path[i]) * cfg.smoothing_strength;
  }
  auto base = bounded_smooth(path, local, cfg.smoothing_max_shift_m, geometry);
  auto broad = quadratic(base, cfg.smoothing_window_m);
  const auto directions = tangents(broad, 1);
  const int span = std::max(1, static_cast<int>(std::round(cfg.turn_window_m / (2.0 * ds))));
  Path weights(base.size());
  for (int i = 0; i < static_cast<int>(base.size()); ++i) {
    const auto a = directions[std::max(0, i - span)];
    const auto b = directions[std::min(static_cast<int>(base.size()) - 1, i + span)];
    const double turn = std::acos(std::clamp(a.dot(b), -1.0, 1.0));
    weights[i].x = 1.0 - std::clamp((turn - radians(cfg.straight_turn_deg)) /
      radians(cfg.corner_turn_deg - cfg.straight_turn_deg), 0.0, 1.0);
  }
  weights = gaussian(weights, 0.05 / ds);
  for (std::size_t i = 0; i < base.size(); ++i) {
    const double taper = std::min(1.0, std::min(i, base.size() - 1U - i) * ds / 0.12);
    broad[i] = base[i] + (broad[i] - base[i]) * (weights[i].x * taper * cfg.smoothing_strength);
  }
  return bounded_smooth(base, broad, cfg.smoothing_max_shift_m, geometry);
}

struct Fragment
{
  int side;
  Path points;
  Path directions;
  std::vector<double> arc;
};
struct Candidate
{
  Point point;
  Point direction;
  std::uint8_t support;
};
}  // namespace

void validate_centerline(const CenterlineConfig & c)
{
  const auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
  const double values[] = {c.lane_width_m, c.bev_width_m, c.bev_height_m, c.sample_spacing_m,
    c.min_fragment_length_m, c.tangent_window_m, c.width_tolerance_m,
    c.pair_along_tolerance_m, c.max_gap_m, c.max_start_distance_m, c.min_clearance_m,
    c.smoothing_window_m, c.smoothing_max_shift_m, c.turn_window_m};
  for (double v : values) {
    if (!positive(v)) {throw std::invalid_argument("Centerline distances must be finite and positive");}
  }
  if (!std::isfinite(c.outside_margin_m) || c.outside_margin_m < 0.0 ||
    !std::isfinite(c.smoothing_sigma_m) || c.smoothing_sigma_m < 0.0 ||
    !std::isfinite(c.smoothing_strength) || c.smoothing_strength < 0.0 || c.smoothing_strength > 1.0 ||
    !positive(c.pair_heading_tolerance_deg) || c.pair_heading_tolerance_deg >= 90.0 ||
    !std::isfinite(c.straight_turn_deg) || c.straight_turn_deg < 0.0 ||
    !positive(c.corner_turn_deg) || c.corner_turn_deg <= c.straight_turn_deg || c.corner_turn_deg >= 180.0 ||
    c.bev_width_m < 0.1 || c.bev_height_m < 0.1 || c.lane_width_m < 0.1 || c.lane_width_m > 5.0 ||
    c.min_clearance_m < 0.005 || c.max_gap_m > 2.0 || c.tangent_window_m > 5.0 ||
    c.sample_spacing_m < 0.005 || c.sample_spacing_m > 0.10 ||
    c.tangent_window_m < c.sample_spacing_m || c.max_gap_m < 2.0 * c.sample_spacing_m ||
    c.min_clearance_m >= c.lane_width_m / 2.0 || c.width_tolerance_m >= c.lane_width_m ||
    c.smoothing_window_m < 5.0 * c.sample_spacing_m || c.bev_width_m > 20.0 || c.bev_height_m > 20.0 ||
    c.smoothing_window_m > 5.0 || c.smoothing_sigma_m > 1.0 || c.turn_window_m > 5.0 ||
    c.outside_margin_m > 2.0 || c.max_samples < 32 || c.max_samples > 5000 ||
    c.line_width_px < 1 || c.line_width_px > 10)
  {throw std::invalid_argument("Invalid centerline geometry, smoothing or resource parameters");}
}

CenterlineResult generate_centerline(
  const cv::Mat & labels, int source_width, int padding, const CenterlineConfig & cfg)
{
  CenterlineResult result;
  result.mask = cv::Mat::zeros(labels.size(), CV_8UC1);
  if (!cfg.enabled) {return result;}
  if (labels.empty() || labels.type() != CV_8UC1 || source_width <= 0 || padding < 0 ||
    labels.cols != source_width + 2 * padding)
  {throw std::invalid_argument("Centerline requires mono8 labels and matching source/padding dimensions");}
  const double sx = cfg.bev_width_m / source_width;
  const double sy = cfg.bev_height_m / labels.rows;
  const auto metric = [=](const cv::Point2f & p) {return Point((p.x - padding) * sx, p.y * sy);};
  const Point ego(cfg.bev_width_m / 2.0, cfg.bev_height_m);
  Path boundary;
  std::vector<cv::Point> pixels;
  cv::findNonZero((labels == 1) | (labels == 2), pixels);
  for (const auto & p : pixels) {boundary.push_back(metric(cv::Point2f(p)));}
  if (boundary.empty()) {return result;}
  const SpatialIndex boundary_index(boundary, cfg.min_clearance_m);
  // Include pixel-cell half diagonal and half sampling step for a conservative
  // sampled clearance check. This measures observed pixels, not drivable space.
  const double check_step = std::min(0.005, cfg.sample_spacing_m / 2.0);
  const Geometry geometry{cfg.bev_width_m, cfg.bev_height_m,
    std::min(cfg.outside_margin_m, std::max(0, padding - cfg.line_width_px) * sx),
    cfg.min_clearance_m + 0.5 * std::hypot(sx, sy) + check_step / 2.0, check_step, boundary_index};
  const auto observed = observed_lane_paths(labels);
  std::vector<Fragment> fragments;
  std::size_t samples = 0U;
  for (int side = 0; side < 2; ++side) {
    for (const auto & curve : observed[side]) {
      Path points;
      for (const auto & p : curve) {points.push_back(metric(p));}
      const auto raw_arc = arc_lengths(points);
      if (raw_arc.back() < cfg.min_fragment_length_m) {continue;}
      const auto required = static_cast<std::size_t>(std::ceil(raw_arc.back() / cfg.sample_spacing_m)) + 1U;
      if (samples + required > static_cast<std::size_t>(cfg.max_samples)) {
        result.sample_limit_reached = true;
        return result;
      }
      if (norm(points.front() - ego) > norm(points.back() - ego)) {std::reverse(points.begin(), points.end());}
      points = gaussian(resample(points, cfg.sample_spacing_m), 1.0);
      const int half = std::max(1, static_cast<int>(std::round(cfg.tangent_window_m / (2.0 * cfg.sample_spacing_m))));
      Fragment f{side, points, tangents(points, half), arc_lengths(points)};
      samples += points.size();
      fragments.push_back(std::move(f));
    }
  }
  std::vector<Candidate> candidates;
  for (const auto & f : fragments) {
    Path offsets;
    std::vector<std::uint8_t> support;
    for (std::size_t i = 0; i < f.points.size(); ++i) {
      const auto & p = f.points[i];
      const auto & t = f.directions[i];
      const Point normal = Point(-t.y, t.x) * (f.side == 0 ? 1.0 : -1.0);
      Point center = p + normal * (cfg.lane_width_m / 2.0);
      std::uint8_t mode = 1U;
      double best = std::numeric_limits<double>::infinity();
      for (const auto & other : fragments) {
        if (other.side == f.side) {continue;}
        for (std::size_t j = 0; j < other.points.size(); ++j) {
          if (other.arc[j] < 0.025 || other.arc.back() - other.arc[j] < 0.025) {continue;}
          const Point delta = other.points[j] - p;
          const double across = delta.dot(normal);
          const double along = std::abs(delta.dot(t));
          if (std::abs(across - cfg.lane_width_m) > cfg.width_tolerance_m ||
            along > cfg.pair_along_tolerance_m ||
            other.directions[j].dot(t) < std::cos(radians(cfg.pair_heading_tolerance_deg))) {continue;}
          const double score = 3.0 * along + std::abs(across - cfg.lane_width_m);
          if (score < best) {best = score; center = (p + other.points[j]) * 0.5; mode = 2U;}
        }
      }
      offsets.push_back(center);
      support.push_back(mode);
    }
    offsets = gaussian(offsets, 1.2);
    for (std::size_t i = 0; i < offsets.size(); ++i) {
      const auto a = offsets[i == 0U ? i : i - 1U];
      const auto b = offsets[std::min(i + 1U, offsets.size() - 1U)];
      if ((b - a).dot(f.directions[i]) <= 0.001 || !geometry.point_ok(offsets[i])) {continue;}
      candidates.push_back({offsets[i], f.directions[i], support[i]});
    }
  }
  if (candidates.size() < 2U) {return result;}
  Path positions;
  int start = 0;
  double nearest = std::numeric_limits<double>::infinity();
  for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
    positions.push_back(candidates[i].point);
    const double d = norm(candidates[i].point - ego);
    if (d < nearest) {nearest = d; start = i;}
  }
  if (nearest > cfg.max_start_distance_m) {return result;}
  const SpatialIndex neighbors(positions, cfg.max_gap_m);
  std::vector<double> cost(positions.size(), std::numeric_limits<double>::infinity());
  std::vector<int> previous(positions.size(), -1);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  cost[start] = 0.0;
  queue.emplace(0.0, start);
  while (!queue.empty()) {
    const auto [distance, i] = queue.top();
    queue.pop();
    if (distance > cost[i]) {continue;}
    for (int j : neighbors.nearby(positions[i], cfg.max_gap_m)) {
      const auto delta = positions[j] - positions[i];
      const double length = norm(delta);
      if (length < 0.006) {continue;}
      const auto direction = delta * (1.0 / length);
      const double aligned = direction.dot(candidates[i].direction);
      if (aligned < 0.5 || direction.dot(candidates[j].direction) < 0.5 ||
        candidates[i].direction.dot(candidates[j].direction) < 0.5 ||
        !geometry.segment_ok(positions[i], positions[j])) {continue;}
      double next = distance + length * (1.0 + (candidates[j].support == 1U ? 0.6 : 0.0) +
        3.0 * (1.0 - aligned));
      next += std::max(0.0, length - 0.065) * 3.0;
      if (next < cost[j]) {cost[j] = next; previous[j] = i; queue.emplace(next, j);}
    }
  }
  int end = start;
  for (int i = 0; i < static_cast<int>(cost.size()); ++i) {
    if (std::isfinite(cost[i]) && norm(positions[i] - ego) > norm(positions[end] - ego)) {end = i;}
  }
  std::vector<int> ids;
  for (int i = end; i >= 0; i = previous[i]) {ids.push_back(i);}
  std::reverse(ids.begin(), ids.end());
  if (ids.size() < 2U) {return result;}
  Path raw;
  for (int id : ids) {raw.push_back(positions[id]);}
  const auto raw_arc = arc_lengths(raw);
  Path path = resample(raw, 0.01);
  const double length = raw_arc.back();
  std::vector<std::uint8_t> provenance;
  std::size_t j = 1U;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double s = length * i / (path.size() - 1U);
    while (j + 1U < raw.size() && raw_arc[j] < s) {++j;}
    provenance.push_back(i == 0U ? candidates[ids.front()].support :
      (raw_arc[j] - raw_arc[j - 1U] > 0.065 ? 3U : candidates[ids[j]].support));
  }
  if (!geometry.path_ok(path)) {return result;}
  path = smooth_centerline(path, geometry, cfg);
  if (!geometry.path_ok(path)) {return result;}
  result.support = std::move(provenance);
  for (const auto & p : path) {
    result.points.emplace_back(static_cast<float>(p.x / sx + padding), static_cast<float>(p.y / sy));
  }
  for (std::size_t i = 1; i < result.points.size(); ++i) {
    cv::line(result.mask, result.points[i - 1], result.points[i], cv::Scalar(255), cfg.line_width_px, cv::LINE_8);
  }
  return result;
}
}  // namespace line_detactor
