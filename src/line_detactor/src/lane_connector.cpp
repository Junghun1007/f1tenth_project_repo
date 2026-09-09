#include "line_detactor/lane_connector.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace line_detactor
{
namespace
{
using Point = cv::Point2f;
constexpr double kPi = 3.14159265358979323846;

double length(const Point & p) {return std::hypot(p.x, p.y);}
double cross(const Point & a, const Point & b) {return a.x * b.y - a.y * b.x;}
Point unit(const Point & p)
{
  const double size = length(p);
  return size > 1.0e-6 ? p * static_cast<float>(1.0 / size) : Point();
}

double arc_length(const std::vector<Point> & points)
{
  double total = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    total += length(points[i] - points[i - 1]);
  }
  return total;
}

// Zhang-Suen thinning. Pad the ROI so border-touching endpoints are retained.
cv::Mat thin(const cv::Mat & mask)
{
  cv::Mat image;
  cv::copyMakeBorder(mask, image, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);
  image /= 255;
  bool changed = true;
  while (changed) {
    changed = false;
    for (int pass = 0; pass < 2; ++pass) {
      std::vector<cv::Point> remove;
      for (int y = 1; y < image.rows - 1; ++y) {
        for (int x = 1; x < image.cols - 1; ++x) {
          if (!image.at<std::uint8_t>(y, x)) {continue;}
          const int n[8] = {
            image.at<std::uint8_t>(y - 1, x), image.at<std::uint8_t>(y - 1, x + 1),
            image.at<std::uint8_t>(y, x + 1), image.at<std::uint8_t>(y + 1, x + 1),
            image.at<std::uint8_t>(y + 1, x), image.at<std::uint8_t>(y + 1, x - 1),
            image.at<std::uint8_t>(y, x - 1), image.at<std::uint8_t>(y - 1, x - 1)};
          int neighbors = 0;
          int transitions = 0;
          for (int k = 0; k < 8; ++k) {
            neighbors += n[k];
            transitions += n[k] == 0 && n[(k + 1) % 8] != 0;
          }
          const bool removable = pass == 0 ?
            n[0] * n[2] * n[4] == 0 && n[2] * n[4] * n[6] == 0 :
            n[0] * n[2] * n[6] == 0 && n[0] * n[4] * n[6] == 0;
          if (neighbors >= 2 && neighbors <= 6 && transitions == 1 && removable) {
            remove.emplace_back(x, y);
          }
        }
      }
      for (const auto & point : remove) {image.at<std::uint8_t>(point) = 0;}
      changed = changed || !remove.empty();
    }
  }
  return image(cv::Rect(1, 1, mask.cols, mask.rows)).clone();
}

// Keep a single ordered skeleton path per component; side branches are not lanes.
std::vector<Point> component_path(const cv::Mat & mask, const cv::Point & origin)
{
  const cv::Mat skeleton = thin(mask);
  std::vector<cv::Point> pixels;
  cv::findNonZero(skeleton, pixels);
  if (pixels.size() < 2U) {return {};}
  cv::Mat indices(mask.size(), CV_32S, cv::Scalar(-1));
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    indices.at<int>(pixels[i]) = static_cast<int>(i);
  }
  std::vector<std::vector<int>> adjacent(pixels.size());
  std::vector<int> endpoints;
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    const auto p = pixels[i];
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const cv::Point q = p + cv::Point(dx, dy);
        if ((dx == 0 && dy == 0) || q.x < 0 || q.y < 0 ||
          q.x >= indices.cols || q.y >= indices.rows) {continue;}
        const int neighbor = indices.at<int>(q);
        // Avoid diagonal shortcuts around a 4-connected corner.
        if (dx && dy && (indices.at<int>(p.y, q.x) >= 0 ||
          indices.at<int>(q.y, p.x) >= 0)) {continue;}
        if (neighbor >= 0) {adjacent[i].push_back(neighbor);}
      }
    }
    if (adjacent[i].size() == 1U) {endpoints.push_back(static_cast<int>(i));}
  }
  if (endpoints.size() < 2U) {return {};}
  const int start = *std::max_element(endpoints.begin(), endpoints.end(), [&](int a, int b) {
    return pixels[a].y < pixels[b].y;
  });
  std::vector<double> distance(pixels.size(), std::numeric_limits<double>::infinity());
  std::vector<int> previous(pixels.size(), -1);
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  distance[start] = 0.0;
  queue.emplace(0.0, start);
  while (!queue.empty()) {
    const auto [cost, i] = queue.top();
    queue.pop();
    if (cost > distance[i]) {continue;}
    for (const int j : adjacent[i]) {
      const double next = cost + cv::norm(pixels[i] - pixels[j]);
      if (next < distance[j]) {
        distance[j] = next;
        previous[j] = i;
        queue.emplace(next, j);
      }
    }
  }
  int end = start;
  for (const int candidate : endpoints) {
    if (std::isfinite(distance[candidate]) && distance[candidate] > distance[end]) {
      end = candidate;
    }
  }
  std::vector<Point> path;
  for (int i = end; i >= 0; i = previous[i]) {
    path.emplace_back(pixels[i] + origin);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<Point> resample(const std::vector<Point> & path)
{
  if (path.empty()) {return {};}
  std::vector<Point> result{path.front()};
  double accumulated = 0.0;
  double target = 1.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double distance = length(path[i] - path[i - 1]);
    while (distance > 0.0 && target <= accumulated + distance) {
      result.push_back(path[i - 1] + (path[i] - path[i - 1]) *
        static_cast<float>((target - accumulated) / distance));
      target += 1.0;
    }
    accumulated += distance;
  }
  if (length(result.back() - path.back()) > 0.01) {
    result.push_back(path.back());
  } else {
    result.back() = path.back();
  }
  return result;
}

void smooth_fragment(std::vector<Point> & path, const LaneSmoothingConfig & config)
{
  if (!config.enabled || static_cast<int>(path.size()) < config.min_segment_rows) {return;}
  // Reuse the spline solver on uniformly sampled arc length, independently for x/y.
  const int size = static_cast<int>(path.size());
  std::vector<LaneRow> rows(2 * size);
  for (int i = 0; i < size; ++i) {
    rows[i] = LaneRow{path[i].x, 1.0F, 0.0F};
    rows[size + i] = LaneRow{path[i].y, 1.0F, 0.0F};
  }
  auto settings = config;
  settings.max_row_jump_px = 1.0e6;
  settings.correction_limit_enabled = false;
  smooth_lane_rows(rows.data(), size, settings);
  double maximum = 0.0;
  for (int i = 0; i < size; ++i) {
    maximum = std::max(maximum, length(Point(rows[i].shift, rows[size + i].shift)));
  }
  const double scale = config.correction_limit_enabled && maximum > 0.0 ?
    std::min(1.0, config.max_correction_px / maximum) : 1.0;
  for (int i = 1; i < size - 1; ++i) {
    // Preserve measured endpoints and taper the correction near them.
    const double taper = std::min(1.0, std::min(i, size - 1 - i) / 3.0);
    path[i] += Point(rows[i].shift, rows[size + i].shift) * static_cast<float>(scale * taper);
  }
}

Point tangent(const std::vector<Point> & path, const bool at_end, const double window)
{
  std::vector<Point> sample;
  double walked = 0.0;
  for (std::size_t j = 0; j < path.size(); ++j) {
    const std::size_t i = at_end ? path.size() - 1U - j : j;
    if (!sample.empty()) {walked += length(path[i] - sample.back());}
    sample.push_back(path[i]);
    if (walked >= window) {break;}
  }
  cv::Vec4f line;
  cv::fitLine(sample, line, cv::DIST_L2, 0.0, 0.01, 0.01);
  Point direction(line[0], line[1]);
  const Point forward = at_end ? sample.front() - sample.back() : sample.back() - sample.front();
  if (direction.dot(forward) < 0.0F) {direction *= -1.0F;}
  return unit(direction);
}

struct Fragment
{
  std::vector<Point> path;
  Point entry;
  Point exit;
  double observed{0.0};
};

bool intersects(const Point & a, const Point & b, const Point & c, const Point & d)
{
  if (std::max(a.x, b.x) < std::min(c.x, d.x) ||
    std::max(c.x, d.x) < std::min(a.x, b.x) ||
    std::max(a.y, b.y) < std::min(c.y, d.y) ||
    std::max(c.y, d.y) < std::min(a.y, b.y)) {return false;}
  const double ab_c = cross(b - a, c - a);
  const double ab_d = cross(b - a, d - a);
  const double cd_a = cross(d - c, a - c);
  const double cd_b = cross(d - c, b - c);
  if (ab_c * ab_d < -1.0e-8 && cd_a * cd_b < -1.0e-8) {return true;}
  const auto on_segment = [](const Point & p, const Point & q, const Point & r) {
    return std::abs(cross(q - p, r - p)) < 1.0e-5 &&
           r.x >= std::min(p.x, q.x) - 1.0e-5 && r.x <= std::max(p.x, q.x) + 1.0e-5 &&
           r.y >= std::min(p.y, q.y) - 1.0e-5 && r.y <= std::max(p.y, q.y) + 1.0e-5;
  };
  return on_segment(a, b, c) || on_segment(a, b, d) ||
         on_segment(c, d, a) || on_segment(c, d, b);
}

bool self_intersects(const std::vector<Point> & points)
{
  for (std::size_t i = 1; i < points.size(); ++i) {
    for (std::size_t j = i + 2U; j < points.size(); ++j) {
      if (intersects(points[i - 1], points[i], points[j - 1], points[j])) {return true;}
    }
  }
  return false;
}

std::vector<Point> bridge(
  const Fragment & a, const Fragment & b, const LaneConnectionConfig & config,
  const int width, const int height)
{
  const Point start = a.path.back();
  const Point end = b.path.front();
  const Point chord = end - start;
  const double gap = length(chord);
  // Forward ordering prevents loops/reuse; x is unrestricted, allowing side excursions.
  if (gap < 0.5 || gap > config.max_gap_px || end.y > start.y + 2.0F) {return {};}
  const double cosine = std::clamp(static_cast<double>(a.exit.dot(b.entry)), -1.0, 1.0);
  if (std::acos(cosine) > config.max_turn_deg * kPi / 180.0) {return {};}
  // Opposing tangents can describe an out-of-view semicircle. Their sum is
  // undefined; the endpoint chord supplies the average travel direction there.
  const Point average = length(a.exit + b.entry) < 0.2 ? unit(chord) : unit(a.exit + b.entry);
  const double corridor = config.corridor_half_width_px +
    gap * std::tan(config.direction_tolerance_deg * kPi / 180.0);
  // A finite-width prediction corridor, not an exact intersection of thin rays.
  if (average.dot(chord) <= 0.0F || std::abs(cross(average, chord)) > corridor ||
    a.exit.dot(chord) < -config.corridor_half_width_px ||
    b.entry.dot(chord) < -config.corridor_half_width_px) {return {};}

  std::vector<Point> best;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const double handle_ratio : {0.25, 0.4, 0.6, 0.9}) {
    const Point c1 = start + a.exit * static_cast<float>(gap * handle_ratio);
    const Point c2 = end - b.entry * static_cast<float>(gap * handle_ratio);
    // The Bezier derivative bound gives <= 1px arc steps.
    const int steps = std::max(8, static_cast<int>(std::ceil(
      3.0 * std::max({length(c1 - start), length(c2 - c1), length(end - c2)}))));
    std::vector<Point> curve;
    double maximum_curvature = 0.0;
    bool valid = true;
    for (int i = 0; i <= steps; ++i) {
      const float t = static_cast<float>(i) / steps;
      const float u = 1.0F - t;
      const Point point = start * (u * u * u) + c1 * (3.0F * u * u * t) +
        c2 * (3.0F * u * t * t) + end * (t * t * t);
      const Point velocity = (c1 - start) * (3.0F * u * u) +
        (c2 - c1) * (6.0F * u * t) + (end - c2) * (3.0F * t * t);
      const Point acceleration = (c2 - c1 * 2.0F + start) * (6.0F * u) +
        (end - c2 * 2.0F + c1) * (6.0F * t);
      const double speed = length(velocity);
      const double curvature = speed > 1.0e-4 ?
        std::abs(cross(velocity, acceleration)) / (speed * speed * speed) :
        std::numeric_limits<double>::infinity();
      maximum_curvature = std::max(maximum_curvature, curvature);
      if (point.x < -config.padding_px || point.x > width - 1 + config.padding_px ||
        point.y < 0.0F || point.y > height - 1 || curvature > config.max_curvature_per_px)
      {valid = false; break;}
      curve.push_back(point);
    }
    if (!valid) {continue;}
    const double arc = arc_length(curve);
    if (arc > gap * config.max_arc_ratio || self_intersects(curve)) {continue;}
    // Prefer short, gently bending bridges, never reward unsupported length.
    const double cost = arc + maximum_curvature * gap;
    if (cost < best_cost) {best_cost = cost; best = std::move(curve);}
  }
  return best;
}

std::vector<Fragment> fragments(
  const cv::Mat & labels, const int lane, const LaneConnectionConfig & config,
  const LaneSmoothingConfig & smoothing)
{
  cv::Mat components, stats, centroids;
  const int count = cv::connectedComponentsWithStats(
    labels == lane + 1, components, stats, centroids, 8, CV_32S);
  std::vector<int> candidates;
  for (int i = 1; i < count; ++i) {
    if (stats.at<int>(i, cv::CC_STAT_AREA) >= config.min_component_area_px) {
      candidates.push_back(i);
    }
  }
  std::stable_sort(candidates.begin(), candidates.end(), [&](int a, int b) {
    return stats.at<int>(a, cv::CC_STAT_AREA) > stats.at<int>(b, cv::CC_STAT_AREA);
  });
  if (candidates.size() > static_cast<std::size_t>(config.max_fragments)) {
    candidates.resize(config.max_fragments);
  }
  std::vector<Fragment> result;
  for (const int i : candidates) {
    const cv::Rect roi(stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
      stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));
    auto path = component_path(components(roi) == i, roi.tl());
    const double observed = arc_length(path);
    if (observed < config.min_fragment_length_px) {continue;}
    path = resample(path);
    smooth_fragment(path, smoothing);
    // Model-supported centerlines stay inside the observed image; only bridges
    // may extend into the side padding.
    bool inside = true;
    for (const auto & point : path) {
      inside = inside && point.x >= 0 &&
        point.x <= labels.cols - 1 && point.y >= 0 && point.y <= labels.rows - 1;
    }
    if (!inside || self_intersects(path)) {
      path = resample(component_path(components(roi) == i, roi.tl()));
    }
    result.push_back(Fragment{
      path, tangent(path, false, config.tangent_window_px),
      tangent(path, true, config.tangent_window_px), observed});
  }
  std::stable_sort(result.begin(), result.end(), [](const Fragment & a, const Fragment & b) {
    return a.path.front().y > b.path.front().y;
  });
  return result;
}

ConnectedLane select_chain(
  const std::vector<Fragment> & parts, const LaneConnectionConfig & config,
  const int width, const int height)
{
  if (parts.empty()) {return {};}
  // A DAG of fragments ordered near-to-far: one predecessor/successor, no branches.
  std::vector<ConnectedLane> chains(parts.size());
  std::vector<double> scores(parts.size());
  int best = -1;
  for (std::size_t j = 0; j < parts.size(); ++j) {
    chains[j].points = parts[j].path;
    chains[j].interpolated.assign(parts[j].path.size(), 0U);
    chains[j].observed_length_px = parts[j].observed;
    scores[j] = parts[j].observed - 0.25 * (height - 1 - parts[j].path.front().y);
    for (std::size_t i = 0; i < j; ++i) {
      if (parts[j].path.back().y >= parts[i].path.back().y) {continue;}
      auto curve = bridge(parts[i], parts[j], config, width, height);
      if (curve.empty()) {continue;}
      const double score = scores[i] + parts[j].observed - 0.05 * arc_length(curve);
      if (score <= scores[j]) {continue;}
      auto combined = chains[i];
      for (std::size_t k = 1; k + 1U < curve.size(); ++k) {
        combined.points.push_back(curve[k]);
        combined.interpolated.push_back(1U);
      }
      combined.points.insert(combined.points.end(), parts[j].path.begin(), parts[j].path.end());
      combined.interpolated.insert(combined.interpolated.end(), parts[j].path.size(), 0U);
      if (self_intersects(combined.points)) {continue;}
      combined.observed_length_px += parts[j].observed;
      chains[j] = std::move(combined);
      scores[j] = score;
    }
    if (chains[j].observed_length_px >= config.min_lane_length_px &&
      (best < 0 || scores[j] > scores[best])) {best = static_cast<int>(j);}
  }
  if (best < 0) {return {};}
  auto result = std::move(chains[best]);
  for (auto & point : result.points) {point.x += config.padding_px;}
  return result;
}

cv::Mat lane_mask(const ConnectedLane & lane, const cv::Size & size, const int thickness)
{
  cv::Mat mask = cv::Mat::zeros(size, CV_8UC1);
  // Two passes ensure model-supported endpoints take priority over bridge pixels.
  for (int source = 1; source >= 0; --source) {
    for (std::size_t i = 1; i < lane.points.size(); ++i) {
      const bool inferred = lane.interpolated[i - 1] || lane.interpolated[i];
      if (inferred != static_cast<bool>(source)) {continue;}
      const auto a = lane.points[i - 1] * 256.0F;
      const auto b = lane.points[i] * 256.0F;
      cv::line(mask, cv::Point(cvRound(a.x), cvRound(a.y)),
        cv::Point(cvRound(b.x), cvRound(b.y)), cv::Scalar(source ? 2 : 1),
        thickness, cv::LINE_8, 8);
    }
  }
  return mask;
}

bool pair_conflict(const cv::Mat & left, const cv::Mat & right)
{
  for (int y = 0; y < left.rows; ++y) {
    int last_left = -1;
    int first_right = right.cols;
    for (int x = 0; x < left.cols; ++x) {
      if (left.at<std::uint8_t>(y, x)) {last_left = x;}
      if (right.at<std::uint8_t>(y, x)) {first_right = std::min(first_right, x);}
    }
    if (last_left >= first_right) {return true;}
  }
  return false;
}

}  // namespace

void validate_lane_connection(const LaneConnectionConfig & config)
{
  const auto positive = [](double value) {return std::isfinite(value) && value > 0.0;};
  if (config.padding_px < 0 || config.padding_px > 300 || config.min_component_area_px < 1 ||
    config.max_fragments < 1 || config.max_fragments > 64 || config.line_width_px < 1 ||
    config.line_width_px > 10 || !positive(config.min_fragment_length_px) ||
    !positive(config.tangent_window_px) || !positive(config.max_gap_px) || config.max_gap_px > 600 ||
    !positive(config.corridor_half_width_px) || !positive(config.direction_tolerance_deg) ||
    config.direction_tolerance_deg >= 80 || !positive(config.max_turn_deg) ||
    config.max_turn_deg > 180 || !positive(config.max_curvature_per_px) ||
    !positive(config.max_arc_ratio) || config.max_arc_ratio < 1.0 ||
    !positive(config.min_lane_length_px))
  {throw std::invalid_argument("Invalid lane connection geometry/size parameters");}
}

LaneConnectionResult connect_lane_fragments(
  const cv::Mat & labels, const LaneConnectionConfig & config,
  const LaneSmoothingConfig & smoothing)
{
  if (labels.type() != CV_8UC1 || labels.empty()) {
    throw std::invalid_argument("Lane connector expects a nonempty mono8 label image");
  }
  LaneConnectionResult result;
  const cv::Size size(labels.cols + 2 * config.padding_px, labels.rows);
  std::array<cv::Mat, 2> masks;
  for (int side = 0; side < 2; ++side) {
    result.lanes[side] = select_chain(fragments(labels, side, config, smoothing),
      config, labels.cols, labels.rows);
    masks[side] = lane_mask(result.lanes[side], size, config.line_width_px);
  }
  if (pair_conflict(masks[0], masks[1])) {
    // Do not swap semantic channels. Similar support means the pair is ambiguous.
    const double left = result.lanes[0].observed_length_px;
    const double right = result.lanes[1].observed_length_px;
    for (int side = 0; side < 2; ++side) {
      const double own = side == 0 ? left : right;
      const double other = side == 0 ? right : left;
      if (own <= other * 1.1) {
        result.lanes[side] = ConnectedLane{};
        masks[side].setTo(0);
      }
    }
  }
  result.labels = cv::Mat::zeros(size, CV_8UC1);
  result.image = cv::Mat::zeros(size, CV_8UC3);
  for (int side = 0; side < 2; ++side) {
    if (result.lanes[side].points.empty()) {continue;}
    result.state |= static_cast<std::uint8_t>(1U << side);
    result.labels.setTo(side + 1, masks[side] == 1);
    result.labels.setTo(side + 3, masks[side] == 2);
    result.image.setTo(side == 0 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255), masks[side]);
  }
  return result;
}

}  // namespace line_detactor
