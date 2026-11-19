#include "semi_slam/core/track_map.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace semi_slam {

namespace {
double length_closed(const std::vector<Vec2> & p) {
  double L = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    const auto & a = p[i];
    const auto & b = p[(i + 1) % p.size()];
    L += std::hypot(b.x - a.x, b.y - a.y);
  }
  return L;
}
}  // namespace

TrackMap TrackMap::load_csv(const std::string & path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open track map " + path);
  TrackMap m;
  std::vector<Vec2> stop, start;
  std::string line;
  std::getline(in, line);   // 헤더
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string name, xs, ys;
    if (!std::getline(ss, name, ',') || !std::getline(ss, xs, ',') || !std::getline(ss, ys, ',')) continue;
    const Vec2 p{std::stod(xs), std::stod(ys)};
    if (name == "outer") m.outer.push_back(p);
    else if (name == "inner") m.inner.push_back(p);
    else if (name == "center") m.center.push_back(p);
    else if (name == "stop_S") stop.push_back(p);
    else if (name == "start_T1") start.push_back(p);
  }
  if (m.outer.size() < 8 || m.inner.size() < 8 || stop.size() != 2 || start.size() != 2)
    throw std::runtime_error("track map needs outer, inner, stop_S(2), start_T1(2): " + path);
  m.stop_a = stop[0]; m.stop_b = stop[1];
  m.start_a = start[0]; m.start_b = start[1];
  double sx = 0, sy = 0;
  for (const auto & p : m.inner) { sx += p.x; sy += p.y; }
  m.middle_ = {sx / double(m.inner.size()), sy / double(m.inner.size())};
  m.lap_length_ = 0.5 * (length_closed(m.outer) + length_closed(m.inner));
  if (m.center.size() < 8) {   // 중앙선이 없으면 바깥 선 점마다 가장 가까운 안쪽 선과의 가운데로 만든다
    m.center.clear();
    for (const auto & o : m.outer) {
      Vec2 best = m.inner[0];
      double bd = 1e9;
      for (const auto & q : m.inner) { const double d = std::hypot(q.x - o.x, q.y - o.y); if (d < bd) { bd = d; best = q; } }
      m.center.push_back({(o.x + best.x) / 2, (o.y + best.y) / 2});
    }
  }
  {   // 반시계 순서로 맞추고 (넓이 부호) 누적 거리
    double area = 0;
    for (std::size_t i = 0; i < m.center.size(); ++i) {
      const auto & a = m.center[i];
      const auto & b = m.center[(i + 1) % m.center.size()];
      area += a.x * b.y - b.x * a.y;
    }
    if (area < 0) std::reverse(m.center.begin(), m.center.end());
    m.center_s_.assign(m.center.size(), 0.0);
    for (std::size_t i = 1; i < m.center.size(); ++i)
      m.center_s_[i] = m.center_s_[i - 1] + std::hypot(m.center[i].x - m.center[i - 1].x, m.center[i].y - m.center[i - 1].y);
    m.center_len_ = length_closed(m.center);
  }
  m.build_fields();
  m.lane_width_ = m.dist_inner(m.outer[0]);   // 아래 변 가운데: 바깥 선에서 안쪽 선까지
  return m;
}

void TrackMap::build_fields(double res) {
  res_ = res;
  double xmax = -1e9, ymax = -1e9;
  x0_ = y0_ = 1e9;
  for (const auto * line : {&outer, &inner})
    for (const auto & p : *line) { x0_ = std::min(x0_, p.x); y0_ = std::min(y0_, p.y); xmax = std::max(xmax, p.x); ymax = std::max(ymax, p.y); }
  x0_ -= 0.5; y0_ -= 0.5; xmax += 0.5; ymax += 0.5;
  const int w = int(std::ceil((xmax - x0_) / res_)) + 1, h = int(std::ceil((ymax - y0_) / res_)) + 1;
  auto make = [&](const std::vector<Vec2> & line) {
    cv::Mat img(h, w, CV_8UC1, cv::Scalar(255));
    std::vector<cv::Point> pts;
    for (const auto & p : line) pts.emplace_back(int(std::lround((p.x - x0_) / res_)), int(std::lround((p.y - y0_) / res_)));
    cv::polylines(img, pts, true, cv::Scalar(0), 1);
    cv::Mat dist;
    cv::distanceTransform(img, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    return cv::Mat(dist * res_);
  };
  outer_field_ = make(outer);
  inner_field_ = make(inner);
}

double TrackMap::field(const cv::Mat & f, const Vec2 & p) const {
  if (f.empty()) return std::numeric_limits<double>::infinity();
  const int c = int(std::lround((p.x - x0_) / res_)), r = int(std::lround((p.y - y0_) / res_));
  if (r < 0 || c < 0 || r >= f.rows || c >= f.cols) return 1e3;
  return f.at<float>(r, c);
}

double TrackMap::wrap_s(double s) const {
  if (center_len_ <= 0) return s;
  s = std::fmod(s, center_len_);
  return s < 0 ? s + center_len_ : s;
}

double TrackMap::arc_diff(double a, double b) const {
  double d = wrap_s(a - b);
  if (d > 0.5 * center_len_) d -= center_len_;
  return d;
}

Vec2 TrackMap::center_at(double s, Vec2 * tangent) const {
  const std::size_t n = center.size();
  if (n < 2) return {};
  s = wrap_s(s);
  std::size_t i = std::size_t(std::upper_bound(center_s_.begin(), center_s_.end(), s) - center_s_.begin());
  i = i == 0 ? 0 : i - 1;
  const Vec2 & a = center[i], & b = center[(i + 1) % n];
  const double seg = std::max(1e-9, std::hypot(b.x - a.x, b.y - a.y));
  const double u = std::clamp((s - center_s_[i]) / seg, 0.0, 1.0);
  if (tangent) *tangent = {(b.x - a.x) / seg, (b.y - a.y) / seg};
  return {a.x + u * (b.x - a.x), a.y + u * (b.y - a.y)};
}

double TrackMap::center_project(const Vec2 & p) const {
  const std::size_t n = center.size();
  double best = 1e18, best_s = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 & a = center[i], & b = center[(i + 1) % n];
    const double dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy;
    const double u = L2 > 0 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / L2, 0.0, 1.0) : 0.0;
    const double ex = a.x + u * dx - p.x, ey = a.y + u * dy - p.y, d = ex * ex + ey * ey;
    if (d < best) { best = d; best_s = center_s_[i] + u * std::sqrt(L2); }
  }
  return wrap_s(best_s);
}

Vec2 TrackMap::straight_start(bool cw) const {
  // 출발선 T1에서 반시계 진행 반대로 1.03 m = 왼쪽 아래 코너가 끝나는 x ≈ 1.0 m. 시계는 트랙 가운데 기준 좌우 대칭 (x ≈ 3.0 m)
  const Vec2 T = start_center(), d = travel_dir_at(T, false);
  Vec2 b{T.x - 1.03 * d.x, T.y - 1.03 * d.y};
  if (cw) b.x = 2.0 * middle_.x - b.x;
  return b;
}

Vec2 TrackMap::travel_dir_at(const Vec2 & p, bool cw) const {
  // 반시계 주행: 트랙 중심에서 p로 향하는 방향을 왼쪽으로 90° 돌린 것 (정사각형 트랙의 직선 구간에서는 축 방향으로 맞춤)
  const double dx = p.x - middle_.x, dy = p.y - middle_.y;
  Vec2 d{-dy, dx};
  if (std::abs(d.x) > std::abs(d.y)) d = {d.x > 0 ? 1.0 : -1.0, 0.0};
  else d = {0.0, d.y > 0 ? 1.0 : -1.0};
  return cw ? Vec2{-d.x, -d.y} : d;
}

}  // namespace semi_slam
