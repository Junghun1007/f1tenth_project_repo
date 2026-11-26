// Copied from src/cone_drive_test/src/core/path_utils.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/path_utils.hpp"
#include "auto_control/drive/core/frenet_planner.hpp"
#include <algorithm>
#include <cmath>

namespace auto_control::drive {

double interp(double x, const std::vector<double> & xp, const std::vector<double> & fp) {
  if (xp.empty()) return 0.0;
  if (x <= xp.front()) return fp.front();
  if (x >= xp.back()) return fp.back();
  const auto it = std::upper_bound(xp.begin(), xp.end(), x);
  const std::size_t i = std::size_t(it - xp.begin());
  const double x0 = xp[i - 1], x1 = xp[i];
  if (x1 - x0 <= 0) return fp[i];
  return fp[i - 1] + (fp[i] - fp[i - 1]) * (x - x0) / (x1 - x0);
}

SimPath::SimPath(const std::vector<Vec2> & pts, double w) : window(w) {
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (i > 0 && std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y) <= 1e-6) continue;
    points.push_back(pts[i]);
  }
  arc.assign(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i)
    arc[i] = arc[i - 1] + std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
}

SimPath::SimPath(const std::vector<Vec2> & pts, const std::vector<double> & a, double w) : points(pts), arc(a), window(w) {}

Vec2 SimPath::point(double a) const {
  if (points.empty()) return {};
  if (a <= arc.front()) return points.front();
  if (a >= arc.back()) return points.back();
  const auto it = std::upper_bound(arc.begin(), arc.end(), a);
  const std::size_t i = std::size_t(it - arc.begin());
  const double t = (a - arc[i - 1]) / std::max(arc[i] - arc[i - 1], 1e-15);
  return {points[i - 1].x + (points[i].x - points[i - 1].x) * t, points[i - 1].y + (points[i].y - points[i - 1].y) * t};
}

double SimPath::heading(double a) const {
  const double h = window / 2;
  double a0 = std::max(0.0, a - h);
  const double a1 = std::min(length(), a + h);
  if (a1 - a0 < 1e-6) a0 = std::max(0.0, a1 - 0.02);
  const Vec2 p0 = point(a0), p1 = point(a1);
  return std::atan2(p1.y - p0.y, p1.x - p0.x);
}

double sim_stanley(const SimPath & path, double speed, const ControlParams & c, double max_steer) {
  const auto & pts = path.points;
  std::size_t best_i = 0;
  double best_d2 = 1e300, best_t = 0, best_len = 1e-6, cx = 0, cy = 0;
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    const double dx = pts[i + 1].x - pts[i].x, dy = pts[i + 1].y - pts[i].y;
    const double len2 = std::max(dx * dx + dy * dy, 1e-12);
    const double t = std::clamp(-(pts[i].x * dx + pts[i].y * dy) / len2, 0.0, 1.0);
    const double qx = pts[i].x + dx * t, qy = pts[i].y + dy * t;
    const double d2 = qx * qx + qy * qy;
    if (d2 < best_d2) { best_d2 = d2; best_i = i; best_t = t; best_len = std::sqrt(len2); cx = qx; cy = qy; }
  }
  const double dx = pts[best_i + 1].x - pts[best_i].x, dy = pts[best_i + 1].y - pts[best_i].y;
  const double cte = (-dy * cx + dx * cy) / best_len;
  const double arc = path.arc[best_i] + best_t * best_len;
  const double heading_error = path.heading(std::min(path.length(), arc + c.heading_lookahead));
  double correction = std::atan2(c.stanley_gain * cte, std::abs(speed) + c.softening_speed);
  if (std::abs(heading_error) >= c.corner_heading_threshold_rad && heading_error * correction < 0) {
    const double lim = std::abs(heading_error) * std::min(std::max(c.corner_opposing_ratio, 0.0), 0.99);
    correction = std::clamp(correction, -lim, lim);
  }
  return std::clamp(heading_error + correction, -max_steer, max_steer);
}

std::vector<Vec2> smooth_center(const std::vector<Vec2> & c) {
  const int n = int(c.size());
  if (n < 5) return c;
  std::vector<Vec2> out(n);
  for (int i = 0; i < n; ++i) {
    double sx = 0, sy = 0;
    for (int k = -2; k <= 2; ++k) { const int j = std::clamp(i + k, 0, n - 1); sx += c[j].x; sy += c[j].y; }
    out[i] = {sx / 5, sy / 5};
  }
  return out;
}

std::vector<Vec2> extend_backward(const std::vector<Vec2> & c, double length, double spacing) {
  if (c.size() < 2) return c;
  double dx = c[1].x - c[0].x, dy = c[1].y - c[0].y;
  const double n0 = std::max(std::hypot(dx, dy), 1e-12);
  dx /= n0; dy /= n0;
  const int n = int(length / spacing);
  std::vector<Vec2> out;
  out.reserve(c.size() + n);
  for (int k = n; k >= 1; --k) out.push_back({c[0].x - dx * k * spacing, c[0].y - dy * k * spacing});
  out.insert(out.end(), c.begin(), c.end());
  return out;
}

std::vector<Vec2> extend_forward_arc(const std::vector<Vec2> & c, double length, double fit_length, double max_curvature,
                                     double spacing) {
  const std::size_t n = c.size();
  if (n < 3 || !(length > 0) || !(spacing > 0)) return c;
  auto heading = [&](std::size_t i) { return std::atan2(c[i + 1].y - c[i].y, c[i + 1].x - c[i].x); };
  // 끝에서 fit_length 앞 지점을 찾아 두 지점의 방향 차이 / 거리 = 평균 곡률
  std::size_t j = n - 2;
  double run = 0;
  while (j > 0 && run < fit_length) { run += std::hypot(c[j + 1].x - c[j].x, c[j + 1].y - c[j].y); --j; }
  const double h1 = heading(n - 2);
  double k = run > 1e-6 ? std::remainder(h1 - heading(j), 2 * M_PI) / run : 0.0;
  k = std::clamp(k, -max_curvature, max_curvature);
  std::vector<Vec2> out = c;
  double x = c.back().x, y = c.back().y, h = h1;
  // 곡률은 끝에서 0까지 선형으로 줄인다: 늘린 길이 L 동안 더 꺾는 각 = k·L/2 (그대로 이으면 k·L로 90°를 넘겨 되돌아온다)
  for (double s = spacing; s <= length + 1e-9; s += spacing) {
    const double km = k * (1.0 - (s - spacing / 2) / length);
    h += km * spacing / 2;   // 중점 방향으로 한 걸음
    x += spacing * std::cos(h);
    y += spacing * std::sin(h);
    h += km * spacing / 2;
    out.push_back({x, y});
  }
  return out;
}

FrenetFrame frenet_frame(const std::vector<Vec2> & center) {
  FrenetFrame f;
  const std::size_t n = center.size();
  f.center = center;
  f.arc.assign(n, 0.0);
  std::vector<Vec2> tang(n);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    tang[i] = {center[i + 1].x - center[i].x, center[i + 1].y - center[i].y};
    f.arc[i + 1] = f.arc[i] + std::hypot(tang[i].x, tang[i].y);
  }
  if (n >= 2) tang[n - 1] = tang[n - 2];
  auto unit = [](Vec2 v) { const double m = std::max(std::hypot(v.x, v.y), 1e-12); return Vec2{v.x / m, v.y / m}; };
  for (auto & t : tang) t = unit(t);
  f.tang = tang;
  for (std::size_t i = 1; i + 1 < n; ++i) f.tang[i] = unit({tang[i - 1].x + tang[i].x, tang[i - 1].y + tang[i].y});
  f.norm.resize(n);
  for (std::size_t i = 0; i < n; ++i) f.norm[i] = {-f.tang[i].y, f.tang[i].x};
  return f;
}

Vec2 project_sl(const Vec2 & p, const FrenetFrame & f) {
  std::size_t best = 0;
  double bd = 1e300;
  for (std::size_t i = 0; i < f.center.size(); ++i) {
    const double dx = f.center[i].x - p.x, dy = f.center[i].y - p.y, d = dx * dx + dy * dy;
    if (d < bd) { bd = d; best = i; }
  }
  const double vx = p.x - f.center[best].x, vy = p.y - f.center[best].y;
  return {f.arc[best] + vx * f.tang[best].x + vy * f.tang[best].y, vx * f.norm[best].x + vy * f.norm[best].y};
}

Vec2 from_frenet(double s, double l, const FrenetFrame & f) {
  const auto & a = f.arc;
  double x, y, tx, ty;
  if (s < a.front()) {
    x = f.center.front().x + (s - a.front()) * f.tang.front().x; y = f.center.front().y + (s - a.front()) * f.tang.front().y;
    tx = f.tang.front().x; ty = f.tang.front().y;
  } else if (s > a.back()) {
    x = f.center.back().x + (s - a.back()) * f.tang.back().x; y = f.center.back().y + (s - a.back()) * f.tang.back().y;
    tx = f.tang.back().x; ty = f.tang.back().y;
  } else {
    // 같은 arc 위의 선형 보간 (np.interp)
    const auto it = std::upper_bound(a.begin(), a.end(), s);
    std::size_t i = std::size_t(it - a.begin());
    if (i == 0) i = 1;
    if (i >= a.size()) i = a.size() - 1;
    const double w = (a[i] - a[i - 1]) > 0 ? (s - a[i - 1]) / (a[i] - a[i - 1]) : 1.0;
    auto lerp = [&](double u, double v) { return u + (v - u) * w; };
    x = lerp(f.center[i - 1].x, f.center[i].x); y = lerp(f.center[i - 1].y, f.center[i].y);
    tx = lerp(f.tang[i - 1].x, f.tang[i].x); ty = lerp(f.tang[i - 1].y, f.tang[i].y);
    const double m = std::hypot(tx, ty);
    tx /= m; ty /= m;
  }
  return {x - l * ty, y + l * tx};
}

}  // namespace auto_control::drive
