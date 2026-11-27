// 코너 경로 계획 (실험). tools/lane_path_lab/planner.js와 같은 알고리즘 — 둘을 함께 고칠 것.
#include "auto_control/drive/core/corner_planner.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace auto_control::drive {
namespace {
constexpr double kStep = 0.05;        // 계획 점 간격
constexpr double kFine = 0.01;        // 차체 궤적 검사 간격
constexpr double kRefSigma = 0.15;    // 기준선 평활화 폭
constexpr double kKnot = 0.30;        // 스플라인 제어점 간격
constexpr double kAlong = 0.04;       // 통로 폭을 잴 때 쓰는 차선 점의 전후 범위
constexpr double kMaxLateral = 1.0;
constexpr double kTarget = 0.02;      // 차체 여유 목표 (되먹임)
constexpr double kInf = std::numeric_limits<double>::infinity();

Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
double norm(Vec2 a) { return std::hypot(a.x, a.y); }
double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
Vec2 unit(Vec2 a) { const double n = norm(a); return n > 0 ? a * (1.0 / n) : Vec2{1, 0}; }

std::vector<Vec2> resample(const std::vector<Vec2> & P, double step) {
  if (P.size() < 2) return P;
  std::vector<Vec2> o{P[0]};
  double acc = 0;
  for (std::size_t i = 1; i < P.size(); ++i) {
    Vec2 a = P[i - 1];
    double l = norm(P[i] - a);
    while (acc + l >= step) {
      a = a + (P[i] - a) * ((step - acc) / l);
      o.push_back(a);
      l = norm(P[i] - a);
      acc = 0;
    }
    acc += l;
  }
  return o;
}

std::vector<double> gauss(const std::vector<double> & v, double s) {
  const int r = int(std::ceil(3 * s)), n = int(v.size());
  std::vector<double> o(v.size());
  for (int i = 0; i < n; ++i) {
    double a = 0, ws = 0;
    for (int j = -r; j <= r; ++j) {
      const int k = std::clamp(i + j, 0, n - 1);
      const double g = std::exp(-j * j / (2 * s * s));
      a += v[k] * g; ws += g;
    }
    o[i] = a / ws;
  }
  return o;
}

std::vector<double> curvature(const std::vector<Vec2> & P, double smooth) {
  const std::size_t N = P.size();
  std::vector<double> k(N, 0.0);
  for (std::size_t i = 1; i + 1 < N; ++i) {
    const Vec2 a = P[i] - P[i - 1], b = P[i + 1] - P[i], c = P[i + 1] - P[i - 1];
    k[i] = 2 * (a.x * b.y - a.y * b.x) / (norm(a) * norm(b) * norm(c) + 1e-9);
  }
  if (N > 1) { k[0] = k[1]; k[N - 1] = k[N - 2]; }
  return smooth > 0 ? gauss(k, smooth) : k;
}

struct Corridor {
  std::vector<Vec2> R, n;            // 기준선, 왼쪽 법선
  std::vector<double> dl, dr;        // 왼쪽·오른쪽 차선까지 거리
  std::vector<int> est;              // 0 관측, 1 왼쪽 추정, 2 오른쪽 추정, 3 둘 다 추정
};

Corridor corridor(std::vector<Vec2> base, const std::vector<Vec2> & left, const std::vector<Vec2> & right, double lane_width) {
  if (norm(base[0]) > 0.03) base.insert(base.begin(), Vec2{0, 0});   // 차량 위치에서 시작
  base = resample(base, kStep);
  // 끝점을 고정한 채 평활화 (양 끝을 점대칭으로 늘려서)
  const double s = kRefSigma / kStep;
  const int r = int(std::ceil(3 * s)), n0 = int(base.size());
  std::vector<double> xs, ys;
  for (int i = r; i >= 1; --i) { const Vec2 q = base[0] * 2 - base[std::min(n0 - 1, i)]; xs.push_back(q.x); ys.push_back(q.y); }
  for (const auto & q : base) { xs.push_back(q.x); ys.push_back(q.y); }
  for (int i = 1; i <= r; ++i) { const Vec2 q = base[n0 - 1] * 2 - base[std::max(0, n0 - 1 - i)]; xs.push_back(q.x); ys.push_back(q.y); }
  xs = gauss(xs, s); ys = gauss(ys, s);
  std::vector<Vec2> sm;
  for (int i = 0; i < n0; ++i) sm.push_back({xs[r + i], ys[r + i]});
  Corridor C;
  C.R = resample(sm, kStep);
  const std::size_t N = C.R.size();
  for (std::size_t i = 0; i < N; ++i) {
    const Vec2 t = unit(C.R[std::min(N - 1, i + 1)] - C.R[i == 0 ? 0 : i - 1]);
    C.n.push_back({-t.y, t.x});
    const auto side = [&](const std::vector<Vec2> & pts, double sign) {
      double best = kInf;
      for (const auto & q : pts) {
        const Vec2 d = q - C.R[i];
        if (std::abs(dot(d, t)) > kAlong) continue;
        const double lat = sign * dot(d, C.n[i]);
        if (lat >= 0 && lat < best) best = lat;
      }
      return best <= kMaxLateral ? best : kInf;
    };
    double l = side(left, 1), rr = side(right, -1);
    int e = 0;
    if (l == kInf && rr < kInf) { l = lane_width - rr; e = 1; }
    else if (rr == kInf && l < kInf) { rr = lane_width - l; e = 2; }
    else if (l == kInf && rr == kInf) { l = rr = lane_width / 2; e = 3; }
    C.dl.push_back(l); C.dr.push_back(rr); C.est.push_back(e);
  }
  return C;
}

struct Speeds { std::vector<double> v; bool over{false}; };
Speeds speed_profile(const std::vector<double> & kabs, double v_now, const CornerPlanParams & p) {
  const std::size_t N = kabs.size();
  std::vector<double> lim(N);
  for (std::size_t i = 0; i < N; ++i) lim[i] = std::min(std::max(v_now, 0.0), std::sqrt(p.lateral_accel / std::max(kabs[i], 1e-6)));
  for (std::size_t i = N - 1; i-- > 0;) lim[i] = std::min(lim[i], std::sqrt(lim[i + 1] * lim[i + 1] + 2 * p.deceleration * kStep));
  Speeds s;
  s.v.push_back(std::max(v_now, 0.0));
  for (std::size_t i = 1; i < N; ++i) {
    const double up = std::sqrt(s.v.back() * s.v.back() + 2 * p.acceleration * kStep);
    const double dn = std::sqrt(std::max(0.0, s.v.back() * s.v.back() - 2 * p.deceleration * kStep));
    const double x = std::max(dn, std::min(lim[i], up));
    if (x > lim[i] + 0.02) s.over = true;
    s.v.push_back(x);
  }
  return s;
}

struct Trace {
  std::vector<Vec2> rear;            // 슬립 반영 뒤축 중심
  std::vector<Vec2> heading;
  double min_clear{kInf}, min_radius{kInf}, apex_speed{0};
  bool over{false};
};

std::size_t nearest(const Corridor & C, Vec2 q) {
  std::size_t best = 0;
  double bd = kInf;
  for (std::size_t t = 0; t < C.R.size(); ++t) { const double d = norm(q - C.R[t]); if (d < bd) { bd = d; best = t; } }
  return best;
}

Trace trace(const std::vector<Vec2> & path, const Corridor & C, double v_now, const CornerPlanParams & p) {
  Trace T;
  const auto F = resample(path, kFine);
  const auto coarse = resample(path, kStep);
  const auto k = curvature(coarse, 1.5);
  std::vector<double> kabs(k.size());
  for (std::size_t i = 0; i < k.size(); ++i) kabs[i] = std::abs(k[i]);
  const auto sp = speed_profile(kabs, v_now, p);
  T.over = sp.over;
  Vec2 r{F[0].x - p.wheelbase, F[0].y};
  for (std::size_t i = 0; i < F.size(); ++i) {
    const Vec2 d = unit(F[i] - r);
    r = F[i] - d * p.wheelbase;
    const std::size_t c = std::min(k.size() - 1, std::size_t(std::lround(i * kFine / kStep)));
    const Vec2 lf{-d.y, d.x};
    const double vr = sp.v[c] / p.v0;
    const double slip = std::max(0.0, std::abs(k[c]) * p.wheelbase * p.wheelbase / 2 * vr * vr);
    const Vec2 rs = r + lf * (-(k[c] > 0 ? 1.0 : k[c] < 0 ? -1.0 : 0.0) * slip);   // 슬립: 뒤축이 회전 바깥쪽으로
    const Vec2 front = F[i] + d * p.front_overhang;
    const std::array<Vec2, 4> corners{rs + lf * (p.vehicle_width / 2), rs - lf * (p.vehicle_width / 2),
                                      front + lf * (p.vehicle_width / 2), front - lf * (p.vehicle_width / 2)};
    for (const auto & q : corners) {
      const std::size_t j = nearest(C, q);
      const double lat = dot(q - C.R[j], C.n[j]);
      if (C.est[j] != 1 && C.est[j] != 3) T.min_clear = std::min(T.min_clear, C.dl[j] - lat);   // 관측한 쪽만 판정
      if (C.est[j] != 2 && C.est[j] != 3) T.min_clear = std::min(T.min_clear, C.dr[j] + lat);
    }
    T.rear.push_back(rs);
    T.heading.push_back(d);
  }
  const auto G = resample(path, 0.02);
  double kmax = 0;
  for (std::size_t i = 12; i + 12 < G.size(); ++i) {
    const Vec2 a = G[i - 12], b = G[i], c = G[i + 12];
    const double den = norm(b - a) * norm(c - b) * norm(a - c);
    if (den > 1e-9) kmax = std::max(kmax, std::abs(2 * ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))) / den);
  }
  T.min_radius = kmax > 0 ? 1 / kmax : kInf;
  std::size_t ia = 0;
  for (std::size_t i = 0; i < k.size(); ++i) if (std::abs(k[i]) > std::abs(k[ia])) ia = i;
  T.apex_speed = sp.v.empty() ? 0 : sp.v[ia];
  return T;
}

struct Basis { int m; std::array<double, 4> w; };

std::vector<double> eval(const std::vector<Basis> & B, const std::vector<double> & c) {
  std::vector<double> d(B.size());
  for (std::size_t i = 0; i < B.size(); ++i)
    d[i] = B[i].w[0] * c[B[i].m] + B[i].w[1] * c[B[i].m + 1] + B[i].w[2] * c[B[i].m + 2] + B[i].w[3] * c[B[i].m + 3];
  return d;
}

// 통로 [lo, hi] 안에서 곡률 제곱합이 최소인 횡 이동량 (제어점 패턴 탐색)
std::vector<double> optimize(const Corridor & C, const std::vector<double> & lo, const std::vector<double> & hi,
                             const std::vector<Basis> & B, std::vector<double> & ctrl) {
  const std::size_t N = C.R.size();
  std::vector<Vec2> Q(N);
  const auto cost = [&](const std::vector<double> & c) {
    const auto d = eval(B, c);
    for (std::size_t i = 0; i < N; ++i) Q[i] = C.R[i] + C.n[i] * d[i];
    double J = 0;
    for (std::size_t i = 1; i + 1 < N; ++i) {
      const Vec2 a = Q[i] - Q[i - 1], b = Q[i + 1] - Q[i], e = Q[i + 1] - Q[i - 1];
      const double k = 2 * (a.x * b.y - a.y * b.x) / (norm(a) * norm(b) * norm(e) + 1e-9);
      J += k * k * kStep;
    }
    for (std::size_t i = 0; i < N; ++i) {
      const double v = std::max(0.0, lo[i] - d[i]) + std::max(0.0, d[i] - hi[i]);
      J += 1e5 * v * v + 0.2 * d[i] * d[i] * kStep;
    }
    return J + 1e5 * (d[0] * d[0] + (d[1] - d[0]) * (d[1] - d[0]));
  };
  double J = cost(ctrl);
  for (double h = 0.08; h > 0.0008; h /= 2) {
    for (int sweep = 0; sweep < 40; ++sweep) {
      bool improved = false;
      for (std::size_t m = 0; m < ctrl.size(); ++m) {
        for (const double s : {h, -h}) {
          ctrl[m] += s;
          const double j = cost(ctrl);
          if (j < J - 1e-9) { J = j; improved = true; break; }
          ctrl[m] -= s;
        }
      }
      if (!improved) break;
    }
  }
  return eval(B, ctrl);
}
}  // namespace

CornerPlanResult plan_corner_path(const std::vector<Vec2> & reference, const std::vector<Vec2> & left,
                                  const std::vector<Vec2> & right, double speed, const CornerPlanParams & p) {
  CornerPlanResult out;
  if (reference.size() < 4 || !(p.v0 > 0) || !(p.wheelbase > 0)) return out;
  for (const auto & q : reference) if (!std::isfinite(q.x) || !std::isfinite(q.y)) return out;
  const auto C = corridor(reference, left, right, p.lane_width);
  const std::size_t N = C.R.size();
  if (N < 4) return out;
  const int nC = int(std::ceil((N - 1) * kStep / kKnot)) + 3;
  std::vector<Basis> B(N);
  for (std::size_t i = 0; i < N; ++i) {
    const double u = i * kStep / kKnot;
    const int m = std::min(int(std::floor(u)), nC - 4);
    const double t = u - m;
    B[i] = {m, {(1 - t) * (1 - t) * (1 - t) / 6, (3 * t * t * t - 6 * t * t + 4) / 6,
                (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6, t * t * t / 6}};
  }
  std::vector<double> ctrl(nC, 0.0), exL(N, 0.0), exR(N, 0.0), lo(N), hi(N);
  const int nL = int(std::lround(p.wheelbase / kStep));
  std::vector<Vec2> path = C.R;
  Trace T;
  for (int loop = 0; loop < 4; ++loop) {
    const auto k = curvature(path, 1.5);
    std::vector<double> kabs(N);
    for (std::size_t i = 0; i < N; ++i) kabs[i] = std::abs(k[i]);
    const auto sp = speed_profile(kabs, speed, p);
    std::vector<double> off(N);
    for (std::size_t i = 0; i < N; ++i) { const double vr = sp.v[i] / p.v0; off[i] = kabs[i] * p.wheelbase * p.wheelbase / 2 * (1 - vr * vr); }
    off = gauss(off, 2);
    out.infeasible = false;
    for (std::size_t i = 0; i < N; ++i) {
      const bool turning = kabs[i] > 0.3, left_inner = k[i] > 0;
      const double inner = turning ? p.inner_margin + std::max(0.0, off[i]) : p.outer_margin;
      const double outer = p.outer_margin + (turning ? std::max(0.0, -off[i]) : 0.0);
      const double mL = left_inner ? inner : outer, mR = left_inner ? outer : inner;
      hi[i] = C.dl[i] - (p.vehicle_width / 2 + mL + exL[i]);
      lo[i] = -(C.dr[i] - (p.vehicle_width / 2 + mR + exR[i]));
      if (lo[i] > hi[i]) {
        out.infeasible = true;
        lo[i] = hi[i] = std::min(C.dl[i] - p.vehicle_width / 2, std::max(-(C.dr[i] - p.vehicle_width / 2), (lo[i] + hi[i]) / 2));
      }
    }
    lo[0] = std::min(lo[0], 0.0); hi[0] = std::max(hi[0], 0.0);
    const auto d = optimize(C, lo, hi, B, ctrl);
    for (std::size_t i = 0; i < N; ++i) path[i] = C.R[i] + C.n[i] * d[i];
    T = trace(path, C, speed, p);
    if (loop == 3 || T.min_clear >= kTarget) break;
    // 뒷바퀴 여유가 모자란 곳: 그 지점부터 축거만큼 앞까지 통로를 좁혀 다시 맞춘다
    std::vector<double> defL(N, 0.0), defR(N, 0.0);
    for (std::size_t i = 0; i < T.rear.size(); i += 2) {
      const Vec2 lf{-T.heading[i].y, T.heading[i].x};
      for (const Vec2 q : {T.rear[i] + lf * (p.vehicle_width / 2), T.rear[i] - lf * (p.vehicle_width / 2)}) {
        const std::size_t j = nearest(C, q);
        const double lat = dot(q - C.R[j], C.n[j]);
        if (C.est[j] != 1 && C.est[j] != 3) defL[j] = std::max(defL[j], kTarget - (C.dl[j] - lat));
        if (C.est[j] != 2 && C.est[j] != 3) defR[j] = std::max(defR[j], kTarget - (C.dr[j] + lat));
      }
    }
    for (std::size_t t = 0; t < N; ++t) {
      double a = 0, b = 0;
      for (std::size_t u = t >= std::size_t(nL) ? t - nL : 0; u <= t; ++u) { a = std::max(a, defL[u]); b = std::max(b, defR[u]); }
      exL[t] = std::min(0.15, exL[t] + a);
      exR[t] = std::min(0.15, exR[t] + b);
    }
  }
  for (const auto & q : path) if (!std::isfinite(q.x) || !std::isfinite(q.y)) return out;
  int est = 0;
  for (int e : C.est) est += e != 0;
  out.ok = true;
  out.path = std::move(path);
  out.min_radius = T.min_radius;
  out.min_clearance = T.min_clear;
  out.apex_speed = T.apex_speed;
  out.estimated_fraction = double(est) / N;
  out.decel_short = T.over;
  return out;
}

}  // namespace auto_control::drive
