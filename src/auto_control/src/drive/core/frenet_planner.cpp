// Copied from src/cone_drive_test/src/core/frenet_planner.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/frenet_planner.hpp"
#include "auto_control/drive/core/path_utils.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_control::drive {

namespace {
constexpr double kInf = 1e9;
int arange_count(double start, double stop, double step) { return std::max(0, int(std::ceil((stop - start) / step))); }
}  // namespace

std::optional<FrenetPath> frenet_dp(const PlannerParams & p, double l0, Vec2 rear_sl, const std::vector<Vec2> & cones_sl,
                                    double s_end, double margin_extra, bool lane_hard, bool relaxed,
                                    const DesiredOffset & l_des, const LanePin * pin) {
  const double ds = lane_hard ? p.ds_lane : p.ds, dl = lane_hard ? p.dl_lane : p.dl, l_max = lane_hard ? p.l_max_lane : p.l_max;
  const int ns = arange_count(0.0, s_end + 1e-9, ds);
  if (ns < 4) return std::nullopt;
  const int nl = arange_count(-l_max, l_max + 1e-9, dl);
  std::vector<double> ss(ns), L(nl);
  for (int i = 0; i < ns; ++i) ss[i] = i * ds;
  for (int k = 0; k < nl; ++k) L[k] = -l_max + k * dl;
  const double hl = p.length / 2, hw = p.width / 2, hc = p.cone_half;
  const double center_off = p.length / 2 - p.rear_overhang - p.wheelbase;   // 앞차축 → 차체 중심 (음수)
  double hard, soft;
  if (relaxed) { hard = p.relaxed_hard_clearance; soft = p.relaxed_soft_clearance; }
  else if (lane_hard) { hard = p.hard_clearance; soft = p.soft_clearance; }
  else { hard = p.free_hard_clearance; soft = p.free_soft_clearance; }
  hard += margin_extra;
  soft += margin_extra;
  // 앞차축 뒤(과거) 경로: 앞차축~뒷차축 직선 외삽
  const double hist_slope = (l0 - rear_sl.y) / std::max(-rear_sl.x, 1e-6);
  const double cur_excess = std::max(0.0, std::max(std::abs(l0), std::abs(rear_sl.y)) + hw - p.lane_half);
  const double w_lin = lane_hard ? p.w_lane_linear : p.w_lane_linear_free;
  const int n_sub = 4;
  const int win = int(std::ceil(1.2 * ds / dl)) + 1;   // |new_l - prev_l| ≤ 1.2·ds 인 격자 칸 수 (+1 여유, 판정은 아래 조건이 그대로 함)
  const bool pinned = pin && pin->s.size() >= 2 && pin->end_s > 0;
  const double pin_off0 = pinned ? std::abs(l0 - interp(0.0, pin->s, pin->l)) : 0.0;   // 지금 고정 경로에서 벗어난 만큼은 0.5 m에 걸쳐 허용

  std::vector<double> cost(nl, kInf), next_cost(nl), total(std::size_t(nl) * nl), lr(std::size_t(nl) * nl), node(std::size_t(nl) * nl);
  std::vector<int> back(std::size_t(ns) * nl, 0);
  std::vector<double> rear(std::size_t(ns) * nl, rear_sl.y);
  int i0 = 0;
  for (int k = 1; k < nl; ++k) if (std::abs(L[k] - l0) < std::abs(L[i0] - l0)) i0 = k;
  cost[i0] = 0.0;
  std::vector<Vec2> sel;
  for (int i = 1; i < ns; ++i) {
    sel.clear();
    for (const auto & c : cones_sl) if (c.x >= ss[i] - 0.8 && c.x <= ss[i] + 0.4) sel.push_back(c);
    double ld = 0.0;
    if (!l_des.s.empty()) ld = interp(ss[i], l_des.s, l_des.l);
    const bool pin_here = pinned && ss[i] <= pin->end_s;
    const double pin_l = pin_here ? interp(ss[i], pin->s, pin->l) : 0.0;
    const double pin_tol = pin_here ? pin->tol + pin_off0 * std::max(0.0, 1.0 - ss[i] / 0.5) : 0.0;
    for (int a = 0; a < nl; ++a) {   // pred
      if (cost[a] >= kInf) continue;   // 도달 불가능한 이전 노드: 아래 최소 탐색에서도 건너뜀
      const double prev_l = L[a];
      const double pprev = i >= 2 ? L[back[std::size_t(i - 1) * nl + a]] : l0 + hist_slope * (-ds);
      for (int b = std::max(0, a - win); b <= std::min(nl - 1, a + win); ++b) {  // new: 기울기 한계 안쪽만
        const double new_l = L[b];
        const std::size_t idx = std::size_t(a) * nl + b;
        if (pin_here && std::abs(new_l - pin_l) > pin_tol) { node[idx] = kInf; continue; }
        // 결과가 어차피 무한 비용인 쌍은 계산 생략 (기울기 한계 초과, 도달 불가능한 이전 노드). 최적 경로는 그대로
        if (cost[a] >= kInf || std::abs(new_l - prev_l) > 1.2 * ds) { node[idx] = kInf; continue; }
        // 뒷차축을 앞차축에 끌려가는 곡선(tractrix)으로 적분 (유한 비용 쌍만 필요)
        double r = rear[std::size_t(i - 1) * nl + a];
        for (int m = 1; m <= n_sub; ++m) {
          const double lf = prev_l + (new_l - prev_l) * (double(m) / n_sub);
          r = r + (lf - r) * (ds / n_sub) / p.wheelbase;
        }
        lr[idx] = r;
        const double curv = (new_l - 2 * prev_l + pprev) / (ds * ds);
        const double yaw = std::atan2(new_l - r, p.wheelbase);
        const double cs = std::cos(yaw), sn = std::sin(yaw);
        const double cy = new_l + sn * center_off, cx = ss[i] + cs * center_off;
        double nd = 0.0;
        if (!sel.empty()) {
          double gmin = std::numeric_limits<double>::infinity(), pen = 0.0;
          for (const auto & c : sel) {
            const double dx = c.x - cx, dy = c.y - cy;
            const double lx = std::abs(cs * dx + sn * dy) - hl, ly = std::abs(-sn * dx + cs * dy) - hw;
            double g = (lx > 0 || ly > 0) ? std::hypot(std::max(lx, 0.0), std::max(ly, 0.0)) : std::max(lx, ly);
            g -= hc;
            const double e = std::max(0.0, soft - g);
            pen += e * e;
            gmin = std::min(gmin, g);
          }
          nd += p.w_obstacle * pen;
          if (gmin < hard) nd = kInf;
        }
        const double edge = std::abs(cy) + std::abs(sn) * hl + cs * hw;
        const double excess = std::max(0.0, edge - p.lane_half);
        const double q = std::min(excess, p.lane_quad_cap);
        nd += p.w_lane * q * q + w_lin * excess;
        // 차선 안 모드는 lane_tolerance, 차선 밖 모드도 free_max_excess까지만 (이미 나가 있으면 복귀 거리에 걸쳐 허용)
        const double tol = (lane_hard ? p.lane_tolerance : p.free_max_excess) + cur_excess * std::max(0.0, 1.0 - ss[i] / p.lane_return_distance);
        if (excess > tol) nd = kInf;
        nd += p.w_offset * (new_l - ld) * (new_l - ld) * ds;
        const double slope = (new_l - prev_l) / ds;
        nd += p.w_slope * slope * slope * ds;
        nd += p.w_curvature * curv * curv * ds;
        if (std::abs(curv) > p.max_curvature) nd = kInf;
        if (std::abs(slope) > 1.2) nd = kInf;
        node[idx] = nd;
      }
    }
    for (int b = 0; b < nl; ++b) {
      int best = 0;
      double bv = 2 * kInf;   // 유한한 후보가 없으면 도달 불가 (값만 kInf 이상이면 됨)
      for (int a = std::max(0, b - win); a <= std::min(nl - 1, b + win); ++a) {
        if (cost[a] >= kInf) continue;
        const double v = cost[a] + node[std::size_t(a) * nl + b];
        if (v < bv) { bv = v; best = a; }
      }
      back[std::size_t(i) * nl + b] = best;
      next_cost[b] = bv;
      rear[std::size_t(i) * nl + b] = lr[std::size_t(best) * nl + b];
    }
    cost.swap(next_cost);
  }
  int j = 0;
  for (int k = 1; k < nl; ++k) if (cost[k] < cost[j]) j = k;
  if (!std::isfinite(cost[j]) || cost[j] >= kInf) return std::nullopt;
  std::vector<int> idx(ns);
  idx[ns - 1] = j;
  for (int i = ns - 1; i > 0; --i) { j = back[std::size_t(i) * nl + j]; idx[i - 1] = j; }
  std::vector<double> ll(ns);
  for (int i = 0; i < ns; ++i) ll[i] = L[idx[i]];
  ll[0] = l0;
  FrenetPath out;
  out.s = ss;
  out.l = ll;
  for (int i = 1; i + 1 < ns; ++i) out.l[i] = 0.25 * ll[i - 1] + 0.5 * ll[i] + 0.25 * ll[i + 1];   // 가벼운 평활화
  out.l[0] = l0;
  return out;
}

double rect_squares_min_gap(double cx, double cy, double yaw, double half_len, double half_wid,
                            const std::vector<Vec2> & centers, double half_sq) {
  const double cs = std::cos(yaw), sn = std::sin(yaw), ac = std::abs(cs), as = std::abs(sn);
  const double r_sq = half_sq * (ac + as);
  double best = std::numeric_limits<double>::infinity();
  for (const auto & c : centers) {
    const double dx = c.x - cx, dy = c.y - cy;
    const double g1 = std::abs(dx * cs + dy * sn) - half_len - r_sq;
    const double g2 = std::abs(-dx * sn + dy * cs) - half_wid - r_sq;
    const double g3 = std::abs(dx) - (half_len * ac + half_wid * as) - half_sq;
    const double g4 = std::abs(dy) - (half_len * as + half_wid * ac) - half_sq;
    best = std::min(best, std::max(std::max(g1, g2), std::max(g3, g4)));
  }
  return best;
}

RolloutResult rollout_ok(const PlannerParams & p, const ControlParams & c, const std::vector<Vec2> & path_local,
                         const std::vector<Vec2> & cones_local, double speed, double steer) {
  if (cones_local.empty()) return {true, 1e9};
  if (path_local.size() < 2) return {false, 1e9};
  const SimPath base(path_local, c.geometry_window);
  double rx = -p.wheelbase, ry = 0.0, psi = 0.0;
  const double v = std::min(std::max(speed, 0.3), c.max_speed);
  double delta = steer, cmd = steer;
  const double dt = 0.01, rate = p.steer_rate_rad_s * dt;
  const double off = p.length / 2 - p.rear_overhang;
  const int n_lat = int(c.latency / dt);
  const int steps = int(p.rollout_sec / dt);
  double best = 1e9;
  SimPath moved(base.points, base.arc, c.geometry_window);   // 매 스텝 점만 덮어써서 재사용 (할당·복사 없음)
  for (int k = 0; k < steps; ++k) {
    if (k >= n_lat) {
      const double fx = rx + p.wheelbase * std::cos(psi), fy = ry + p.wheelbase * std::sin(psi);
      const double cps = std::cos(psi), sps = std::sin(psi);
      for (std::size_t m = 0; m < base.points.size(); ++m) {
        const double dx = base.points[m].x - fx, dy = base.points[m].y - fy;
        moved.points[m] = {cps * dx + sps * dy, -sps * dx + cps * dy};
      }
      const double raw = sim_stanley(moved, v, c, p.max_steer_rad);
      cmd = c.steering_current_weight * raw + (1 - c.steering_current_weight) * cmd;
    }
    delta += std::max(-rate, std::min(rate, cmd - delta));
    rx += v * std::cos(psi) * dt;
    ry += v * std::sin(psi) * dt;
    psi += v / p.wheelbase * std::tan(delta) * dt;
    if (k % 2) continue;
    const double bx = rx + off * std::cos(psi), by = ry + off * std::sin(psi);
    const double g = rect_squares_min_gap(bx, by, psi, p.length / 2, p.width / 2, cones_local, p.cone_half);
    best = std::min(best, g);
    if (g < 0.0) return {false, g};
  }
  return {true, best};
}

}  // namespace auto_control::drive
