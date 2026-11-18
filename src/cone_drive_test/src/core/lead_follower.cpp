#include "cone_drive_test/core/lead_follower.hpp"
#include "cone_drive_test/core/duty_only.hpp"
#include <algorithm>
#include <cmath>

namespace cone_drive_test {

double lead_gap_along_path(const std::vector<Vec2> & path, const ConeObservation & v, const LeadParams & p) {
  if (path.size() < 2) return -1;
  std::vector<double> arc(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) arc[i] = arc[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  const double half = p.ego_half_width + p.lateral_margin;
  double best = -1;
  // 차량 사각형(차선 방향 정렬 가정)을 7×3 점으로 훑어 경로에 투영
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 7; ++j) {
      const Vec2 q{v.center.x - v.length / 2 + v.length * i / 2.0, v.center.y - v.width / 2 + v.width * j / 6.0};
      double bd = 1e18, bs = 0;
      for (std::size_t k = 0; k + 1 < path.size(); ++k) {
        const double dx = path[k + 1].x - path[k].x, dy = path[k + 1].y - path[k].y, L2 = dx * dx + dy * dy;
        if (L2 < 1e-12) continue;
        const double u = std::clamp(((q.x - path[k].x) * dx + (q.y - path[k].y) * dy) / L2, 0.0, 1.0);
        const double d = std::hypot(q.x - (path[k].x + u * dx), q.y - (path[k].y + u * dy));
        if (d < bd) { bd = d; bs = arc[k] + u * std::sqrt(L2); }
      }
      if (bd > half) continue;                       // 지나갈 폭 밖
      if (bs <= 1e-6 && q.x < path.front().x) continue;   // 경로 시작보다 뒤
      if (bs >= arc.back() - 1e-6) continue;         // 경로 끝 너머 (계획 거리 밖)
      const double g = bs - p.front_overhang;
      if (best < 0 || g < best) best = std::max(0.0, g);
    }
  return best;
}

double LeadFollower::cap_for(double gap, double v_lead, double v_ego) {
  const double v_eff = v_lead < p_.min_follow_speed ? 0.0 : v_lead;   // 너무 느린 앞차는 서 있는 것으로 (멈췄다 가기)
  if (v_eff <= 0.0 && p_.duty_only) {
    // 굴러서 서므로 정지 위치가 조금 짧을 수 있다: 정지 거리 +5 cm 안에서 멈추면 정지 유지, 더 짧으면 다시 다가감
    if (gap <= p_.stop_gap + 0.02 || (std::abs(v_ego) < 0.05 && gap <= p_.stop_gap + 0.05)) s_.holding = true;
    else if (gap >= p_.resume_gap) s_.holding = false;
    if (s_.holding) return 0.0;
    const double remaining = gap - p_.stop_gap;
    if (duty_only_coast_release(remaining, v_ego, p_.coast_decel, p_.release_delay, p_.release_speed + 0.05)) return 0.0;
    return duty_only_stop_speed_limit(remaining, p_.decel, p_.coast_decel, p_.release_speed, p_.response);
  }
  if (v_eff <= 0.0) {
    if (gap <= p_.stop_gap + 0.02) s_.holding = true;
    else if (gap >= p_.resume_gap) s_.holding = false;
    return s_.holding ? 0.0 : std::sqrt(2.0 * p_.decel * std::max(0.0, gap - p_.stop_gap));
  }
  s_.holding = false;
  const double target = p_.resume_gap + p_.time_gap * std::max(0.0, v_ego);
  if (gap >= target) return v_eff + std::sqrt(2.0 * p_.decel * (gap - target));
  return v_eff * std::clamp((gap - p_.stop_gap) / std::max(1e-3, target - p_.stop_gap), 0.0, 1.0);
}

const LeadState & LeadFollower::update(const std::vector<Vec2> & path, const std::vector<ConeObservation> & vehicles, bool new_det,
                                       double t, double dist, double v_ego) {
  if (!p_.enabled) { reset(); return s_; }
  s_.seen = false;
  if (new_det) {
    double g = -1;
    for (const auto & v : vehicles) {
      const double gv = lead_gap_along_path(path, v, p_);
      if (gv >= 0 && (g < 0 || gv < g)) g = gv;
    }
    if (g >= 0) {
      if (s_.active && have_meas_ && t - meas_t_ > 0.01 && std::abs(g - (meas_gap_ - (dist - meas_dist_))) < 0.5) {
        const double dt = t - meas_t_;
        const double raw = std::max(0.0, ((g - meas_gap_) + (dist - meas_dist_)) / dt);   // 앞차 이동 거리 / 시간
        const double a = dt / (p_.speed_tau + dt);
        s_.lead_speed += a * (raw - s_.lead_speed);
      } else {
        s_.lead_speed = 0.0;   // 처음 본(또는 갑자기 바뀐) 앞차는 서 있다고 가정 (보수적)
      }
      s_.active = true;
      s_.seen = true;
      s_.gap = g;
      meas_gap_ = g; meas_t_ = t; meas_dist_ = dist; have_meas_ = true;
    } else if (s_.active) {
      const double pred = meas_gap_ - (dist - meas_dist_) + s_.lead_speed * (t - meas_t_);
      if (!vehicles.empty()) {
        reset();                                   // 차량은 보이는데 경로 밖 → 비켜났음
      } else if (pred <= p_.hold_near) {
        s_.lead_speed = 0.0;                       // 가까이서 놓침(카메라 아래 사각): 다시 보일 때까지 서 있다고 가정
        meas_gap_ = pred; meas_t_ = t; meas_dist_ = dist;
      } else if (t - meas_t_ > p_.lost_timeout) {
        reset();                                   // 멀리서 놓침: 잠깐은 예측으로 유지, 넘으면 해제
      }
    }
  }
  if (s_.active) {
    s_.gap = std::max(0.0, meas_gap_ - (dist - meas_dist_) + s_.lead_speed * (t - meas_t_));
    s_.cap = cap_for(s_.gap, s_.lead_speed, v_ego);
  } else {
    s_.cap = std::numeric_limits<double>::infinity();
    s_.holding = false;
  }
  return s_;
}

}  // namespace cone_drive_test
