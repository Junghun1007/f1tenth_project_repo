#include "semi_slam/core/lane_side_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace semi_slam {

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

std::vector<float> profile(const std::vector<const std::vector<cv::Point2f> *> & frags, int rows) {   // 행별 평균 열
  std::vector<float> sum(rows, 0.f), cnt(rows, 0.f);
  for (const auto * f : frags)
    for (const auto & p : *f) {
      const int r = int(std::lround(p.y));
      if (r >= 0 && r < rows) { sum[r] += p.x; cnt[r] += 1.f; }
    }
  for (int r = 0; r < rows; ++r) sum[r] = cnt[r] > 0 ? sum[r] / cnt[r] : kNaN;
  return sum;
}

// 조각 점들과 기준 행별 열의 평균 (기준 - 조각) 열 차이. 겹치는 점이 min_n 미만이면 NaN
double mean_offset(const std::vector<cv::Point2f> & f, const std::vector<float> & ref, int min_n = 4) {
  double s = 0;
  int n = 0;
  for (std::size_t i = 0; i < f.size(); i += 2) {
    const int r = int(std::lround(f[i].y));
    if (r < 0 || r >= int(ref.size()) || std::isnan(ref[r])) continue;
    s += ref[r] - f[i].x;
    ++n;
  }
  return n >= min_n ? s / n : std::numeric_limits<double>::quiet_NaN();
}
}  // namespace

LaneSideFixStats LaneSideTracker::apply(Paths & paths, int rows, double t) {
  LaneSideFixStats st;
  const double gate = c_.match_gate_m / c_.meter_per_pixel;
  const double lw = c_.lane_width_m / c_.meter_per_pixel, tol = c_.width_tolerance_m / c_.meter_per_pixel;
  struct Frag { const std::vector<cv::Point2f> * pts; int model; int side; double evidence; };   // side -1 = 미정
  std::vector<Frag> fr;
  for (int s = 0; s < 2; ++s)
    for (const auto & f : paths[s]) if (!f.empty()) fr.push_back({&f, s, -1, 1e9});
  const bool prev_ok[2] = {t - prev_t_[0] <= c_.max_age_sec && !prev_[0].empty(), t - prev_t_[1] <= c_.max_age_sec && !prev_[1].empty()};

  // 1) 직전 프레임 기준
  for (auto & f : fr) {
    double best = gate;
    for (int s = 0; s < 2; ++s) {
      if (!prev_ok[s]) continue;
      const double d = std::abs(mean_offset(*f.pts, prev_[s]));
      if (d <= best) { best = d; f.side = s; f.evidence = d; }   // NaN은 비교에서 탈락
    }
  }
  // 2) 기준 없는 조각: 이번 프레임에 확정된 선과의 거리로 (확정된 선이 없으면 모델 라벨)
  bool any = false;
  for (const auto & f : fr) any |= f.side >= 0;
  for (auto & f : fr) {
    if (f.side >= 0) continue;
    if (!any) { f.side = f.model; continue; }
    for (int s = 0; s < 2 && f.side < 0; ++s) {
      std::vector<const std::vector<cv::Point2f> *> ref;
      for (const auto & g : fr) if (g.side == s && g.evidence < 1e8) ref.push_back(g.pts);
      if (ref.empty()) continue;
      const double off = mean_offset(*f.pts, profile(ref, rows));   // 확정선 열 - 조각 열 (양수 = 조각이 왼쪽)
      if (std::isnan(off)) continue;
      if (std::abs(off) <= gate) f.side = s;                                           // 같은 선
      else if (std::abs(std::abs(off) - lw) <= tol) f.side = off > 0 ? 0 : 1;          // 차선 폭만큼 떨어진 반대편
      else { f.side = 2; ++st.dropped; }                                                // 옆 차선 등 → 제외
    }
    if (f.side < 0) f.side = f.model;   // 겹치는 구간이 없어 비교 불가
  }
  // 3) 폭 검사
  for (int iter = 0; iter < 2; ++iter) {
    std::vector<const std::vector<cv::Point2f> *> side_frags[2];
    for (const auto & f : fr) if (f.side == 0 || f.side == 1) side_frags[f.side].push_back(f.pts);
    if (side_frags[0].empty() || side_frags[1].empty()) break;
    const auto L = profile(side_frags[0], rows), R = profile(side_frags[1], rows);
    int both = 0, bad = 0;
    for (int r = 0; r < rows; ++r) {
      if (std::isnan(L[r]) || std::isnan(R[r])) continue;
      ++both;
      if (std::abs((R[r] - L[r]) - lw) > tol) ++bad;
    }
    if (both < 5 || bad * 3 < both) break;   // 겹친 구간의 1/3 이상이 틀릴 때만
    // 근거가 약한 쪽(직전 기준 거리가 큰 쪽, 없으면 점이 적은 쪽)의 겹친 조각을 뺀다
    auto weakness = [&](int s) {
      double ev = 0, pts = 0;
      for (const auto & f : fr) if (f.side == s) { ev = std::max(ev, f.evidence); pts += double(f.pts->size()); }
      return ev < 1e8 ? ev : 1e6 - pts;
    };
    const int drop = weakness(0) >= weakness(1) ? 0 : 1;
    const auto & other = drop == 0 ? R : L;
    for (auto & f : fr) if (f.side == drop && !std::isnan(mean_offset(*f.pts, other))) { f.side = 2; ++st.dropped; }
  }
  // 결과 반영 + 다음 프레임 기준 저장
  Paths out;
  for (const auto & f : fr) {
    if (f.side == 0 || f.side == 1) {
      if (f.side != f.model) ++st.relabeled;
      out[f.side].push_back(*f.pts);
    }
  }
  paths.swap(out);
  for (int s = 0; s < 2; ++s) {
    if (paths[s].empty()) continue;
    std::vector<const std::vector<cv::Point2f> *> ref;
    for (const auto & f : paths[s]) ref.push_back(&f);
    prev_[s] = profile(ref, rows);
    prev_t_[s] = t;
  }
  return st;
}

}  // namespace semi_slam
