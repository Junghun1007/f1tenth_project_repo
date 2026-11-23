#include "cone_drive_test/core/lane_memory.hpp"
#include <algorithm>
#include <cmath>

namespace cone_drive_test {

void LaneConeMemory::drop_behind() {
  const double limit = -(p_.front_axle_to_rear_bumper + p_.delete_behind);
  std::vector<LaneCone> keep;
  for (const auto & c : tracks_) if (c.s >= limit && c.misses < p_.miss_delete) keep.push_back(c);
  tracks_.swap(keep);
}

void LaneConeMemory::advance(double ds) {
  if (!(ds > 0.0)) return;   // 후진·정지는 무시
  for (auto & c : tracks_) c.s -= ds;
  drop_behind();
}

void LaneConeMemory::update(const std::vector<Vec2> & obs, const std::vector<double> & ranges, bool complete) {
  std::vector<char> used(tracks_.size(), 0);
  for (std::size_t i = 0; i < obs.size(); ++i) {
    const auto & o = obs[i];
    const double sig = p_.range_noise_base + p_.range_noise_per_m * (i < ranges.size() ? ranges[i] : 1.0);
    const double w = 1.0 / (sig * sig);
    int best = -1;
    double bd = p_.match_radius;
    for (std::size_t k = 0; k < tracks_.size(); ++k) {
      if (used[k]) continue;
      const double d = std::hypot(tracks_[k].s - o.x, tracks_[k].l - o.y);
      if (d < bd) { bd = d; best = int(k); }
    }
    if (best < 0) {
      tracks_.push_back({o.x, o.y, 1, 0, next_id_++, w});
      used.push_back(1);
    } else {
      auto & c = tracks_[best];   // 가까운 관측일수록 크게 반영, 상한으로 너무 굳지 않게 (월드 기억과 같음)
      c.s = (c.s * c.weight + o.x * w) / (c.weight + w);
      c.l = (c.l * c.weight + o.y * w) / (c.weight + w);
      c.weight = std::min(c.weight + w, 50.0 * w);
      ++c.hits;
      c.misses = 0;
      used[best] = 1;
    }
  }
  for (std::size_t k = 0; k < tracks_.size(); ++k)
    if (complete && !used[k] && tracks_[k].s >= p_.visible_s_min) ++tracks_[k].misses;
  drop_behind();
}

std::vector<Vec2> LaneConeMemory::cones_sl() const {
  std::vector<Vec2> out;
  for (const auto & c : tracks_) if (confirmed(c)) out.push_back({c.s, c.l});
  return out;
}

}  // namespace cone_drive_test
