#include "cone_drive_test/core/cone_memory.hpp"
#include <algorithm>
#include <cmath>

namespace cone_drive_test {

Vec2 to_world(const Vec2 & l, const Pose2 & p) {
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return {p.x + c * l.x - s * l.y, p.y + s * l.x + c * l.y};
}

Vec2 to_local(const Vec2 & w, const Pose2 & p) {
  const double c = std::cos(p.yaw), s = std::sin(p.yaw), dx = w.x - p.x, dy = w.y - p.y;
  return {c * dx + s * dy, -s * dx + c * dy};
}

ConeMemory::ConeMemory(const MemoryParams & p) : p_(p) {}

void ConeMemory::update(const Pose2 & pose, double t, const std::vector<ConeObservation> & cones,
                        const ExpectedVisibleFn & expected, std::vector<std::uint32_t> * ids) {
  std::vector<char> used(tracks_.size(), 0);
  if (ids) ids->clear();
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  for (const auto & d : cones) {
    const Vec2 o = to_world(d.center, pose);
    // 스테레오 오차는 시선 방향으로 크다 → 시선·옆 방향을 따로 보는 타원 게이트
    const double rx = d.center.x - p_.camera_x, ry = d.center.y;
    const double r = std::hypot(rx, ry);
    const double ulx = rx / std::max(r, 1e-6), uly = ry / std::max(r, 1e-6);
    const double ux = c * ulx - s * uly, uy = s * ulx + c * uly;
    const double sig = p_.range_noise_base + p_.range_noise_per_m * r;
    const double gate_r = std::min(p_.gate_radial_max, p_.gate + 2.5 * sig);
    int best = -1;
    double bscore = 1.0;
    for (std::size_t k = 0; k < tracks_.size(); ++k) {
      if (used[k]) continue;
      const double dx = tracks_[k].world.x - o.x, dy = tracks_[k].world.y - o.y;
      const double along = dx * ux + dy * uy, across = -dx * uy + dy * ux;
      const double score = (along / gate_r) * (along / gate_r) + (across / p_.gate_lateral) * (across / p_.gate_lateral);
      if (score < bscore) { best = int(k); bscore = score; }
    }
    const double w = 1.0 / (sig * sig);
    if (best < 0) {
      tracks_.push_back({o, w, 1, 0, t, next_id_++});
      used.push_back(1);
      if (ids) ids->push_back(tracks_.back().id);
    } else {
      auto & tr = tracks_[best];   // 가까이서 본 관측일수록 크게 반영, 가중치 상한으로 너무 굳지 않게
      tr.world = {(tr.world.x * tr.weight + o.x * w) / (tr.weight + w), (tr.world.y * tr.weight + o.y * w) / (tr.weight + w)};
      tr.weight = std::min(tr.weight + w, 50.0 * w);
      tr.hits += 1;
      tr.misses = 0;
      tr.last_seen = t;
      used[best] = 1;
      if (ids) ids->push_back(tr.id);
    }
  }
  // 병합으로 인덱스가 바뀌므로 used를 id로 보관
  std::vector<std::uint32_t> used_ids;
  for (std::size_t k = 0; k < tracks_.size(); ++k) if (used[k]) used_ids.push_back(tracks_[k].id);
  merge_duplicates();
  if (!tracks_.empty() && expected) {
    std::vector<Vec2> centers;
    centers.reserve(tracks_.size());
    for (const auto & tr : tracks_) centers.push_back(tr.world);
    for (int i : expected(centers, pose)) {
      if (i < 0 || i >= int(tracks_.size())) continue;
      auto & tr = tracks_[i];
      if (std::find(used_ids.begin(), used_ids.end(), tr.id) != used_ids.end()) continue;
      const Vec2 l = to_local(tr.world, pose);
      if (l.x >= 0.15 && l.x <= p_.bev_x_max - 0.1 && std::abs(l.y) <= p_.bev_y_max - 0.05) tr.misses += 1;
    }
  }
  const double behind = -(p_.front_axle_to_rear_bumper + p_.delete_behind);
  std::vector<TrackedCone> keep;
  keep.reserve(tracks_.size());
  for (const auto & tr : tracks_) {
    const Vec2 l = to_local(tr.world, pose);
    if (l.x < behind) continue;
    const bool blind = l.x <= p_.blind_front && std::abs(l.y) <= p_.bev_y_max;   // 차 옆·바로 앞: 지나가는 중
    if (p_.ghost_timeout > 0 && !blind && t - tr.last_seen > p_.ghost_timeout) continue;
    if (tr.misses >= p_.miss_delete) continue;
    if (!confirmed(tr) && t - tr.last_seen > p_.unconfirmed_timeout) continue;
    keep.push_back(tr);
  }
  tracks_.swap(keep);
}

void ConeMemory::merge_duplicates() {
  // 실제 콘은 중심 간격 13 cm 미만으로 겹칠 수 없다
  bool merged = true;
  while (merged) {
    merged = false;
    for (std::size_t a = 0; a < tracks_.size() && !merged; ++a) {
      for (std::size_t b = a + 1; b < tracks_.size(); ++b) {
        auto & ta = tracks_[a];
        const auto & tb = tracks_[b];
        if (std::hypot(ta.world.x - tb.world.x, ta.world.y - tb.world.y) < p_.merge_distance) {
          const double tot = ta.weight + tb.weight;
          ta.world = {(ta.world.x * ta.weight + tb.world.x * tb.weight) / tot, (ta.world.y * ta.weight + tb.world.y * tb.weight) / tot};
          ta.weight = tot;
          ta.hits += tb.hits;
          ta.misses = std::min(ta.misses, tb.misses);
          ta.last_seen = std::max(ta.last_seen, tb.last_seen);
          tracks_.erase(tracks_.begin() + b);
          merged = true;
          break;
        }
      }
    }
  }
}

const TrackedCone * ConeMemory::find(std::uint32_t id) const {
  for (const auto & tr : tracks_) if (tr.id == id) return &tr;
  return nullptr;
}

std::vector<Vec2> ConeMemory::confirmed_world() const {
  std::vector<Vec2> out;
  for (const auto & tr : tracks_) if (confirmed(tr)) out.push_back(tr.world);
  return out;
}

}  // namespace cone_drive_test
