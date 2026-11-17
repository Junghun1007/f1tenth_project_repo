#include "cone_drive_test/core/odometry.hpp"
#include <cmath>

namespace cone_drive_test {

void Odometry::reset(const Pose2 & p, double t) {
  psi_ = p.yaw;
  rx_ = p.x - L_ * std::cos(psi_);
  ry_ = p.y - L_ * std::sin(psi_);
  hist_.clear();
  hist_.push_back({t, pose(), dist_});
}

void Odometry::update(double t, double v, double w) {
  const double dt = hist_.empty() ? 0.0 : t - hist_.back().t;
  if (dt > 0) {
    rx_ += v * std::cos(psi_) * dt;
    ry_ += v * std::sin(psi_) * dt;
    psi_ += w * dt;
    dist_ += std::abs(v) * dt;
  }
  if (!hist_.empty() && t <= hist_.back().t) return;   // 시각이 뒤로 가면 이력에 넣지 않음
  hist_.push_back({t, pose(), dist_});
  while (hist_.size() > 2 && hist_.front().t < t - history_) hist_.pop_front();
}

Pose2 Odometry::pose() const { return {rx_ + L_ * std::cos(psi_), ry_ + L_ * std::sin(psi_), psi_}; }

std::optional<Pose2> Odometry::pose_at(double t) const {
  if (hist_.empty() || t < hist_.front().t - 1e-9 || t > hist_.back().t + 1e-9) return std::nullopt;
  for (std::size_t i = 1; i < hist_.size(); ++i) {
    const auto & a = hist_[i - 1];
    const auto & b = hist_[i];
    if (t <= b.t + 1e-12) {
      const double u = b.t > a.t ? (t - a.t) / (b.t - a.t) : 1.0;
      double dyaw = b.front.yaw - a.front.yaw;
      dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
      return Pose2{a.front.x + (b.front.x - a.front.x) * u, a.front.y + (b.front.y - a.front.y) * u, a.front.yaw + dyaw * u};
    }
  }
  return hist_.back().front;
}

}  // namespace cone_drive_test

namespace cone_drive_test {
std::optional<double> Odometry::distance_at(double t) const {
  if (hist_.empty() || t < hist_.front().t - 1e-9 || t > hist_.back().t + 1e-9) return std::nullopt;
  for (std::size_t i = 1; i < hist_.size(); ++i) {
    const auto & a = hist_[i - 1];
    const auto & b = hist_[i];
    if (t <= b.t + 1e-12) return a.dist + (b.dist - a.dist) * (b.t > a.t ? (t - a.t) / (b.t - a.t) : 1.0);
  }
  return hist_.back().dist;
}
}  // namespace cone_drive_test
