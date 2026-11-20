#include "semi_slam/core/vehicle_accel.hpp"
#include <algorithm>
#include <cmath>

namespace semi_slam {

namespace {
// 1차 저역 통과 계수 (시정수 tau)
double gain(double dt, double tau) { return 1.0 - std::exp(-std::max(0.0, dt) / tau); }
}  // namespace

void VehicleAccelModel::on_speed(double v, double t) {
  std::lock_guard<std::mutex> l(m_);
  if (!v_init_) { v_ = v; vd_ = 0; v_t_ = t; v_init_ = true; return; }
  const double dt = t - v_t_;
  if (dt <= 1e-4) return;
  if (dt > 0.3) { v_ = v; vd_ = 0; v_t_ = t; return; }   // 끊겼다 다시 들어오면 미분하지 않음
  // 지연이 크면 가속도계와 어긋나 오히려 남는 흔들림이 커진다 (기록 재생: 0.15 s → roll 1.6°, 0.03 s → 1.2°)
  const double v_new = v_ + gain(dt, 0.03) * (v - v_);
  vd_ += gain(dt, 0.08) * ((v_new - v_) / dt - vd_);
  v_ = v_new;
  v_t_ = t;
}

void VehicleAccelModel::on_yaw_rate(double w, double t) {
  std::lock_guard<std::mutex> l(m_);
  if (!w_init_) { w_ = w; wd_ = 0; w_t_ = t; w_init_ = true; return; }
  const double dt = t - w_t_;
  if (dt <= 1e-5) return;
  if (dt > 0.1) { w_ = w; wd_ = 0; w_t_ = t; return; }
  const double w_new = w_ + gain(dt, 0.005) * (w - w_);
  wd_ += gain(dt, 0.03) * ((w_new - w_) / dt - wd_);
  w_ = w_new;
  w_t_ = t;
}

std::optional<Vec3d> VehicleAccelModel::accel(double t) const {
  std::lock_guard<std::mutex> l(m_);
  if (!v_init_ || !w_init_ || t - v_t_ > 0.3) return std::nullopt;
  return Vec3d{vd_ - w_ * w_ * r_, v_ * w_ + wd_ * r_, 0.0};
}

}  // namespace semi_slam
