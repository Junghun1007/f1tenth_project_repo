// Copied from src/cone_drive_test/src/core/traffic_stop.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/traffic_stop.hpp"
#include "auto_control/drive/core/duty_only.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace auto_control::drive {

TrafficSignal classify_traffic_color(const cv::Mat & bgr, int min_s, int min_v) {
  if (bgr.empty() || bgr.type() != CV_8UC3) return TrafficSignal::Unknown;
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  std::uint64_t red = 0, green = 0;
  for (int r = 0; r < hsv.rows; ++r) {
    const auto * px = hsv.ptr<cv::Vec3b>(r);
    for (int c = 0; c < hsv.cols; ++c) {
      const auto p = px[c];
      if (p[1] < min_s || p[2] < min_v) continue;
      const std::uint64_t energy = unsigned(p[1]) * p[2];
      if (p[0] <= 20 || p[0] >= 165) red += energy;
      else if (p[0] >= 35 && p[0] <= 95) green += energy;
    }
  }
  return red > green ? TrafficSignal::Red : green > red ? TrafficSignal::Green : TrafficSignal::Unknown;
}

void TrafficStopper::on_signal(TrafficSignal sig, double t) {
  if (!p_.enabled) return;
  if (sig == TrafficSignal::Red) red_t_ = std::max(red_t_, t);
  if (sig == TrafficSignal::Green) green_t_ = std::max(green_t_, t);
  if (sig != TrafficSignal::Unknown) { last_sig_ = sig; sig_t_ = std::max(sig_t_, t); }
}

const TrafficState & TrafficStopper::update(double m, double t, double dist, double v) {
  if (!p_.enabled) { reset(); return s_; }
  s_.signal = t - sig_t_ <= p_.state_timeout ? last_sig_ : TrafficSignal::Unknown;
  // 정지선 추적 (앞차축 기준 거리)
  const double pred = s_.line_tracked ? meas_line_ - (dist - meas_dist_) : std::numeric_limits<double>::quiet_NaN();
  if (std::isfinite(m)) {
    if (s_.line_tracked && std::abs(m - pred) <= p_.match_tolerance) {
      meas_line_ = pred + p_.current_weight * (m - pred);
      meas_dist_ = dist; meas_t_ = t;
    } else {
      const double cand_pred = cand_line_ - (dist - cand_dist_);
      cand_n_ = (cand_n_ > 0 && std::abs(m - cand_pred) <= p_.match_tolerance) ? cand_n_ + 1 : 1;
      cand_line_ = m; cand_dist_ = dist;
      if (cand_n_ >= p_.line_confirm && (!s_.line_tracked || !s_.committed)) {   // 정지 중엔 추적 중인 선을 다른 선으로 갈아타지 않음
        s_.line_tracked = true;
        meas_line_ = m; meas_dist_ = dist; meas_t_ = t;
      }
    }
  } else {
    cand_n_ = 0;
  }
  if (s_.line_tracked) {
    const double bumper = meas_line_ - (dist - meas_dist_) - p_.front_overhang;
    const bool terminal = bumper <= p_.terminal_distance + p_.stop_range_max;
    const double age = t - meas_t_;
    const bool keep = s_.holding || age <= (terminal ? p_.terminal_timeout : p_.line_timeout);
    if (!keep) { s_.line_tracked = false; lost_t_ = t; }
  }
  s_.line_m = s_.line_tracked ? meas_line_ - (dist - meas_dist_) - p_.front_overhang : std::numeric_limits<double>::quiet_NaN();
  // 정지 시작·해제
  const bool red = t - red_t_ <= p_.state_timeout;
  const bool green = t - green_t_ <= p_.state_timeout && green_t_ > red_t_;
  if (!s_.committed && s_.line_tracked && red && s_.line_m > -0.05) s_.committed = true;
  if (s_.committed && green) { s_.committed = false; s_.holding = false; }        // 초록불 → 출발
  if (s_.committed && s_.line_tracked && s_.line_m < -p_.pass_overshoot) { s_.committed = false; s_.holding = false; }
  // 정지 중 정지선을 놓친 채 빨간불도 안 보이는 상태가 오래가면(오검출이었던 정지선) 해제
  if (s_.committed && !s_.line_tracked && !s_.holding && !red && t - lost_t_ > p_.line_lost_release) s_.committed = false;
  if (!s_.committed) s_.holding = false;
  s_.cap = s_.committed ? cap_for(v) : std::numeric_limits<double>::infinity();
  return s_;
}

double TrafficStopper::cap_for(double v) {
  if (s_.holding) return 0.0;
  if (!s_.line_tracked) return p_.reapproach_speed;   // 정지 중 정지선을 놓침: 천천히 다가가며 다시 찾음
  const double bumper = s_.line_m;
  const double aim = bumper - p_.stop_margin;
  const bool stopped = std::abs(v) < 0.05;
  if (stopped && bumper <= p_.stop_range_max) { s_.holding = true; return 0.0; }   // 정차 범위 안
  if (p_.duty_only) {
    if (duty_only_coast_release(aim, v, p_.coast_decel, p_.release_delay, p_.release_speed + 0.05)) return 0.0;
    const double lim = duty_only_stop_speed_limit(aim, p_.decel, p_.coast_decel, p_.release_speed, p_.response);
    return stopped ? std::min(lim, p_.reapproach_speed) : lim;   // 짧게 섰으면 천천히 다시 접근
  }
  if (aim <= 0.02) return 0.0;
  const double lim = std::sqrt(2.0 * p_.decel * aim);
  return stopped ? std::min(lim, p_.reapproach_speed) : lim;
}

}  // namespace auto_control::drive
