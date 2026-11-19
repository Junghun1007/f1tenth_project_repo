#include "semi_slam/core/drive_loop.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace semi_slam {

DriveLoop::DriveLoop(const RuntimeConfig & rc)
  : rc_(rc), sm_(DriveMode::Drive, 1.0), lon_(LongitudinalParams{}), steer_(rc.steering) {
  apply_settings(Settings{});
}

void DriveLoop::apply_settings(const Settings & s) {
  std::lock_guard<std::mutex> l(m_);
  s_ = s;
  lon_.set_params(s.lon);
  auto sp = rc_.steering;
  sp.steering_current_weight = s.steering_current_weight;
  sp.rate_limit_rad_s = s.steering_rate_rad_s;
  steer_.set_params(sp);
  sm_.set_link_loss_stop_sec(s.link_loss_stop_sec);
}

double DriveLoop::on_erpm(int erpm, double t) {
  const double v = erpm_to_speed_mps(erpm, rc_.wheel_diameter_m, rc_.motor_pole_pairs, rc_.motor_pinion_teeth, rc_.spur_gear_teeth,
                                     rc_.differential_pinion_teeth, rc_.differential_ring_teeth, rc_.erpm_direction_sign,
                                     rc_.speed_scale_correction);
  on_speed(v, t);
  return v;
}

void DriveLoop::on_speed(double v, double t) {
  std::lock_guard<std::mutex> l(m_);
  lon_.on_speed(v, t);
  speed_ = lon_.speed();
  speed_atomic_.store(speed_, std::memory_order_relaxed);
  last_erpm_t_ = t;
}

void DriveLoop::on_vesc_connected(bool c) { std::lock_guard<std::mutex> l(m_); vesc_connected_ = c; }
void DriveLoop::on_handover_mode(const std::string & mode, double t) { std::lock_guard<std::mutex> l(m_); sm_.on_handover_mode(mode, t); }
void DriveLoop::set_attitude(const AttitudeInfo & a) { std::lock_guard<std::mutex> l(m_); attitude_ = a; }
void DriveLoop::set_model_status(const InspectionModel & m) { std::lock_guard<std::mutex> l(m_); model_ = m; }
void DriveLoop::on_inspection_done(bool ok, const std::string & reason, double t) { std::lock_guard<std::mutex> l(m_); sm_.on_inspection_done(ok, t, reason); }
void DriveLoop::on_heartbeat(const std::string & c, double now) { std::lock_guard<std::mutex> l(m_); sm_.on_heartbeat(c, now); }
DriveState DriveLoop::state() const { std::lock_guard<std::mutex> l(m_); return sm_.state(); }

Readiness DriveLoop::readiness(double now) const {
  Readiness r;
  r.attitude_ok = attitude_.ok;
  r.lane_model_ready = model_.status == "READY";
  r.yolo_model_ready = true;   // semi_slam은 YOLO를 쓰지 않음
  r.vesc_connected = vesc_connected_;
  r.erpm_fresh = now - last_erpm_t_ <= rc_.erpm_timeout_sec;
  r.lane_fresh = lane_.valid && (s_.lane_timeout_sec <= 0 || now - last_lane_t_ <= s_.lane_timeout_sec);
  r.client_alive = sm_.client_alive(now);
  return r;
}

StepOutput DriveLoop::on_lane(const LaneObservation & lane, double now) {
  std::lock_guard<std::mutex> l(m_);
  if (lane.valid) { lane_ = lane; last_lane_t_ = now; }
  return control_step(now, lane.valid);
}

StepOutput DriveLoop::tick(double now) {
  std::lock_guard<std::mutex> l(m_);
  return control_step(now, false);
}

StepOutput DriveLoop::control_step(double now, bool fresh_path) {
  const double t0 = now_sec();
  const Readiness r = readiness(now);
  sm_.tick(r, speed_, now);
  const double dt = last_control_t_ < 0 ? 0.01 : std::clamp(now - last_control_t_, 1e-4, 0.1);
  last_control_t_ = now;
  const bool stop = sm_.state() != DriveState::Driving;
  // 차선이 잠깐 늦으면 조향은 그대로 두고 속도만 유지 (lane_timeout 넘으면 상태기계가 정지)
  const bool path_ok = lane_.valid && (s_.lane_timeout_sec <= 0 || now - last_lane_t_ <= s_.lane_timeout_sec);
  if (fresh_path) {
    std::vector<Point> pts;
    for (const auto & q : lane_.path) pts.push_back({q.x, q.y});
    if (const auto path = build_ordered_path(pts, 4, rc_.path_min_span_m, -0.5, 3.5, rc_.path_max_gap_m, rc_.geometry_window_m)) {
      const auto st = stanley_control(*path, speed_, s_.stanley_gain, s_.stanley_softening, s_.heading_lookahead_m,
                                      rc_.steering.max_steer_rad, rc_.steering.corner_heading_threshold_rad, 0.8);
      cte_ = st.cross_track_error_m;
      steer_.update(st.steering_angle_rad, dt, st.heading_error_rad, stop ? 0.0 : s_.run_speed, speed_);
    }
  }
  const auto cmd = lon_.update(dt, path_ok ? s_.run_speed : 0.0, false, std::numeric_limits<double>::infinity(), stop || !path_ok);
  target_ = cmd.target_speed;
  phase_ = cmd.phase;
  StepOutput out;
  out.duty = cmd.duty;
  out.brake_current = cmd.brake_current;
  out.brake_mode = cmd.brake_mode;
  out.servo = steer_.servo(steer_.angle());
  out.publish = sm_.auto_output_allowed();
  if (!out.publish && sm_.state() != DriveState::Stopping) lon_.reset();
  if (now - last_request_t_ >= rc_.handover_request_period_sec) {
    if (sm_.wants_auto_rearm()) out.handover_request = "auto";
    else if (sm_.wants_manual_handover()) out.handover_request = "manual";
    if (!out.handover_request.empty()) last_request_t_ = now;
  }
  last_out_ = out;
  stats_.record("control", t0, now_sec());
  return out;
}

std::pair<bool, std::string> DriveLoop::request_start(double now) {
  std::lock_guard<std::mutex> l(m_);
  auto res = sm_.request_start(readiness(now), now);
  if (res.first) { lon_.reset(); steer_.reset(); last_request_t_ = -1e9; }
  return res;
}

void DriveLoop::request_stop(const std::string & reason, double now) { std::lock_guard<std::mutex> l(m_); sm_.request_stop(reason, now); }

DriveLoop::Snapshot DriveLoop::snapshot(double now) const {
  std::lock_guard<std::mutex> l(m_);
  Snapshot s;
  s.state = state_name(sm_.state());
  s.reason = sm_.last_reason();
  s.readiness = readiness(now);
  s.attitude = attitude_;
  s.model = model_;
  s.vehicle = {speed_, target_, steer_.angle() * 180.0 / 3.14159265358979323846, cte_, last_out_.duty, last_out_.brake_current, phase_,
               lon_.active_mode()};
  s.lane = lane_;
  return s;
}

}  // namespace semi_slam
