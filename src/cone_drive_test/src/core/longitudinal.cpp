#include "cone_drive_test/core/longitudinal.hpp"
#include <algorithm>
#include <cmath>

namespace cone_drive_test {

LongitudinalController::LongitudinalController(const LongitudinalParams & p) { set_params(p); }

void LongitudinalController::set_params(const LongitudinalParams & p) {
  p_ = p;
  pid_ = std::make_unique<SpeedPid>(p.pid_kp, p.pid_ki, p.pid_kd, p.pid_integral_limit, -1.0, 1.0);
}

void LongitudinalController::reset() {
  pid_->reset();
  duty_ = brake_ = 0;
  coast_probe_ = 0;
  obstacle_planned_ = -1;
  brake_mode_ = service_brake_ = traction_recovery_ = false;
  departure_pending_ = true;
  last_target_ = 0;
  phase_ = "reset";
}

void LongitudinalController::on_speed(double raw, double t) {
  now_t_ = t;
  if (!have_speed_ || t <= last_t_ || t - last_t_ > 0.5) {
    speed_ = raw;
    have_speed_ = true;
    have_decel_ = false;
    last_t_ = t;
    return;
  }
  const double dt = t - last_t_, prev = speed_;
  speed_ += (1.0 - std::exp(-dt / std::max(1e-4, p_.speed_filter_time_constant_sec))) * (raw - speed_);
  const double d = std::clamp((prev - speed_) / dt, -5.0, 5.0);
  decel_ = have_decel_ ? decel_ + (1.0 - std::exp(-dt / p_.deceleration_filter_sec)) * (d - decel_) : d;
  have_decel_ = true;
  // duty 감속만으로 얻는 감속도: 제동이 끝나고 안정된 뒤 duty 감속 단계에서만 갱신
  const bool brake_settled = t - last_brake_t_ >= p_.response_time_sec + 3.0 * (p_.speed_filter_time_constant_sec + p_.deceleration_filter_sec);
  if (brake_settled && phase_ == "reduce_duty" && brake_ == 0.0) { coast_decel_ = std::max(0.0, decel_); coast_seen_t_ = t; }
  last_t_ = t;
}

LongitudinalCommand LongitudinalController::update(double dt, double plan_target, bool obstacle_active, double speed_cap, bool stop) {
  dt = std::max(dt, 1e-6);
  const double v = std::max(0.0, speed_);
  // 장애물 목표 램프: 검출 시 감속률로 낮추고, 해제 시 가속률로 회복
  obstacle_requested_ = obstacle_active ? std::min(p_.maximum_speed_mps, p_.obstacle_speed_mps) : p_.maximum_speed_mps;
  if (obstacle_planned_ < 0)
    obstacle_planned_ = obstacle_active ? std::clamp(std::max(v, obstacle_requested_), 0.0, p_.maximum_speed_mps) : p_.maximum_speed_mps;
  const double prev_obs = obstacle_planned_;
  const bool recovering = obstacle_requested_ > prev_obs;
  obstacle_planned_ = move_toward(prev_obs, obstacle_requested_,
    (recovering ? p_.obstacle_planning_acceleration_mps2 : p_.obstacle_planning_deceleration_mps2) * dt);
  obstacle_plan_decel_ = std::max(0.0, (prev_obs - obstacle_planned_) / dt);
  const bool obstacle_profile = obstacle_active || obstacle_planned_ < p_.maximum_speed_mps - 1e-6;

  double target = std::min(plan_target, obstacle_planned_);
  if (v >= 0.05 && last_target_ > 0.0) target = std::min(target, last_target_ + p_.planning_acceleration_mps2 * dt);
  target = std::min({target, speed_cap, p_.maximum_speed_mps});
  if (stop) target = 0.0;
  target = std::max(0.0, target);

  const double overspeed = v - target;
  const bool zero = target <= 0.0;
  if (zero != zero_target_) pid_->reset();
  zero_target_ = zero;
  const bool braking_allowed = p_.electrical_brake_enabled || obstacle_profile || stop;
  const bool approaching_obstacle = obstacle_profile && !zero;
  double required = 0.0;
  if (approaching_obstacle)
    required = obstacle_plan_decel_ + std::max(0.0, overspeed) / std::max(0.10, p_.obstacle_coast_probe_sec + p_.longitudinal_brake_release_response_sec);
  const double coast = now_t_ - coast_seen_t_ <= 1.0 ? coast_decel_ : 0.0;
  const double observed = have_decel_ ? std::max(0.0, decel_) : 0.0;
  const bool was_braking = service_brake_;
  const double horizon = brake_ / p_.brake_current_fall_amps_per_sec + p_.longitudinal_brake_release_response_sec +
                         p_.speed_filter_time_constant_sec + p_.recovery_start_ramp_sec;
  const double predicted_release = std::max(0.0, v - observed * horizon);
  const double release_target = approaching_obstacle
    ? std::min(target, std::max(obstacle_requested_, target - obstacle_plan_decel_ * horizon)) : target;
  const double release_margin = approaching_obstacle ? p_.obstacle_brake_exit_margin_mps : p_.brake_exit_speed_error_mps;
  const bool undershoot = predicted_release < release_target - release_margin;

  if (zero) {
    departure_pending_ = true;
    service_brake_ = braking_allowed;
    coast_probe_ = 0.0;
    traction_recovery_ = false;
  } else if (approaching_obstacle) {
    const bool insufficient = required > coast + p_.obstacle_brake_deceleration_margin_mps2;
    if (service_brake_) {
      if (overspeed <= p_.obstacle_brake_exit_margin_mps || undershoot ||
          required <= coast + 0.5 * p_.obstacle_brake_deceleration_margin_mps2) service_brake_ = false;
    } else if (overspeed > 0.0 && !brake_mode_) {
      coast_probe_ += dt;
      service_brake_ = coast_probe_ >= p_.obstacle_coast_probe_sec && overspeed > p_.obstacle_brake_entry_margin_mps &&
                       insufficient && !undershoot;
    } else {
      coast_probe_ = 0.0;
    }
  } else {
    service_brake_ = braking_allowed && overspeed > (service_brake_ ? p_.brake_exit_speed_error_mps : p_.brake_entry_speed_error_mps);
    coast_probe_ = 0.0;
  }
  if (!braking_allowed) service_brake_ = false;
  double requested_brake = 0.0;
  if (service_brake_) {
    const double missing = approaching_obstacle ? std::max(0.0, required - coast) : 0.0;
    const double err = zero ? std::abs(speed_) : std::max(0.0, overspeed);
    requested_brake = staged_brake_current(err, missing, p_.brake_speed_gain, p_.brake_deceleration_gain, p_.brake_maximum_current_amps);
  }
  if (was_braking && !service_brake_) { traction_recovery_ = !zero; coast_probe_ = 0.0; }

  LongitudinalCommand out;
  out.target_speed = target;
  if (service_brake_ || brake_mode_) {
    departure_pending_ = true;
    pid_->reset();
    duty_ = 0.0;
    brake_ = move_toward(brake_, requested_brake,
      (requested_brake > brake_ ? p_.brake_current_rise_amps_per_sec : p_.brake_current_fall_amps_per_sec) * dt);
    if (brake_ > 1e-6) {
      brake_mode_ = true;
      last_brake_t_ = now_t_;
      phase_ = service_brake_ ? (zero ? "stop_brake" : "additional_brake") : "release_brake";
      out.brake_current = brake_;
      out.brake_mode = true;
      out.phase = phase_;
      last_target_ = target;
      return out;
    }
    brake_ = 0.0;
    if (brake_mode_) {   // 해제는 별도 주기로 내보낸 뒤 duty 복귀
      brake_mode_ = false;
      traction_recovery_ = !zero;
      phase_ = "release_brake";
      out.brake_mode = true;
      out.phase = phase_;
      last_target_ = target;
      return out;
    }
  }
  if (zero) {
    duty_ = brake_ = 0.0;
    pid_->reset();
    phase_ = "zero_target";
    out.phase = phase_;
    last_target_ = target;
    return out;
  }
  const double ff = std::clamp(p_.feedforward_offset_duty * std::min(1.0, target / p_.feedforward_fade_speed_mps) +
                               p_.feedforward_duty_per_mps * target, 0.0, p_.maximum_duty);
  const double effort = pid_->update(target, speed_, ff / p_.maximum_duty, dt);
  const double desired = std::clamp(effort * p_.maximum_duty, 0.0, p_.maximum_duty);
  const bool start = departure_pending_ && p_.longitudinal_start_duty > 0.0 && std::abs(speed_) < 0.05 && duty_ <= 1e-6 && desired > 0.0;
  if (start) traction_recovery_ = false;
  const double rise = traction_recovery_ ? p_.recovery_duty_rise_per_sec : p_.duty_rise_rate_per_sec;
  const double prev = duty_;
  const double recovery_duty = std::min(p_.longitudinal_start_duty, desired);
  const double time_to_recover = std::max(0.0, recovery_duty - duty_) / rise;
  const bool recover = !start && (traction_recovery_ || departure_pending_) && recovery_duty > duty_ && target > 0.05 &&
                       (overspeed < -0.025 || undershoot) && v - observed * (time_to_recover + p_.response_time_sec) < 0.10;
  duty_ = start ? std::min(p_.longitudinal_start_duty, desired)
                : move_toward(duty_, desired, (desired > duty_ ? rise : p_.duty_fall_rate_per_sec) * dt);
  if (recover) {
    duty_ = std::max(duty_, move_toward(prev, recovery_duty, p_.longitudinal_start_duty / p_.recovery_start_ramp_sec * dt));
    traction_recovery_ = true;
  }
  if (start || p_.longitudinal_start_duty <= 0.0 || (recovery_duty > 0.0 && duty_ >= recovery_duty) ||
      (v >= 0.10 && std::abs(overspeed) <= 0.05)) departure_pending_ = false;
  pid_->apply_output_limit(effort, duty_ / p_.maximum_duty);
  if (traction_recovery_ && std::abs(duty_ - desired) < 1e-4 && overspeed >= -0.05) traction_recovery_ = false;
  phase_ = start ? "start_drive" : recover ? "recover_motion" : traction_recovery_ ? "recover_drive"
         : (overspeed > 0.0 || duty_ < prev ? "reduce_duty" : "track_speed");
  out.duty = duty_;
  out.phase = phase_;
  last_target_ = target;
  return out;
}

}  // namespace cone_drive_test
