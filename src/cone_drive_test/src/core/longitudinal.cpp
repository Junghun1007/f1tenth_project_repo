#include "cone_drive_test/core/longitudinal.hpp"
#include "cone_drive_test/core/duty_only.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace cone_drive_test {

LongitudinalController::LongitudinalController(const LongitudinalParams & p) {
  set_params(p);
  active_mode_ = p.mode;
}

void LongitudinalController::set_params(const LongitudinalParams & p) {
  p_ = p;
  pid_ = std::make_unique<SpeedPid>(p.pid_kp, p.pid_ki, p.pid_kd, p.pid_integral_limit, -1.0, 1.0);
  const auto & d = p.duty_only;
  duty_pid_ = std::make_unique<SpeedPid>(d.kp, d.ki, d.kd, d.integral_limit, 0.0, d.maximum_duty);
}

// 모드 전환은 정지했을 때만: 움직이는 중에 다른 제어기의 적분·제동 상태·duty 램프를 넘겨받지 않게 (auto_control과 같음)
void LongitudinalController::apply_requested_mode() {
  if (active_mode_ == p_.mode || std::abs(speed_) >= 0.05) return;
  active_mode_ = p_.mode;
  pid_->reset();
  duty_pid_->reset();
  service_brake_ = traction_recovery_ = duty_emergency_ = false;
  coast_probe_ = 0.0;
  departure_pending_ = true;
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
  apply_requested_mode();
  const bool duty_only = active_mode_ == 2;
  const double v = std::max(0.0, speed_);
  // 장애물 목표 램프: 검출 시 감속률로 낮추고, 해제 시 가속률로 회복
  obstacle_requested_ = obstacle_active ? std::min(p_.maximum_speed_mps, p_.obstacle_speed_mps) : p_.maximum_speed_mps;
  if (obstacle_planned_ < 0)
    obstacle_planned_ = obstacle_active ? std::clamp(std::max(v, obstacle_requested_), 0.0, p_.maximum_speed_mps) : p_.maximum_speed_mps;
  const double prev_obs = obstacle_planned_;
  const bool recovering = obstacle_requested_ > prev_obs;
  obstacle_planned_ = move_toward(prev_obs, obstacle_requested_,
    (recovering ? p_.obstacle_planning_acceleration_mps2
                : duty_only ? p_.duty_only.obstacle_planning_deceleration_mps2 : p_.obstacle_planning_deceleration_mps2) * dt);
  obstacle_plan_decel_ = std::max(0.0, (prev_obs - obstacle_planned_) / dt);
  const bool obstacle_profile = obstacle_active || obstacle_planned_ < p_.maximum_speed_mps - 1e-6;

  double target = std::min(plan_target, obstacle_planned_);
  if (v >= 0.05 && last_target_ > 0.0) target = std::min(target, last_target_ + p_.planning_acceleration_mps2 * dt);
  target = std::min({target, speed_cap, p_.maximum_speed_mps});
  if (stop) target = 0.0;
  target = std::max(0.0, target);
  if (duty_only) return duty_only_update(dt, target, obstacle_profile);

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

// 모드 2: duty만으로 목표 속도를 따라간다. 지금 속도의 duty보다 낮추면 VESC가 회생 감속하므로 코너·장애물·앞차 감속에
// 전류 제동으로 바꾸지 않는다. 전류 제동은 정지 유지(선택)와 비상(감속 부족·큰 속도 초과)에만 쓴다 (auto_control과 같음)
LongitudinalCommand LongitudinalController::duty_only_update(double dt, double target, bool obstacle_profile) {
  const auto & d = p_.duty_only;
  pid_->reset();
  service_brake_ = traction_recovery_ = false;
  coast_probe_ = 0.0;
  const double speed = std::max(0.0, speed_);
  const double overspeed = speed - target;
  const bool zero = target <= 0.0;
  if (zero != zero_target_) duty_pid_->reset();
  zero_target_ = zero;
  const double required = obstacle_profile ? obstacle_plan_decel_ : std::numeric_limits<double>::quiet_NaN();
  const bool emergency_enabled = d.emergency_deceleration_mps2 > 0.0 || d.emergency_overspeed_mps > 0.0;
  auto condition = [&](double scale) {
    const bool decel = d.emergency_deceleration_mps2 > 0.0 && std::isfinite(required) && required >= scale * d.emergency_deceleration_mps2;
    const bool fast = d.emergency_overspeed_mps > 0.0 && overspeed >= scale * d.emergency_overspeed_mps;
    return decel || fast;
  };
  if (!emergency_enabled || speed < 0.05) duty_emergency_ = false;
  else duty_emergency_ = condition(duty_emergency_ ? 0.7 : 1.0);   // 걸린 뒤엔 70%까지 내려와야 해제

  LongitudinalCommand out;
  out.target_speed = target;
  double requested = duty_emergency_ ? d.emergency_brake_current_amps : 0.0;
  const bool hold = zero && std::abs(speed_) < 0.05 && d.hold_current_amps > 0.0;
  if (hold) requested = std::max(requested, d.hold_current_amps);
  if (requested > 0.0 || brake_mode_) {
    departure_pending_ = true;
    duty_pid_->reset();
    duty_ = 0.0;
    brake_ = move_toward(brake_, requested,
                         (requested > brake_ ? d.emergency_brake_rise_amps_per_sec : d.emergency_brake_fall_amps_per_sec) * dt);
    if (brake_ > 1e-6) {
      brake_mode_ = true;
      last_brake_t_ = now_t_;
      phase_ = duty_emergency_ ? "duty_only_emergency_brake" : hold ? "duty_only_hold" : "duty_only_brake_release";
      out.brake_current = brake_;
      out.brake_mode = true;
      out.phase = phase_;
      last_target_ = target;
      return out;
    }
    brake_ = 0.0;
    if (brake_mode_) {   // 제동 해제 주기를 따로 보낸 뒤 duty
      brake_mode_ = false;
      phase_ = "duty_only_brake_release";
      out.brake_mode = true;
      out.phase = phase_;
      last_target_ = target;
      return out;
    }
  }
  if (zero) {   // 모터를 풀고 굴러서 선다
    duty_ = brake_ = 0.0;
    duty_pid_->reset();
    departure_pending_ = true;
    phase_ = "duty_only_coast";
    out.phase = phase_;
    last_target_ = target;
    return out;
  }
  const double ff = duty_feedforward(target, d.ff_offset_duty, d.ff_duty_per_mps, d.ff_fade_speed_mps, d.maximum_duty);
  const double desired = duty_pid_->update(target, speed_, ff, dt);
  const double prev = duty_;
  const bool start = departure_pending_ && d.start_duty > 0.0 && std::abs(speed_) < 0.05 && duty_ <= 1e-6 && desired > 0.0;
  bool matched = false;
  if (start) {
    duty_ = std::min(d.start_duty, desired);
  } else {
    if (speed >= 0.05 && desired > duty_) {   // 굴러가다 다시 걸 때: 0부터 올리지 않고 지금 속도에 맞는 duty에서 시작
      const double matching = std::min(desired, duty_feedforward(speed, d.ff_offset_duty, d.ff_duty_per_mps, d.ff_fade_speed_mps, d.maximum_duty));
      if (matching > duty_) { duty_ = matching; matched = true; }
    }
    duty_ = move_toward(duty_, desired, (desired > duty_ ? d.duty_rise_rate_per_sec : d.duty_fall_rate_per_sec) * dt);
  }
  duty_pid_->apply_output_limit(desired, duty_);
  if (start || speed >= 0.10) departure_pending_ = false;
  phase_ = start ? "duty_only_start" : matched ? "duty_only_resume" : duty_ < prev - 1e-9 ? "duty_only_decelerate" : "duty_only_track";
  out.duty = duty_;
  out.phase = phase_;
  last_target_ = target;
  return out;
}

}  // namespace cone_drive_test
