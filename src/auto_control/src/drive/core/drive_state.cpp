// Copied from src/cone_drive_test/src/core/drive_state.cpp (1113ML) — auto_control로 옮김
#include "auto_control/drive/core/drive_state.hpp"
#include <cmath>

namespace auto_control::drive {

const char * state_name(DriveState s) {
  switch (s) {
    case DriveState::Boot: return "BOOT";
    case DriveState::Inspecting: return "INSPECTING";
    case DriveState::Ready: return "READY";
    case DriveState::Arming: return "ARMING";
    case DriveState::Driving: return "DRIVING";
    case DriveState::Stopping: return "STOPPING";
    case DriveState::Manual: return "MANUAL";
    default: return "REFITTING";
  }
}
const char * mode_name(DriveMode m) { return m == DriveMode::Drive ? "drive" : "detect"; }

DriveStateMachine::DriveStateMachine(DriveMode initial, double link_loss, double arming_timeout, double v_still, double t_still)
  : mode_(initial), link_loss_(link_loss), arming_timeout_(arming_timeout), standstill_speed_(v_still), standstill_sec_(t_still) {}

bool DriveStateMachine::client_alive(double now) const { return alive_clients(now) > 0; }

int DriveStateMachine::alive_clients(double now) const {
  int n = 0;
  for (const auto & [id, t] : clients_) n += now - t <= link_loss_;
  return n;
}

void DriveStateMachine::on_heartbeat(const std::string & client, double now) { clients_[client] = now; }

std::string DriveStateMachine::readiness_problem(const Readiness & r, double now) const {
  if (!r.attitude_ok) return "camera attitude not measured";
  if (!r.lane_model_ready) return "lane model not ready";
  if (!r.yolo_model_ready) return "yolo model not ready";
  if (!r.ir_model_ready) return "tunnel IR lane model not ready";
  if (!r.vesc_connected) return "vesc disconnected";
  if (!r.erpm_fresh) return "erpm stale";
  if (!r.lane_fresh) return "lane input stale";
  if (!client_alive(now)) return "no browser heartbeat";
  return "";
}

std::pair<bool, std::string> DriveStateMachine::request_start(const Readiness & r, double now) {
  if (mode_ != DriveMode::Drive) return {false, "detect mode: switch to drive mode first"};
  if (state_ != DriveState::Ready && state_ != DriveState::Manual) return {false, std::string("cannot start in ") + state_name(state_)};
  const auto problem = readiness_problem(r, now);
  if (!problem.empty()) return {false, problem};
  state_ = DriveState::Arming;
  arming_since_ = now;
  reason_.clear();
  return {true, ""};
}

void DriveStateMachine::to_stopping(const std::string & reason) {
  state_ = DriveState::Stopping;
  reason_ = reason;
  still_since_ = -1;
}

void DriveStateMachine::request_stop(const std::string & reason, double) {
  if (state_ == DriveState::Driving) to_stopping(reason);
  else if (state_ == DriveState::Arming) { state_ = DriveState::Ready; reason_ = reason; }
}

std::pair<bool, std::string> DriveStateMachine::request_refit(double) {
  switch (state_) {
    case DriveState::Ready:
    case DriveState::Manual:
      state_ = DriveState::Refitting;
      refit_pending_ = true;
      return {true, ""};
    case DriveState::Driving:
      refit_pending_ = true;
      to_stopping("refit requested");
      return {true, ""};
    case DriveState::Stopping:
      refit_pending_ = true;
      return {true, ""};
    default:
      return {false, std::string("cannot refit in ") + state_name(state_)};
  }
}

std::pair<bool, std::string> DriveStateMachine::request_mode(DriveMode m, double) {
  if (state_ != DriveState::Ready && state_ != DriveState::Manual) return {false, std::string("stop first (") + state_name(state_) + ")"};
  mode_ = m;
  return {true, ""};
}

void DriveStateMachine::on_inspection_done(bool ok, double, const std::string & reason) {
  if (state_ != DriveState::Inspecting && state_ != DriveState::Boot) return;
  if (ok) { state_ = DriveState::Ready; reason_.clear(); }
  else reason_ = reason.empty() ? "inspection failed" : reason;
}

void DriveStateMachine::on_refit_done(bool ok, double, const std::string & reason) {
  if (state_ != DriveState::Refitting) return;
  refit_pending_ = false;
  state_ = DriveState::Ready;
  reason_ = ok ? "" : "refit failed: " + (reason.empty() ? std::string("measurement rejected") : reason) + " (previous calibration kept)";
}

void DriveStateMachine::on_handover_mode(const std::string & m, double now) {
  safety_mode_ = m;
  if (m == "AUTO") {
    if (state_ == DriveState::Arming && mode_ == DriveMode::Drive) { state_ = DriveState::Driving; reason_.clear(); }
    return;
  }
  // ARMING 중 MANUAL은 거절이 아니다 (안전 프로세스가 주기적으로 다시 알림). 거절은 arming_timeout으로 처리
  if (state_ == DriveState::Arming && m == "MANUAL") return;
  if (state_ == DriveState::Driving || state_ == DriveState::Arming) to_stopping(m == "STOPPING" ? "joystick X handover" : "safety switched to manual");
  if (m == "MANUAL" && state_ == DriveState::Stopping) {
    state_ = refit_pending_ ? DriveState::Refitting : DriveState::Manual;
  }
  (void)now;
}

void DriveStateMachine::tick(const Readiness & r, double speed, double now) {
  now_ = now;
  if (state_ == DriveState::Arming && now - arming_since_ > arming_timeout_) {
    state_ = DriveState::Ready;
    reason_ = "safety did not accept AUTO: vehicle must be stopped (fresh ERPM) with pedals/steering released";
  }
  if (state_ == DriveState::Driving) {
    const auto problem = readiness_problem(r, now);
    if (!problem.empty()) to_stopping(problem == "no browser heartbeat" ? "browser link lost" : problem);
  }
  if (state_ == DriveState::Stopping) {
    if (std::abs(speed) < standstill_speed_) { if (still_since_ < 0) still_since_ = now; }
    else still_since_ = -1;
  }
}

bool DriveStateMachine::auto_output_allowed() const {
  return mode_ == DriveMode::Drive && safety_mode_ == "AUTO" &&
         (state_ == DriveState::Driving || state_ == DriveState::Stopping);
}

bool DriveStateMachine::wants_manual_handover() const {
  if (safety_mode_ != "AUTO") return false;
  // 출발 승인 없이 안전 프로세스가 AUTO (늦게 수락된 START, 주행 프로세스 재시작 등) → 바로 수동으로 되돌림
  if (state_ != DriveState::Arming && state_ != DriveState::Driving && state_ != DriveState::Stopping) return true;
  // 정지가 standstill_sec 이상 유지되면 안전 프로세스에 수동 전환 요청
  return state_ == DriveState::Stopping && still_since_ >= 0 && now_ - still_since_ >= standstill_sec_;
}

}  // namespace auto_control::drive
