// Copied from src/cone_drive_test/include/cone_drive_test/core/drive_state.hpp (1113ML) — auto_control로 옮김
// 주행 상태기계. ROS 없이 시간(now, s)을 받아 동작한다.
//   Boot/Inspecting → Ready ⇄ (START) Arming → (안전 프로세스 AUTO) Driving → Stopping → (정지 확인, 안전 MANUAL) Manual
//   Refit: Ready/Manual에서 즉시, Driving에서는 Stopping → Manual 확인 후 Refitting → Ready
#pragma once
#include <map>
#include <string>
#include <utility>

namespace auto_control::drive {

enum class DriveMode { Detect, Drive };
enum class DriveState { Boot, Inspecting, Ready, Arming, Driving, Stopping, Manual, Refitting };
const char * state_name(DriveState s);
const char * mode_name(DriveMode m);

struct Readiness {
  bool attitude_ok{false}, lane_model_ready{false}, yolo_model_ready{false};
  bool vesc_connected{false}, erpm_fresh{false}, lane_fresh{false};
  bool client_alive{false};   // 상태기계가 heartbeat로 직접 계산해 덮어씀
  bool ir_model_ready{true};  // 터널(IR) 차선 모델: 터널을 켠 경우에만 검사 (auto_control)
  bool ir_model_used{false};
};

class DriveStateMachine {
public:
  DriveStateMachine(DriveMode initial, double link_loss_stop_sec, double arming_timeout_sec = 1.0,
                    double standstill_speed = 0.05, double standstill_sec = 0.5);
  std::pair<bool, std::string> request_start(const Readiness & r, double now);
  void request_stop(const std::string & reason, double now);
  std::pair<bool, std::string> request_refit(double now);
  std::pair<bool, std::string> request_mode(DriveMode m, double now);
  void on_heartbeat(const std::string & client, double now);
  void on_inspection_done(bool ok, double now, const std::string & reason = "");
  void on_refit_done(bool ok, double now, const std::string & reason = "");
  void on_handover_mode(const std::string & safety_mode, double now);   // AUTO | STOPPING | MANUAL
  void tick(const Readiness & r, double speed, double now);
  void set_link_loss_stop_sec(double s) { link_loss_ = s; }

  DriveState state() const { return state_; }
  DriveMode mode() const { return mode_; }
  bool auto_output_allowed() const;     // 자동 명령(구동·제동·조향) 발행 허용
  bool wants_manual_handover() const;   // 안전 프로세스에 "manual" 요청
  bool wants_auto_rearm() const { return state_ == DriveState::Arming; }   // 안전 프로세스에 "auto" 요청
  bool is_standstill(double speed) const;
  bool refit_pending() const { return refit_pending_; }
  bool client_alive(double now) const;
  int alive_clients(double now) const;
  const std::string & last_reason() const { return reason_; }
  const std::string & safety_mode() const { return safety_mode_; }

private:
  std::string readiness_problem(const Readiness & r, double now) const;
  void to_stopping(const std::string & reason);
  DriveMode mode_;
  DriveState state_{DriveState::Inspecting};
  double link_loss_, arming_timeout_, standstill_speed_, standstill_sec_;
  std::map<std::string, double> clients_;
  std::string reason_, safety_mode_{"MANUAL"};
  double arming_since_{0}, still_since_{-1}, now_{0};
  bool refit_pending_{false};
};

}  // namespace auto_control::drive
