// Copied from src/cone_drive_test/include/cone_drive_test/core/longitudinal.hpp (1113ML) — auto_control로 옮김
// 종방향 단계 제어 (auto_control_node longitudinal_motor 이식, 신호등·정지선·코너 밴드 분기 제외):
// 목표 속도 → duty 감속 → 부족한 감속은 전류 제동 → 해제·회복, 정지 출발 시작 duty, 장애물 목표 램프
#pragma once
#include <memory>
#include <string>
#include "auto_control/drive/copied/control_core.hpp"

namespace auto_control::drive {

// duty-only 모드(2) 전용 값 (auto_control duty_only_* 와 같은 이름·기본값). 모드 1(staged) 튜닝에는 영향 없음
struct DutyOnlyParams {
  double kp{0.06}, ki{0.03}, kd{0.0}, integral_limit{1.0};       // duty/(m/s)
  double ff_offset_duty{0.01}, ff_duty_per_mps{0.05}, ff_fade_speed_mps{0.10};
  double maximum_duty{0.12}, start_duty{0.053};
  double duty_rise_rate_per_sec{0.04}, duty_fall_rate_per_sec{0.06};
  double planning_deceleration_mps2{0.40};          // 코너·차량 추종 감속 계획
  double obstacle_planning_deceleration_mps2{0.35}; // 장애물 목표 하강률
  double response_sec{0.10};                        // duty 명령 → 감속 시작 지연
  double coast_deceleration_mps2{0.50};             // 모터를 풀었을 때 굴러가며 줄어드는 감속도
  double coast_release_speed_mps{0.35}, coast_release_delay_sec{0.03};
  double hold_current_amps{0.0};                    // 정지 유지 전류 (0 = 안 씀)
  double emergency_deceleration_mps2{1.2}, emergency_overspeed_mps{0.5};   // 비상 제동 조건
  double emergency_brake_current_amps{2.0}, emergency_brake_rise_amps_per_sec{8.0}, emergency_brake_fall_amps_per_sec{8.0};
};

struct LongitudinalParams {
  int mode{1};                                    // 1 = staged (auto_control 기본), 2 = duty_only. 바꾸면 정지했을 때 적용
  DutyOnlyParams duty_only;
  double minimum_speed_mps{1.0}, maximum_speed_mps{1.8};
  double obstacle_speed_mps{1.0};                 // 장애물 구간 도달 속도
  double obstacle_planning_deceleration_mps2{0.35}, obstacle_planning_acceleration_mps2{0.35};
  double obstacle_coast_probe_sec{0.20};
  double obstacle_brake_entry_margin_mps{0.10}, obstacle_brake_exit_margin_mps{0.03};
  double obstacle_brake_deceleration_margin_mps2{0.10};
  double longitudinal_brake_release_response_sec{0.15};
  double recovery_start_ramp_sec{0.20};
  double planning_acceleration_mps2{0.8};         // 목표 상승 상한 (corner_planning_acceleration)
  double pid_kp{1.0}, pid_ki{0.5}, pid_kd{0.0}, pid_integral_limit{1.5};
  double brake_speed_gain{0.8}, brake_deceleration_gain{0.5};
  double maximum_duty{0.12};
  double duty_rise_rate_per_sec{0.04}, duty_fall_rate_per_sec{0.03};
  double recovery_duty_rise_per_sec{0.045};
  double longitudinal_start_duty{0.053};
  double feedforward_offset_duty{0.01}, feedforward_duty_per_mps{0.05}, feedforward_fade_speed_mps{0.10};
  double speed_filter_time_constant_sec{0.1}, deceleration_filter_sec{0.20};
  bool electrical_brake_enabled{true};
  double brake_entry_speed_error_mps{0.3}, brake_exit_speed_error_mps{0.1};
  double brake_maximum_current_amps{2.0};
  double brake_current_rise_amps_per_sec{1.5}, brake_current_fall_amps_per_sec{6.0};
  double response_time_sec{0.15};                 // traffic_response_time 대체 (coast 추정 안정 시간)
};

struct LongitudinalCommand {
  double duty{0}, brake_current{0}, target_speed{0};
  std::string phase{"idle"};
  bool brake_mode{false};
};

class LongitudinalController {
public:
  explicit LongitudinalController(const LongitudinalParams & p);
  void set_params(const LongitudinalParams & p);
  void reset();
  void on_speed(double raw_speed_mps, double t);   // ERPM 표본: 속도 필터·감속도 추정
  double speed() const { return speed_; }
  bool speed_valid() const { return have_speed_; }
  // plan_target: 곡률 속도 계획 목표, obstacle_active: 근처 콘, speed_cap: 롤아웃 실패 등 즉시 상한, stop: 정지 요청
  LongitudinalCommand update(double dt, double plan_target, bool obstacle_active, double speed_cap, bool stop);
  int active_mode() const { return active_mode_; }
  const std::string & phase() const { return phase_; }

private:
  LongitudinalCommand duty_only_update(double dt, double target, bool obstacle_profile);
  void apply_requested_mode();
  LongitudinalParams p_;
  int active_mode_{1};
  std::unique_ptr<SpeedPid> duty_pid_;
  bool duty_emergency_{false};
  std::unique_ptr<SpeedPid> pid_;
  double speed_{0}, decel_{0}, coast_decel_{0};
  bool have_speed_{false}, have_decel_{false};
  double last_t_{0}, coast_seen_t_{-1e9}, last_brake_t_{-1e9}, now_t_{0};
  double duty_{0}, brake_{0}, last_target_{0}, coast_probe_{0};
  double obstacle_planned_{-1}, obstacle_requested_{0}, obstacle_plan_decel_{0};
  bool zero_target_{false}, brake_mode_{false}, service_brake_{false}, departure_pending_{true}, traction_recovery_{false};
  std::string phase_{"idle"};
};

}  // namespace auto_control::drive
