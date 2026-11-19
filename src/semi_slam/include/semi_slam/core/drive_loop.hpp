// semi_slam 주행 판단부 (ROS·DepthAI 없음): 차선 중앙선을 일정 속도로 따라간다 (Stanley + duty-only/staged 종방향).
// 정지하면 안전 프로세스에 수동 전환을 요청한다 (cone_drive_test와 같은 상태기계). 공개 함수는 스레드 안전.
#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <utility>
#include "semi_slam/copied/control_core.hpp"
#include "semi_slam/core/drive_state.hpp"
#include "semi_slam/core/settings.hpp"
#include "semi_slam/core/stage_stats.hpp"

namespace semi_slam {

struct RuntimeConfig {   // 재시작해야 바뀌는 값 (ROS 파라미터)
  double wheelbase{0.33};
  double wheel_diameter_m{0.1095};
  int motor_pole_pairs{2}, motor_pinion_teeth{13}, spur_gear_teeth{54}, differential_pinion_teeth{13}, differential_ring_teeth{37};
  double erpm_direction_sign{1.0}, speed_scale_correction{1.0};
  SteeringParams steering;
  double erpm_timeout_sec{0.2};
  double path_min_span_m{0.3}, path_max_gap_m{0.25}, geometry_window_m{0.12};
  double handover_request_period_sec{0.2};
};

struct AttitudeInfo { bool ok{false}; double height_m{0}, roll_deg{0}, pitch_down_deg{0}; std::string status{"WAITING"}; std::string message; };
struct InspectionModel { std::string name, status, stage, message; double elapsed_s{0}; };

struct StepOutput {
  bool publish{false};
  double duty{0}, brake_current{0}, servo{0.5};
  bool brake_mode{false};
  std::string handover_request;   // "auto" | "manual" | ""
};

struct VehicleState { double speed{0}, target{0}, steer_deg{0}, cte{0}, duty{0}, brake{0}; std::string phase; int lon_mode{2}; };

class DriveLoop {
public:
  explicit DriveLoop(const RuntimeConfig & rc);
  void apply_settings(const Settings & s);
  Settings settings() const { std::lock_guard<std::mutex> l(m_); return s_; }

  double on_erpm(int erpm, double t);   // 반환: 계산한 속도 (m/s, 보정 전)
  void on_speed(double v, double t);
  void on_vesc_connected(bool c);
  void on_handover_mode(const std::string & mode, double t);
  void set_attitude(const AttitudeInfo & a);
  void set_model_status(const InspectionModel & m);
  void on_inspection_done(bool ok, const std::string & reason, double t);

  StepOutput on_lane(const LaneObservation & lane, double now);
  StepOutput tick(double now);

  std::pair<bool, std::string> request_start(double now);
  void request_stop(const std::string & reason, double now);
  void on_heartbeat(const std::string & client, double now);

  DriveState state() const;
  double speed() const { return speed_atomic_.load(std::memory_order_relaxed); }
  StageStats & stats() { return stats_; }

  // 텔레메트리용 스냅샷
  struct Snapshot {
    std::string state, reason;
    Readiness readiness;
    AttitudeInfo attitude;
    InspectionModel model;
    VehicleState vehicle;
    LaneObservation lane;
  };
  Snapshot snapshot(double now) const;

private:
  Readiness readiness(double now) const;
  StepOutput control_step(double now, bool fresh_path);
  RuntimeConfig rc_;
  Settings s_;
  mutable std::mutex m_;
  DriveStateMachine sm_;
  LongitudinalController lon_;
  SteeringFilter steer_;
  StageStats stats_;
  AttitudeInfo attitude_;
  InspectionModel model_{"차선 (Fast-SCNN)", "WAITING", "", "", 0};
  LaneObservation lane_;
  double last_lane_t_{-1e9}, last_erpm_t_{-1e9}, last_control_t_{-1}, last_request_t_{-1e9};
  double speed_{0}, target_{0}, cte_{0};
  std::atomic<double> speed_atomic_{0};
  bool vesc_connected_{false};
  StepOutput last_out_;
  std::string phase_;
};

}  // namespace semi_slam
