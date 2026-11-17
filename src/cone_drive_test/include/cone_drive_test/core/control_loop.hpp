// 주행 프로세스의 판단부 전체 (ROS·DepthAI·CUDA 없음): 센서 입력 → 콘 기억 → 회피 계획 → Stanley·종방향 제어
// → 상태기계 → 자동 명령과 안전 프로세스 요청, 텔레메트리. 모든 공개 함수는 스레드 안전.
#pragma once
#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include "cone_drive_test/copied/control_core.hpp"
#include "cone_drive_test/core/drive_state.hpp"
#include "cone_drive_test/core/lane_memory.hpp"
#include "cone_drive_test/core/path_utils.hpp"
#include "cone_drive_test/core/odometry.hpp"
#include "cone_drive_test/core/path_lock.hpp"
#include "cone_drive_test/core/settings.hpp"
#include "cone_drive_test/core/telemetry.hpp"

namespace cone_drive_test {

std::vector<ConeObservation> obstacle_cells(const std::vector<ConeObservation> & obs, double cone_size, bool vehicles);

struct RuntimeConfig {   // 재시작해야 바뀌는 값 (ROS 파라미터)
  double wheelbase{0.33};
  double wheel_diameter_m{0.1095};
  int motor_pole_pairs{2}, motor_pinion_teeth{13}, spur_gear_teeth{54}, differential_pinion_teeth{13}, differential_ring_teeth{37};
  double erpm_direction_sign{1.0}, speed_scale_correction{1.0};
  SteeringParams steering;   // servo 매핑, 최대 조향각
  CameraGeometry camera;     // 보여야 할 콘 판정
  double lane_timeout_sec{0.0}, erpm_timeout_sec{0.2};   // lane_timeout 0 = 제한 없음
  double path_min_span_m{0.3}, path_max_gap_m{0.25}, geometry_window_m{0.12};
  double handover_request_period_sec{0.2};
};

struct AttitudeInfo { bool ok{false}; double height_m{0}, roll_deg{0}, pitch_down_deg{0}; std::string status{"WAITING"}; std::string message; };

struct StepOutput {
  bool publish{false};          // 자동 명령 발행 (안전 프로세스가 AUTO일 때 전달)
  double duty{0}, brake_current{0}, servo{0.5};
  bool brake_mode{false};
  std::string handover_request; // "auto" | "manual" | ""
};

class ControlLoop {
public:
  ControlLoop(DriveMode mode, const RuntimeConfig & rc);
  void apply_settings(const Settings & s);
  Settings settings() const { std::lock_guard<std::mutex> l(m_); return s_; }

  // 센서
  void on_erpm(int erpm, double t);
  void on_speed(double speed_mps, double t);
  void on_yaw_rate(double yaw_rate, double t);
  void on_vesc_connected(bool connected, double t);
  void on_handover_mode(const std::string & mode, double t);
  void on_detections(const DetectionFrame & frame);
  // 점검
  void set_attitude(const AttitudeInfo & a);
  void set_camera_geometry(const CameraGeometry & g);
  CameraGeometry camera_geometry() const { std::lock_guard<std::mutex> l(m_); return rc_.camera; }   // 측정한 높이·pitch를 '보여야 할 콘' 판정에 반영
  void set_model_status(int index, const InspectionModel & m);   // 0=차선, 1=YOLO
  void on_inspection_done(bool ok, const std::string & reason, double t);
  void on_refit_done(bool ok, const std::string & reason, double t);
  // 제어 (차선 결과마다) / 감시 (차선이 오지 않을 때 주기적으로)
  StepOutput on_lane(const LaneObservation & lane, double now);
  StepOutput tick(double now);
  // 웹 요청
  std::pair<bool, std::string> request_start(double now);
  void request_stop(const std::string & reason, double now);
  std::pair<bool, std::string> request_refit(double now);
  std::pair<bool, std::string> request_mode(DriveMode m, double now);
  void on_heartbeat(const std::string & client, double now);

  bool cones_nearby() const;
  bool refit_pending() const;
  DriveState state() const;
  double speed() const { return speed_atomic_.load(std::memory_order_relaxed); }   // 락 없음 (IMU 스레드에서 호출)
  std::uint64_t dropped_detections() const { std::lock_guard<std::mutex> l(m_); return dropped_; }
  TelemetryFrame telemetry(bool full, double now) const;
  StageStats & stats() { return stats_; }

private:
  Readiness readiness(double now) const;
  // 오도메트리는 400 Hz IMU 스레드가 갱신하므로 별도 락 (계획이 도는 동안 IMU·영상 안정화가 기다리지 않게).
  // 락 순서는 항상 m_ → odom_m_
  Pose2 odom_pose() const;
  std::optional<Pose2> odom_pose_at(double t) const;
  double odom_distance() const;
  std::optional<double> odom_distance_at(double t) const;
  // memory.mode = 1, 2: 차선 기준 단기 기억 갱신 → 계획에 쓸 콘(월드 좌표, pose_lane 기준).
  // blind_world가 있으면(모드 2) 사각지대 콘은 계획용에서 빼서 따로 담고, blind_end에 경로 고정 끝(s)을 넣는다
  std::vector<Vec2> lane_memory_cones(const Pose2 & pose_lane, std::vector<Vec2> * blind_world = nullptr, double * blind_end = nullptr);
  StepOutput control_step(double now, bool have_path);
  void integrate_detections();
  RuntimeConfig rc_;
  Settings s_;
  mutable std::mutex m_;
  mutable std::mutex odom_m_;   // odom_, odom_speed_, last_yaw_t_ 보호
  DriveStateMachine sm_;
  ConeMemory memory_;
  LaneConeMemory lane_mem_;
  double last_lane_dist_{-1};
  std::vector<MemoryInfo> lane_mem_view_;   // 화면용 (차량 좌표)
  PathLock lock_;                           // 모드 2: 화면 밖 콘 옆 경로 고정
  FrenetFrame lane_f_;                      // 이번 차선 프레임의 (s, l) 기준 (lane_memory_cones가 갱신)
  bool lane_f_valid_{false};
  std::vector<Vec2> latest_sl_;             // 모드 3: 가장 최근 검출 프레임의 콘 (s, l) — 기억 없음
  double latest_det_t_{-1};
  AvoidancePlanner planner_;
  Odometry odom_;
  LongitudinalController lon_;
  SteeringFilter steer_;
  StageStats stats_;
  AttitudeInfo attitude_;
  InspectionModel models_[2];
  bool vesc_connected_{false};
  double last_erpm_t_{-1e9}, last_lane_t_{-1e9}, last_control_t_{-1}, last_request_t_{-1e9}, last_yaw_t_{-1};
  double speed_{0};
  double odom_speed_{0};
  std::atomic<double> speed_atomic_{0.0};
  std::optional<DetectionFrame> pending_det_;
  DetectionFrame last_det_;
  LaneObservation lane_;
  PlanResult plan_;
  Pose2 plan_pose_;
  std::vector<Vec2> control_path_;
  double target_{0}, cte_{0};
  StepOutput last_out_;
  std::uint64_t dropped_{0};
};

}  // namespace cone_drive_test
