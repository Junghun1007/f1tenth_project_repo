// Copied from src/cone_drive_test/include/cone_drive_test/core/control_loop.hpp (1113ML) — auto_control로 옮김
// 주행 프로세스의 판단부 전체 (ROS·DepthAI·CUDA 없음): 센서 입력 → 콘 기억 → 회피 계획 → Stanley·종방향 제어
// → 상태기계 → 자동 명령과 안전 프로세스 요청, 텔레메트리. 모든 공개 함수는 스레드 안전.
#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include "auto_control/drive/copied/control_core.hpp"
#include "auto_control/drive/core/drive_state.hpp"
#include "auto_control/drive/core/lane_memory.hpp"
#include "auto_control/drive/core/path_utils.hpp"
#include "auto_control/drive/core/odometry.hpp"
#include "auto_control/drive/core/path_lock.hpp"
#include "auto_control/drive/core/lead_follower.hpp"
#include "auto_control/drive/core/traffic_stop.hpp"
#include "auto_control/drive/core/settings.hpp"
#include "auto_control/drive/core/telemetry.hpp"

namespace auto_control::drive {

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

// 콘 관측 기록 한 줄 (처음 기억한 위치와 최근 위치가 틀어지는 원인 분석용)
struct ConeLogRow {
  double t_rgb{0}, t_depth{-1}, t_pose{0};   // 박스 시각, 깊이 시각, 자세를 찾은 시각
  std::uint32_t id{0};
  int class_id{0};
  double local_x{0}, local_y{0}, front_x{0}, front_y{0}, range{0}, bearing{0};   // 차량 좌표 (앞차축 기준), range·bearing은 카메라 기준 앞면
  double pose_x{0}, pose_y{0}, pose_yaw{0};   // odom 앞차축 자세
  double speed{0}, yaw_rate{0}, gyro_bias{0};
  double world_x{0}, world_y{0};              // 이 관측 하나의 월드 위치
  double mem_x{0}, mem_y{0};                  // 갱신 후 기억 위치
  int hits{0};
};

// 수동 주행 녹화 한 프레임 (차선 결과마다): 사람이 달린 경로와 같은 순간 계획기가 낸 경로를 함께 남긴다
struct RecordFrame {
  double t{0};                         // 차선 영상 촬영 시각
  Pose2 pose;                          // 그 시각 odom 앞차축 자세
  double speed{0}, yaw_rate{0};
  std::string state;                   // DRIVING / MANUAL ...
  bool lane_valid{false};
  std::vector<Vec2> left, right, path; // 차선 (차량 좌표, 그 시각 기준)
  std::vector<Vec2> center;            // 코너 바깥 이동 없는 중앙선 (콘이 근처일 때만, 계획기의 기준선)
  std::string plan_mode;               // center / lane / lane-tight / free / fallback
  bool rollout_ok{true}, obstacle_active{false};
  std::vector<Vec2> plan_path;         // 계획기가 낸 경로 (차량 좌표)
  std::vector<Vec2> cones_world;       // 계획에 쓴 콘 (odom 좌표)
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
  // 터널(IR) 구간 (auto_control): 켜면 콘·차량 기억을 모두 비우고 그동안 검출을 받지 않는다
  void set_obstacles_suspended(bool on);
  void on_traffic_signal(TrafficSignal signal, double capture_time);   // 신호등 색 (검출 스레드)
  // 점검
  void set_attitude(const AttitudeInfo & a);
  void set_camera_geometry(const CameraGeometry & g);
  CameraGeometry camera_geometry() const { std::lock_guard<std::mutex> l(m_); return rc_.camera; }   // 측정한 높이·pitch를 '보여야 할 콘' 판정에 반영
  void set_model_status(int index, const InspectionModel & m);   // 0=차선, 1=YOLO, 2=터널 IR 차선
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
  // 콘 관측 기록 (검출이 계획에 반영될 때마다, 제어 스레드에서 호출). 비워 두면 기록 안 함
  std::function<void(const std::vector<ConeLogRow> &)> on_cone_log;
  // 수동 주행 녹화 (차선 결과마다, 제어 스레드에서). 비워 두면 기록 안 함
  std::function<void(const RecordFrame &)> on_record;
  double gyro_bias() const { return gyro_bias_.load(std::memory_order_relaxed); }

private:
  void clear_memories();   // 콘 기억(월드·차선 기준), 경로 고정, 보이는 콘, 계획 상태를 모두 비움. m_ 잡은 채로 호출
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
  void remember_vehicles(const DetectionFrame & det, std::optional<double> dist_at_capture);
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
  LeadFollower lead_;                        // 차량 추종
  TrafficStopper traffic_;                   // 신호등 정지
  std::vector<ConeObservation> new_vehicles_;   // 새 깊이 검출의 차량 (촬영 시점 좌표)
  double new_vehicles_dist_{0};                 // 그 촬영 시점 바퀴 누적 거리
  bool new_det_{false};
  AvoidancePlanner planner_;
  Odometry odom_;
  LongitudinalController lon_;
  SteeringFilter steer_;
  StageStats stats_;
  AttitudeInfo attitude_;
  bool obstacles_suspended_{false};
  InspectionModel models_[3];   // 0=차선, 1=YOLO, 2=터널 IR 차선 (이름이 비면 안 씀)
  bool vesc_connected_{false};
  double last_erpm_t_{-1e9}, last_lane_t_{-1e9}, last_control_t_{-1}, last_request_t_{-1e9}, last_yaw_t_{-1};
  double speed_{0};
  double odom_speed_{0};
  std::atomic<double> speed_atomic_{0.0};
  std::vector<Vec2> rec_cones_;   // 녹화용: 이번 계획에 쓴 콘
  // 자이로 영점: 정지 중(속도 ≈ 0, ERPM 신선) 요레이트 평균. 오도메트리에서 뺀다 (안 빼면 약 0.15°/s씩 방향이 돈다)
  std::atomic<double> gyro_bias_{0.0}, last_erpm_atomic_{-1e9}, last_yaw_rate_{0.0};
  bool gyro_bias_init_{false};
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

}  // namespace auto_control::drive
