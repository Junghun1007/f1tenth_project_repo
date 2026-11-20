// semi_slam 핵심: 정지선 S와 바퀴 수만으로 오도메트리를 보정하고, 트랙 중앙선 위 위치를 추정한다.
//
//  위치 적분   x += v·k(v)·cos(ψ)·dt,  ψ += g·(ω − b0)·dt      k(v): 속도별 거리 배율, g: 자이로 배율, b0: 출발 직전 정지 영점
//  보정        트랙 모양(테이프 휨·폭 차이)과 주행 라인에 영향받지 않는 기준만 쓴다
//              · 자이로: S → S 한 바퀴 회전량 = 정확히 360°
//              · 바퀴:   S → S 거리 = 지도 중앙선 한 바퀴 길이 (주행 라인 차이로 ±1% 남음)
//              여러 주행의 바퀴 수 가중 평균으로 갱신 (한 번에 덮어쓰지 않음)
//  위치 추정   진행 방향 = 마지막 S 통과 후 거리로 중앙선 위 지점, 옆 = BEV 차 바로 앞(0.3 m) 차선 가운데, 방향 = 자이로
//              (자유 적분 궤적도 비교용으로 따로 유지)
//  지도 맞춤   차선 점과 정답 흰 선 거리 RMS는 참고용 (보정 기준 아님)
// ROS·DepthAI 없이 동작 (호스트 테스트 가능). SlamCore의 공개 함수는 스레드 안전.
#pragma once
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include "semi_slam/core/track_map.hpp"
#include "semi_slam/core/types.hpp"

namespace semi_slam {

struct OdomCalib {
  static constexpr int kVersion = 2;   // 1 = 예전 (Nelder–Mead, 카메라 각도 포함) → 읽으면 version 1로 표시
  int version{kVersion};
  std::map<int, double> wheel_scale;   // 키: 속도 (cm/s), 값: 거리 배율. 사이는 선형 보간, 밖은 끝값, 비면 1
  std::map<int, int> wheel_laps;       // 그 배율에 쓰인 바퀴 수 (누적 평균 가중치)
  double gyro_scale{1.0};
  int gyro_laps{0};
  double gyro_bias{0.0};    // rad/s. 잔여 영점 (같은 속도 주행으로는 배율과 구분이 안 돼 추정하지 않음, 0)
  double latency{0.0};      // s. 차선 영상 시각 + latency = 그 장면을 본 위치의 시각 (설정값)
  double wheel_scale_for(double v) const;
  void set_wheel_scale(double v, double scale);
  static int speed_key(double v);   // 5 cm/s 단위
  std::string to_json() const;
  static OdomCalib from_json(const std::string & text);   // 형식 오류면 기본값
};

struct SpeedSample { double t, v; };        // ERPM에서 구한 속도 (보정 전)
struct GyroSample { double t, w; };         // 차량 요레이트 rad/s (보정 전, 반시계 +)
struct ImuSample {                          // IMU 원값 6축, 차량 좌표 (x 앞 y 왼쪽 z 위). 가속도는 시각이 안 맞으면 NaN
  double t{0};
  double ax{0}, ay{0}, az{0};               // m/s² (가속도계 = 차 가속도 − 중력)
  double gx{0}, gy{0}, gz{0};               // rad/s
};
struct LaneSample {                         // 차선 결과 (앞차축 기준 차량 좌표, 관측 범위 안만)
  double t{0}, stop_m{0};                   // stop_m: 앞차축 → 정지선, 없으면 NaN
  std::vector<Vec2> left, right;
};
struct Pose { double t{0}, x{0}, y{0}, yaw{0}, dist{0}; };

struct RunData {
  Pose start;
  double gyro_bias0{0};
  double target_speed{0};
  int laps_target{0};
  bool stopline_correction{true};
  bool cw{false};                           // 주행 방향: false = 반시계, true = 시계
  std::vector<SpeedSample> speed;
  std::vector<GyroSample> gyro;
  std::vector<ImuSample> imu;               // 저주파 IMU 보정(가속도에서 중력 분리) 분석용 기록. 보정 계산에는 쓰지 않음
  std::vector<LaneSample> lanes;
  std::vector<double> crossing_t;           // 정지선 S 통과 시각 (바퀴 경계). 기록은 첫 S 통과(start.t)부터
  double crossing_latency{0};
  // 정지선 S에 다가가며 BEV 양쪽 차선으로 잰 값: x = 차선 기울기 각 atan(a), y = 두 선 가운데의 차 기준 옆 위치. 없으면 NaN
  Vec2 start_rel{std::numeric_limits<double>::quiet_NaN(), 0};
  std::vector<Vec2> anchor_rel;
  std::map<std::string, double> params;     // 주행 당시 주요 파라미터 (기록용)
};

class Trajectory {
public:
  std::vector<Pose> poses;
  Pose at(double t) const;                  // 선형 보간 (범위 밖은 끝값)
};

// 기록을 보정 값으로 다시 적분 (자유 적분). stopline_correction이면 S 통과 시각마다 진행 방향 위치를 S에 맞춘다
Trajectory integrate(const RunData & run, const OdomCalib & calib, const TrackMap & map, bool stopline_correction);

struct LapMetrics {
  int lap{0};
  double dist_m{0};           // 이 바퀴 주행 거리 (보정 적용)
  double s_err_m{0};          // S 도착 오차: 주행 거리 − 중앙선 한 바퀴 (+ = 거리로는 S를 지나서야 실제 S)
  double turn_deg{0};         // 이 바퀴 회전량 (보정 적용, 정답 360)
  double lat_mean_m{0};       // 평균 옆 위치 (+ = 중앙선보다 진행 방향 왼쪽)
  double along_err_m{0};      // 자유 적분 궤적의 S 통과 순간 진행 방향 오차 (참고)
  double heading_err_deg{0};  // 자유 적분 궤적의 S 통과 순간 방향 오차 (참고, 차선 대비 비스듬함 포함)
  double length_err{0};       // dist_m / 중앙선 − 1
  double rms_m{0};            // 그 바퀴 인식 선과 정답 선 거리 RMS (참고)
  int points{0};
};
struct Evaluation {
  double rms_m{0}, accuracy{0};   // accuracy = 1 − rms / 차선 반폭 (참고)
  double stop_rms_m{0};
  int points{0};
  double cost{0};
  std::vector<LapMetrics> laps;
};
Evaluation evaluate(const RunData & run, const OdomCalib & calib, const TrackMap & map, double max_range_m = 1.5, double cap_m = 0.15);

// 한 주행에서 직접 구한 값 (보정 전 원값 기준)
struct OdomEstimate {
  bool ok{false};
  std::string message;
  int laps{0};
  double gyro_scale{std::numeric_limits<double>::quiet_NaN()};    // 360° / 원값 회전량
  double wheel_scale{std::numeric_limits<double>::quiet_NaN()};   // 중앙선 길이 / 원값 거리
  double raw_turn_deg_per_lap{0}, raw_dist_per_lap{0}, ref_length{0};
  // 주행 중 검산: 직선에서 BEV 차선 각도 − (옆으로 밀리는 속도로 구한 실제 비스듬함) = BEV가 더 기울어 보이는 각 (0이면 카메라 yaw 맞음)
  double bev_yaw_residual_deg{std::numeric_limits<double>::quiet_NaN()};
  double straight_angle_deg{std::numeric_limits<double>::quiet_NaN()}, straight_slip_deg{std::numeric_limits<double>::quiet_NaN()};
  int straights{0};
};
OdomEstimate estimate_odometry(const RunData & run, const TrackMap & map);

struct CalibResult {
  OdomCalib calib;          // 갱신 후 값 (applied가 false면 이전 값 그대로)
  OdomEstimate est;
  bool applied{false};
  bool gyro_used{false}, wheel_used{false};
  std::string message;
  Evaluation before, after;   // 참고: 지도 맞춤 RMS
};
// 이전 값과 바퀴 수 가중 평균 (이전 가중치는 최대 max_prev_laps). 이전 값과 outlier_ratio 넘게 다르면 그 항목은 반영하지 않음
CalibResult calibrate(const RunData & run, const OdomCalib & current, const TrackMap & map, double max_range_m = 1.5,
                      int max_prev_laps = 30, double outlier_ratio = 0.02);

struct SlamConfig {
  double max_range_m{1.5};          // 매핑에 쓰는 차선 점 최대 거리
  bool stopline_correction{true};
  int stop_confirm{3};              // 정지선 관측 확인 횟수
  double stop_tolerance_m{0.25};
  // 정지선 오인식(코너에서 휜 흰 선 등)을 바퀴로 세지 않게 두 가지로 거른다
  double stop_gate_m{1.2};          // 관측한 정지선의 중앙선 위 위치가 S에서 이 거리 안일 때만
  double lap_window{0.25};          // 직전 통과 후 주행 거리가 한 바퀴 길이의 (1 ± 이 비율) 안일 때만 통과로 셈
};

struct LiveState {
  bool running{false}, finished{false};
  bool cw{false};
  bool waiting_s{true};             // 정지선 S를 처음 지나기 전: 가속만 (기록·매핑·바퀴 세기는 S부터)
  int lap{0}, laps_target{0};
  double target_speed{0};
  Pose pose;                        // 위치 추정 (중앙선 위 지점 + 옆 위치, 방향은 자이로)
  Pose free_pose;                   // 자유 적분 (비교용)
  double s_arc{0};                  // 중앙선 위 위치 (m, 반시계 기준)
  double lat{0};                    // 옆 위치 (+ = 진행 방향 왼쪽)
  bool lat_valid{false};
  double lap_dist{0};               // 이번 바퀴 주행 거리
  double lap_length{0};             // 중앙선 한 바퀴
  double bias0{0};
  double stop_seen_m{std::numeric_limits<double>::quiet_NaN()};   // 마지막으로 본 정지선 거리 (앞차축 기준)
  bool stop_near_s{false};          // 그 정지선이 중앙선 위에서 S 근처인지
  double lane_width{std::numeric_limits<double>::quiet_NaN()};   // BEV로 잰 차선 폭
  int rejected_far{0};              // S와 멀어서 무시한 정지선 (오인식)
  int rejected_lap{0};              // 바퀴 거리가 안 맞아 무시한 통과
};

class SlamCore {
public:
  explicit SlamCore(TrackMap map) : map_(std::move(map)) {}
  const TrackMap & map() const { return map_; }
  void set_config(const SlamConfig & c);
  void set_calib(const OdomCalib & c);
  OdomCalib calib() const;

  void begin_run(double target_speed, int laps, const Pose & start, bool cw = false);
  void set_run_params(const std::map<std::string, double> & p);
  void end_run();
  void on_speed(double v, double t);
  void on_gyro(double w, double t);
  void on_imu(const ImuSample & s);   // 기록만 (S를 지난 뒤부터, 주행 중)
  // 차선 결과마다. 이번 프레임에서 목표 바퀴 수를 채웠으면 true
  bool on_lane(const LaneObservation & lane);

  LiveState live() const;
  std::vector<LapMetrics> lap_metrics() const;
  std::vector<Vec2> take_map_points(std::vector<int> * side);   // 새로 매핑한 점 (side 0 왼쪽, 1 오른쪽)
  std::vector<Pose> track_since(std::size_t * cursor) const;   // 화면용 위치 추정 궤적 (20 Hz)
  std::vector<Pose> free_since(std::size_t * cursor) const;    // 화면용 자유 적분 궤적 (20 Hz)
  RunData run_data() const;
  std::vector<std::pair<int, double>> corrections() const;    // (바퀴, 정지선 위치 보정량 m)

private:
  void step(double t);
  Pose pose_at(double t) const;                // 최근 자유 적분 이력 보간 (락 안에서)
  double arc_at_dist(double dist) const;       // 누적 거리 → 중앙선 위 위치
  Pose snap_of(const Pose & free) const;       // 자유 적분 자세 → 위치 추정 (중앙선 위 지점 + 옆 위치, 방향은 그대로)
  void on_crossing(double t);
  Vec2 take_lane_rel(double yaw_now);   // S 접근 중 잰 차선 대비 방향을 지금(통과 순간)으로 옮긴 평균 (그 사이 자이로 회전만큼)
  TrackMap map_;
  SlamConfig cfg_;
  OdomCalib calib_;
  mutable std::mutex m_;
  RunData run_;
  LiveState live_;
  double last_v_{0}, last_gyro_t_{-1}, last_w_{0};
  double bias_ema_{0};
  bool bias_init_{false};
  std::deque<Pose> hist_;
  std::vector<Pose> track_, free_track_;
  double track_last_t_{-1};
  std::vector<Vec2> new_pts_;
  std::vector<int> new_side_;
  double anchor_s_{0}, anchor_dist_{0}, s_stop_{0};        // 중앙선 위치 기준점 (마지막 S 또는 출발점)
  // 정지선 통과 예측
  int stop_n_{0};
  double stop_cand_{0}, armed_dist_{-1}, last_cross_dist_{0};
  std::vector<LapMetrics> laps_;
  std::vector<std::pair<int, double>> corr_;
  double lap_sq_{0}, lap_turn_{0}, lap_lat_sum_{0};
  int lap_n_{0}, lap_lat_n_{0};
  bool finish_reported_{false};
  bool last_rejected_{false};
  int lane_rel_n_{0};
  double lane_rel_yaw_{0}, lane_rel_lat_{0}, lane_rel_ref_{0};
};

}  // namespace semi_slam
