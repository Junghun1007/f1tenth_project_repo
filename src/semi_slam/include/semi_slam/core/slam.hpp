// semi_slam 핵심: 바퀴 거리 + IMU 회전으로 위치를 적분하고, 정답 트랙과 비교해 오도메트리 보정 값을 찾는다.
//
//  위치 적분   x += v·k(v)·cos(ψ)·dt,  ψ += g·(ω − b0 − b)·dt
//              k(v): 속도별 거리 배율, g: 자이로 배율, b0: 출발 직전 정지 중 잰 영점, b: 잔여 영점
//  지도 비교   차선 인식의 왼쪽 선 → 정답 안쪽 흰 선, 오른쪽 선 → 바깥 흰 선 (반시계 주행).
//              중앙선(점선)은 정확한 중앙이 아니고 코너에선 한쪽 선 + 반폭 가정이라 쓰지 않는다
//  정지선 S    관측 거리로 통과 지점을 예측해 바퀴 수를 세고, (켜져 있으면) 통과할 때 진행 방향 위치를 바로잡는다
//  보정        주행이 끝나면 기록 전체로 (k(주행 속도), g, b, 카메라 시간차)를 Nelder–Mead로 찾는다
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
  std::map<int, double> wheel_scale;   // 키: 속도 (cm/s), 값: 거리 배율. 사이는 선형 보간, 밖은 끝값, 비면 1
  double gyro_scale{1.0};
  double gyro_bias{0.0};    // rad/s. 출발 직전 정지 측정 영점에 더하는 잔여
  double latency{0.0};      // s. 차선 영상 시각 + latency = 그 장면을 본 위치의 시각
  double wheel_scale_for(double v) const;
  void set_wheel_scale(double v, double scale);
  std::string to_json() const;
  static OdomCalib from_json(const std::string & text);   // 형식 오류면 기본값
};

struct SpeedSample { double t, v; };        // ERPM에서 구한 속도 (보정 전)
struct GyroSample { double t, w; };         // 차량 요레이트 rad/s (보정 전, 반시계 +)
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
  std::vector<SpeedSample> speed;
  std::vector<GyroSample> gyro;
  std::vector<LaneSample> lanes;
  std::vector<double> crossing_t;           // 정지선 S 통과 시각 (바퀴 경계)
  double crossing_latency{0};               // 통과 시각을 계산할 때 쓴 카메라 시간차 (다시 적분할 때 새 시간차만큼 옮긴다)
};

class Trajectory {
public:
  std::vector<Pose> poses;
  Pose at(double t) const;                  // 선형 보간 (범위 밖은 끝값)
};

// 기록을 보정 값으로 다시 적분. stopline_correction이면 S 통과 시각마다 진행 방향 위치를 S에 맞춘다
Trajectory integrate(const RunData & run, const OdomCalib & calib, const TrackMap & map, bool stopline_correction);

struct LapMetrics {
  int lap{0};
  double along_err_m{0};      // S 통과 순간 추정 위치의 진행 방향 오차 (+ = 앞)
  double heading_err_deg{0};
  double length_err{0};       // 바퀴 길이 / 정답 − 1
  double rms_m{0};            // 그 바퀴 인식 선과 정답 선 거리 RMS
  int points{0};
};
struct Evaluation {
  double rms_m{0}, accuracy{0};   // accuracy = 1 − rms / 차선 반폭(0.325)
  double stop_rms_m{0};
  int points{0};
  double cost{0};
  std::vector<LapMetrics> laps;
};
Evaluation evaluate(const RunData & run, const OdomCalib & calib, const TrackMap & map, double max_range_m = 1.5);

struct CalibResult {
  OdomCalib calib;
  Evaluation before, after;
  bool improved{false};
  int evaluations{0};
  std::string message;
};
CalibResult calibrate(const RunData & run, const OdomCalib & start, const TrackMap & map, double max_range_m = 1.5);

struct SlamConfig {
  double max_range_m{1.5};          // 매핑·보정에 쓰는 차선 점 최대 거리 (먼 점은 BEV 오차가 큼)
  bool stopline_correction{true};
  int stop_confirm{3};              // 정지선 관측 확인 횟수
  double stop_tolerance_m{0.25};
  // 정지선 오인식(코너에서 휜 흰 선 등)을 바퀴로 세지 않게 두 가지로 거른다
  double stop_gate_m{1.2};          // 관측한 정지선의 지도 위치가 정지선 S에서 이 거리 안일 때만
  double lap_window{0.25};          // 직전 통과 후 주행 거리가 한 바퀴 길이의 (1 ± 이 비율) 안일 때만 통과로 셈
};

struct LiveState {
  bool running{false}, finished{false};
  int lap{0}, laps_target{0};
  double target_speed{0};
  Pose pose;
  double lap_dist{0};               // 이번 바퀴 주행 거리
  double lap_length{0};
  double bias0{0};
  double stop_seen_m{std::numeric_limits<double>::quiet_NaN()};   // 마지막으로 본 정지선 거리 (앞차축 기준)
  bool stop_near_s{false};          // 그 정지선이 지도상 정지선 S 근처인지
  int rejected_far{0};              // 지도상 S와 멀어서 무시한 정지선 (오인식)
  int rejected_lap{0};              // 바퀴 거리가 안 맞아 무시한 통과
};

class SlamCore {
public:
  explicit SlamCore(TrackMap map) : map_(std::move(map)) {}
  const TrackMap & map() const { return map_; }
  void set_config(const SlamConfig & c);
  void set_calib(const OdomCalib & c);
  OdomCalib calib() const;

  void begin_run(double target_speed, int laps, const Pose & start);
  void end_run();
  void on_speed(double v, double t);
  void on_gyro(double w, double t);
  // 차선 결과마다. 이번 프레임에서 목표 바퀴 수를 채웠으면 true
  bool on_lane(const LaneObservation & lane);

  LiveState live() const;
  std::vector<LapMetrics> lap_metrics() const;
  std::vector<Vec2> take_map_points(std::vector<int> * side);   // 새로 매핑한 점 (side 0 왼쪽, 1 오른쪽)
  std::vector<Pose> track_since(std::size_t * cursor) const;   // 화면용 궤적 (20 Hz)
  RunData run_data() const;
  std::vector<std::pair<int, double>> corrections() const;    // (바퀴, 정지선 위치 보정량 m)

private:
  void step(double t);                         // gyro 시각까지 적분 (락 안에서)
  Pose pose_at(double t) const;                // 최근 이력 보간 (락 안에서)
  void on_crossing(double t);
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
  std::vector<Pose> track_;
  double track_last_t_{-1};
  std::vector<Vec2> new_pts_;
  std::vector<int> new_side_;
  // 정지선 통과 예측
  int stop_n_{0};
  double stop_cand_{0}, armed_dist_{-1}, last_cross_dist_{0};
  std::vector<LapMetrics> laps_;
  std::vector<std::pair<int, double>> corr_;
  double lap_sq_{0};
  int lap_n_{0};
  bool finish_reported_{false};
  bool last_rejected_{false};
};

}  // namespace semi_slam
