// Copied from src/cone_drive_test/include/cone_drive_test/core/traffic_stop.hpp (1113ML) — auto_control로 옮김
// 신호등 정지 (traffic.enabled, 1030ML auto_control + traffic_detection_test 방식).
//  - 신호 색: multiclass YOLO의 traffic_light 박스 중 점수가 가장 높은 것의 픽셀을 HSV로 보고 빨강/초록 에너지 비교 (룰베이스)
//  - 정지선: 차선 모델의 정지선 채널 + 중앙선 교차점 거리(line_detactor stop_line_distance). 연속 확인 후 추적,
//    관측 사이와 차 아래로 들어간 뒤에는 바퀴 이동 거리로 남은 거리를 이어 간다 (근접 추적)
//  - RED가 보이고 정지선이 추적 중이면 정지에 들어가고(commit), 앞범퍼가 정지선 앞 0~stop_range_max 안에 서도록 속도 상한.
//    들어간 뒤에는 빨간불이 안 보여도 유지하고 GREEN을 봐야 출발. UNKNOWN(신호등 안 보임)이면 정지하지 않는다
#pragma once
#include <cstdint>
#include <limits>
#include <opencv2/core.hpp>

namespace auto_control::drive {

enum class TrafficSignal : std::uint8_t { Unknown = 0, Red = 1, Green = 2 };

// BGR 박스 영상 → 신호 색 (traffic_detection_test traffic_light_node::color 와 같은 규칙)
TrafficSignal classify_traffic_color(const cv::Mat & bgr, int min_saturation, int min_value);

struct TrafficParams {
  bool enabled{true};
  double state_timeout{0.30};        // RED·GREEN 관측 유효 시간
  double line_timeout{0.50};         // 정지선을 못 봐도 추적을 유지하는 시간
  int line_confirm{3};               // 정지선 관측 확인 횟수
  double match_tolerance{0.30};      // 예측(이동 거리 보정)과 관측의 일치 허용
  double current_weight{0.35};       // 새 관측 반영 비율
  double terminal_distance{0.10};    // 앞범퍼→정지선이 이 안이면 근접 추적 (안 보여도 이동 거리로)
  double terminal_timeout{1.50};
  double stop_range_max{0.25};       // 허용 정차 범위: 앞범퍼가 정지선 앞 0 ~ 이 거리
  double stop_margin{0.10};          // 목표 정차점: 정지선 앞 이 거리
  double front_overhang{0.05};
  double decel{0.8};                 // staged 감속 계획
  double reapproach_speed{0.20};     // 정차 범위보다 짧게 섰을 때 다시 다가가는 속도 상한
  double pass_overshoot{0.30};       // 정지선을 이만큼 지나치면 정지 해제
  double line_lost_release{2.0};     // 정지 중 정지선을 놓치고 빨간불도 안 보이는 채로 이만큼 지나면 해제
  bool duty_only{false};
  double coast_decel{0.5}, release_speed{0.35}, release_delay{0.03}, response{0.10};
};

struct TrafficState {
  TrafficSignal signal{TrafficSignal::Unknown};   // 유효 시간 안의 마지막 신호
  bool line_tracked{false};
  double line_m{std::numeric_limits<double>::quiet_NaN()};   // 앞범퍼 → 정지선 (추적 중)
  bool committed{false};   // 빨간불 정지 중 (GREEN 볼 때까지)
  bool holding{false};     // 정차 범위 안에 서 있음
  double cap{std::numeric_limits<double>::infinity()};
};

class TrafficStopper {
public:
  void set_params(const TrafficParams & p) { p_ = p; if (!p.enabled) reset(); }
  void reset() { s_ = TrafficState{}; cand_n_ = 0; red_t_ = green_t_ = -1e9; }
  void on_signal(TrafficSignal sig, double t);   // 검출 프레임마다 (UNKNOWN도 전달)
  // stop_line_m: 이번 차선 프레임의 앞차축 → 정지선 거리 (NaN = 없음), t: 그 촬영 시각, dist: 그 시각 바퀴 누적 거리
  const TrafficState & update(double stop_line_m, double t, double dist, double v_ego);
  const TrafficState & state() const { return s_; }

private:
  double cap_for(double v_ego);
  TrafficParams p_;
  TrafficState s_;
  double red_t_{-1e9}, green_t_{-1e9};
  TrafficSignal last_sig_{TrafficSignal::Unknown};
  double sig_t_{-1e9};
  double meas_line_{0}, meas_dist_{0}, meas_t_{0};   // 앞차축 기준
  double cand_line_{0}, cand_dist_{0};
  int cand_n_{0};
  double lost_t_{-1e9};
};

}  // namespace auto_control::drive
