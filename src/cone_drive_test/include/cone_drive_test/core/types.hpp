// 공통 타입. 좌표: 앞차축 기준 차량 좌표 x 앞, y 왼쪽 (m), 각도 rad.
#pragma once
#include <vector>

namespace cone_drive_test {

struct Vec2 { double x{0}, y{0}; };
struct Pose2 { double x{0}, y{0}, yaw{0}; };   // odom 좌표, 앞차축

struct ConeObservation {      // 관측 시점 차량 좌표
  Vec2 center;
  Vec2 front;
  double bearing{0};
  double range{0};
  int class_id{0};
  float score{0};
  double length{0.13}, width{0.13};
};

struct LaneObservation {
  std::vector<Vec2> path;     // 제어용 중앙선 (코너 바깥 이동 포함)
  std::vector<Vec2> center;   // 바깥 이동 없는 중앙선 (근처에 콘이 있을 때만 채움, 없으면 빈 벡터)
  std::vector<Vec2> left, right;
  int side_relabeled{0}, side_dropped{0};   // 차선 좌우 판정 고정: 이번 프레임 교정·제외 조각 수
  double capture_time{0};
  bool valid{false};
};

struct DetectionFrame {
  double capture_time{0};
  bool depth_valid{true};   // 짝이 맞는 깊이가 없으면 false → 위치를 못 구했으므로 "안 보임"으로 세지 않는다
  std::vector<ConeObservation> cones;
};

double now_sec();   // steady clock (s)

}  // namespace cone_drive_test
