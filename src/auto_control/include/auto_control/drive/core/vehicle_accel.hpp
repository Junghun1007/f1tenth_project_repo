// Copied from src/cone_drive_test/include/cone_drive_test/core/vehicle_accel.hpp (1113ML) — auto_control로 옮김
// semi_slam/core/vehicle_accel 과 같음 (저주파 IMU 보정)
// 차가 만드는 가속도 모델 (IMU 위치, 차량 좌표 x 앞 y 왼쪽 z 위). 가속도계 값에서 이걸 빼면 중력만 남는다 → 주행 중에도 기울기 보정에 쓸 수 있음.
//   ax = dv/dt − ω²·r          (앞뒤 가속 + 회전 중심에서 r 앞에 있어서 생기는 안쪽 성분)
//   ay = v·ω + (dω/dt)·r       (원심 + 회전이 빨라지거나 느려질 때 성분)
//   r: 뒤 차축(회전 중심)에서 IMU(카메라)까지 앞쪽 거리
// 기록 재생 (semi_slam 7개, 이 클래스와 같은 실시간 필터, 0.25 s 평균 기준): 기울기 흔들림 roll 2.8~9.0° → 0.9~1.5°,
// pitch 1.3~3.2° → 0.8~1.2°. 가속도계와의 시간 맞춤은 효과 없음 (±0.01 s)
// 속도는 ERPM 콜백, 요레이트·조회는 IMU 스레드에서 들어와 내부에서 잠근다.
#pragma once
#include <mutex>
#include <optional>
#include "auto_control/drive/core/types.hpp"

namespace auto_control::drive {

struct Vec3d { double x{0}, y{0}, z{0}; };

class VehicleAccelModel {
public:
  explicit VehicleAccelModel(double lever_m = 0.24) : r_(lever_m) {}
  void set_lever(double r) { std::lock_guard<std::mutex> l(m_); r_ = r; }
  void on_speed(double v, double t);       // 보정 적용한 속도 (m/s)
  void on_yaw_rate(double w, double t);    // 영점 뺀 요레이트 (rad/s)
  // 속도가 0.3 s 넘게 안 들어왔으면 없음 (그때는 보정하지 않음)
  std::optional<Vec3d> accel(double t) const;

private:
  mutable std::mutex m_;
  double r_;
  double v_{0}, vd_{0}, v_t_{-1};
  double w_{0}, wd_{0}, w_t_{-1};
  bool v_init_{false}, w_init_{false};
};

}  // namespace auto_control::drive
