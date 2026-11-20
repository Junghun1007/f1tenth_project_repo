// Copied from src/cone_drive_test/include/cone_drive_test/core/visibility.hpp (1113ML) — auto_control로 옮김
// 기억 콘 중 "지금 보여야 하는" 것 (miss 판정용). 시뮬레이터의 픽셀 래스터 대신 간단한 기하 판정:
// 수평 화각 안, 화면 세로 방향으로 콘 면적의 threshold 이상이 ROI 안(아래쪽 roi), 더 가까운 콘에 threshold 이상 가리지 않음.
#pragma once
#include <vector>
#include "auto_control/drive/core/types.hpp"

namespace auto_control::drive {

struct CameraGeometry {
  double camera_x{-0.09};          // 앞차축 기준
  double height{0.20};
  double pitch_rad{13.0 * 3.14159265358979323846 / 180.0};   // 아래로 +
  double hfov_rad{69.0 * 3.14159265358979323846 / 180.0};
  double aspect{800.0 / 1280.0};   // 세로/가로
  double roi_height_ratio{0.65};   // 화면 아래에서부터 검출 영역
  double cone_height{0.18};
  double max_range{3.0};
  double threshold{0.8};
};

// 화각을 실제 카메라(검출 영상) 내부 파라미터로: 가로 화각 = 2·atan(폭/2 / fx), 세로/가로 비 = (높이/2 / fy) / (폭/2 / fx).
// 기본값(69°, 800/1280)은 실제(약 97°, 아래쪽 36°)보다 좁아, 0.2~0.42 m 앞 콘을 '못 보는 위치'로 판정했다
// → 안 보여도 놓침이 안 쌓이고 사각지대 보호로 유령 정리도 안 돼 정지 중 계속 남았다
void set_field_of_view(CameraGeometry & g, double fx, double fy, int width, int height);

std::vector<int> expected_visible(const std::vector<Vec2> & cones_local, const CameraGeometry & g, double cone_size);

}  // namespace auto_control::drive
