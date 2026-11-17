// 기억 콘 중 "지금 보여야 하는" 것 (miss 판정용). 시뮬레이터의 픽셀 래스터 대신 간단한 기하 판정:
// 수평 화각 안, 화면 세로 방향으로 콘 면적의 threshold 이상이 ROI 안(아래쪽 roi), 더 가까운 콘에 threshold 이상 가리지 않음.
#pragma once
#include <vector>
#include "cone_drive_test/core/types.hpp"

namespace cone_drive_test {

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

std::vector<int> expected_visible(const std::vector<Vec2> & cones_local, const CameraGeometry & g, double cone_size);

}  // namespace cone_drive_test
