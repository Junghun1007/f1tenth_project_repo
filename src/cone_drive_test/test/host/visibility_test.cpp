#include "check.hpp"
#include "cone_drive_test/core/visibility.hpp"
#include <cmath>

using namespace cone_drive_test;

// 실측 장면: 높이 0.227 m, 숙임 6.5°, 앞차축 0.29 m 앞 콘. 실제 카메라(fx 561, 1280×800)로는 화면 안이라 '보여야 함'
TEST(near_cone_is_expected_with_real_field_of_view) {
  CameraGeometry g;
  g.camera_x = -0.08;
  g.height = 0.227;
  g.pitch_rad = 6.5 * M_PI / 180.0;
  const std::vector<Vec2> cone{{0.29, -0.11}};
  CHECK(expected_visible(cone, g, 0.13).empty());   // 기본 화각(69°)으로는 사각으로 판정 → 놓침이 안 쌓였다
  set_field_of_view(g, 701.751174926 / 1.25, 701.420440674 / 1.25, 1280, 800);
  CHECK_NEAR(g.hfov_rad * 180.0 / M_PI, 97.5, 0.5);
  CHECK(expected_visible(cone, g, 0.13).size() == 1);
  CHECK(expected_visible({{0.15, 0.0}}, g, 0.13).empty());   // 정말 화면 아래로 벗어난 거리는 여전히 사각
}
