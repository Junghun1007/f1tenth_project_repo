#include "check.hpp"
#include "cone_drive_test/core/box_depth.hpp"
#include "cone_drive_test/copied/bev_geometry.hpp"
#include <cmath>

using namespace cone_drive_test;

static const DepthIntrinsics K{350.0, 350.0, 320.0, 200.0, 640, 400};
static const CameraMount M{-0.09, 0.0, 0.20, 0.0, 13.0 * M_PI / 180.0, 0.0};

// 합성 깊이: 지면 평면 + (x0..x0+0.13, |y-yc|<=0.065, 0<z<0.18) 콘 앞면 상자
static std::vector<std::uint16_t> render(double x0, double yc) {
  std::vector<std::uint16_t> d(std::size_t(K.width) * K.height, 0);
  const auto R = mountRotationVehicleFromCamera(M.roll_rad, M.pitch_down_rad, M.yaw_rad);
  for (int v = 0; v < K.height; ++v) for (int u = 0; u < K.width; ++u) {
    const cv::Vec3d ro((u - K.cx) / K.fx, (v - K.cy) / K.fy, 1.0);   // 광학 z=1로 정규화한 광선
    const cv::Vec3d r = R * ro;
    double best = 1e9;
    if (r[2] < -1e-9) best = M.height / -r[2];                        // 지면 z=0
    if (r[0] > 1e-9) {                                                // 콘 앞면 x = x0
      const double t = (x0 - M.x) / r[0];
      const double y = M.y + t * r[1], z = M.height + t * r[2];
      if (std::abs(y - yc) <= 0.065 && z >= 0 && z <= 0.18) best = std::min(best, t);
    }
    if (best < 9.0) d[std::size_t(v) * K.width + u] = std::uint16_t(std::lround(best * 1000.0));   // t = 광학 깊이
  }
  return d;
}

static cv::Rect2f box_of(double x0, double yc) {   // 콘 외곽을 영상에 투영한 박스
  const auto R = mountRotationVehicleFromCamera(M.roll_rad, M.pitch_down_rad, M.yaw_rad).t();
  float u0 = 1e9, v0 = 1e9, u1 = -1e9, v1 = -1e9;
  for (double dx : {0.0, 0.13}) for (double dy : {-0.065, 0.065}) for (double z : {0.0, 0.18}) {
    const cv::Vec3d o = R * cv::Vec3d(x0 + dx - M.x, yc + dy - M.y, z - M.height);
    const float u = float(K.fx * o[0] / o[2] + K.cx), v = float(K.fy * o[1] / o[2] + K.cy);
    u0 = std::min(u0, u); u1 = std::max(u1, u); v0 = std::min(v0, v); v1 = std::max(v1, v);
  }
  return {u0, v0, u1 - u0, v1 - v0};
}

TEST(center_row_ray_hits_ground_ahead) {   // 좌표 규약 확인: 영상 중앙 행 = 13° 아래
  const auto d = render(10.0, 5.0);       // 콘은 시야 밖
  const double depth = d[std::size_t(200) * K.width + 320] / 1000.0;
  // 광학 깊이 t에서 지면까지 수평거리 = t·cos(13°)
  CHECK_NEAR(depth * std::cos(13.0 * M_PI / 180.0) + M.x, M.height / std::tan(13.0 * M_PI / 180.0) + M.x, 0.005);
}

TEST(box_depth_flat_cone) {
  const auto d = render(1.2, 0.1);
  const auto obs = localize_box(d.data(), K.width, K, M, box_of(1.2, 0.1), 0, 0.9f, {0.13, 0.13}, {});
  CHECK(obs.has_value());
  CHECK_NEAR(obs->center.x, 1.2 + 0.065, 0.02);
  CHECK_NEAR(obs->center.y, 0.1, 0.02);
  CHECK(obs->class_id == 0);
}

TEST(box_depth_rejects_ground_only) {
  const auto d = render(10.0, 5.0);
  CHECK(!localize_box(d.data(), K.width, K, M, box_of(1.2, 0.1), 0, 0.9f, {0.13, 0.13}, {}).has_value());
}

TEST(box_depth_clips_box_outside_image) {
  const auto d = render(1.2, 0.1);
  CHECK(!localize_box(d.data(), K.width, K, M, {-50.f, -50.f, 10.f, 10.f}, 0, 0.9f, {0.13, 0.13}, {}).has_value());
}
