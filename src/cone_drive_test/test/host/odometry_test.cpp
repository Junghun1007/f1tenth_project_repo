#include "check.hpp"
#include "cone_drive_test/core/odometry.hpp"
#include "cone_drive_test/core/visibility.hpp"

using namespace cone_drive_test;

TEST(odometry_straight_line) {
  Odometry od(0.33);
  od.reset({0, 0, 0});
  for (int k = 1; k <= 100; ++k) od.update(k * 0.01, 1.0, 0.0);
  CHECK_NEAR(od.pose().x, 1.0, 1e-9);
  CHECK_NEAR(od.pose().y, 0.0, 1e-12);
}

TEST(odometry_pose_at_interpolates_and_rejects_out_of_range) {   // Review Focus 1
  Odometry od(0.33);
  od.reset({0, 0, 0});
  for (int k = 1; k <= 300; ++k) od.update(k * 0.01, 1.0, 0.2);    // 0~3 s, 이력은 최근 2 s
  const auto mid = od.pose_at(2.505);
  CHECK(mid.has_value());
  const auto a = od.pose_at(2.50), b = od.pose_at(2.51);
  CHECK(a && b);
  CHECK_NEAR(mid->x, 0.5 * (a->x + b->x), 1e-3);
  CHECK_NEAR(mid->yaw, 0.5 * (a->yaw + b->yaw), 1e-6);
  CHECK(!od.pose_at(3.05).has_value());     // 미래
  CHECK(!od.pose_at(0.5).has_value());      // 2 s보다 오래됨
  CHECK(!od.pose_at(-0.1).has_value());
  const auto now = od.pose_at(3.0);
  CHECK(now.has_value());
  CHECK_NEAR(now->x, od.pose().x, 1e-9);
}

TEST(hidden_cone_not_expected) {
  CameraGeometry g;
  const auto vis = expected_visible({{0.45, 0.0}, {0.85, 0.0}, {2.0, 0.3}}, g, 0.13);
  CHECK(vis.size() == 1 && vis[0] == 0);
}
