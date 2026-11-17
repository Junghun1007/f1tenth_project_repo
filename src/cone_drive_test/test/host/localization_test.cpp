// Ported from src/multiclass_yolo/test/object_localization_test.cpp (origin/1113ML)
#include "check.hpp"
#include "cone_drive_test/copied/object_localization.hpp"

using namespace cone_drive_test;
static bool near(double a, double b, double tolerance = 1e-6) { return std::abs(a - b) <= tolerance; }
static void add(std::vector<ObjectPoint> & points, double x, double width, int count) {
  for (int i = 0; i < count; ++i) points.push_back({x + .002 * (i % 5), width * (static_cast<double>(i) / (count - 1) - .5)});
}

TEST(localization_matches_original) {
  const Footprint sedan{.50, .30}, cone{.13, .13};
  std::vector<ObjectPoint> points;
  add(points, 1.00, .28, 30);
  add(points, 2.50, .60, 100);
  points.push_back({.40, 0}); points.push_back({.42, .01}); points.push_back({.45, -.01});
  auto object = localize(points, 0, 0, 0.0, sedan);
  CHECK(object.valid);
  CHECK(near(object.range, 1.0, .02));
  CHECK(object.cluster_points >= 30 && object.cluster_points < 40);
  CHECK(object.cluster_max < 2.0 && object.cluster_min > .9);
  CHECK(near(object.near_range, 1.0, .02));
  CHECK(near(object.center_x, object.range + .25) && near(object.center_y, 0));
  CHECK(near(object.corners[0].x, object.range) && near(object.corners[0].y, .15));
  CHECK(near(object.corners[2].x, object.range + .50) && near(object.corners[2].y, -.15));
  const double bearing = std::atan2(1.0, 1.0);
  object = localize(points, 0, 0, bearing, cone);
  CHECK(near(object.front_x, object.front_y) && near(object.center_x, object.center_y));
  CHECK(near(std::hypot(object.center_x, object.center_y), object.range + .065));
  std::vector<ObjectPoint> left;
  for (int i = 0; i < 12; ++i) left.push_back({1.0, 1.0 + .001 * i});
  object = localize(left, 0, 0, std::nullopt, cone);
  CHECK(object.valid && near(object.bearing, std::atan2(1.0, 1.0), .01));
  object = localize(left, -.09, 0, std::nullopt, cone);
  CHECK(near(object.range, std::hypot(1.09, 1.005), .01));
  CHECK(!localize({{1, 0}, {1, .01}, {1, .02}}, 0, 0, 0.0, cone).valid);
  std::vector<ObjectPoint> scattered;
  for (int i = 0; i < 20; ++i) scattered.push_back({1.0 + .5 * i, 0});
  CHECK(!localize(scattered, 0, 0, 0.0, cone).valid);
  object = localize(left, 0, 0, std::nullopt, Footprint{});
  CHECK(object.valid && near(object.center_x, object.front_x));
}
