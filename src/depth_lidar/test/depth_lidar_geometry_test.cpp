#include "depth_lidar/depth_lidar_geometry.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

TEST(DepthLidarGeometry, ComputesRoiAndProjectsAFlatDepthImage)
{
  depth_lidar::ProjectionConfig config;
  config.roi_width_ratio = 0.5;
  config.roi_height_ratio = 0.25;
  config.roi_bottom_offset_ratio = 0.25;
  config.min_range_m = 0.1;
  config.max_range_m = 5.0;
  config.scan_bins = 5;

  constexpr int width = 8;
  constexpr int height = 4;
  std::vector<std::uint16_t> depth(width * height, 1000U);
  const auto projection = depth_lidar::projectDepthToScan(
    depth.data(), width, height, width, 4.0, 3.5, config);

  EXPECT_EQ(projection.roi.x, 2);
  EXPECT_EQ(projection.roi.y, 2);
  EXPECT_EQ(projection.roi.width, 4);
  EXPECT_EQ(projection.roi.height, 1);
  EXPECT_GT(projection.angle_max, projection.angle_min);
  EXPECT_GT(projection.valid_bins, 0U);
  EXPECT_EQ(projection.ranges.size(), 5U);
}
