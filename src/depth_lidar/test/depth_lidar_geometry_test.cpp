#include "depth_lidar/depth_lidar_geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
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
  const auto projection =
    depth_lidar::projectDepthToScan(depth.data(), width, height, width, 4.0, 3.5, config);

  EXPECT_EQ(projection.roi.x, 2);
  EXPECT_EQ(projection.roi.y, 2);
  EXPECT_EQ(projection.roi.width, 4);
  EXPECT_EQ(projection.roi.height, 1);
  EXPECT_GT(projection.angle_max, projection.angle_min);
  EXPECT_GT(projection.valid_bins, 0U);
  EXPECT_EQ(projection.ranges.size(), 5U);
}

namespace
{

depth_lidar::ProjectionConfig fullRoiConfig()
{
  depth_lidar::ProjectionConfig config;
  config.roi_height_ratio = 1.0;
  config.roi_bottom_offset_ratio = 0.0;
  config.min_range_m = 0.1;
  config.max_range_m = 3.0;
  config.scan_bins = 3;
  return config;
}

depth_lidar::ScanProjection makeScan(const std::size_t bins, const float increment)
{
  depth_lidar::ScanProjection scan;
  scan.angle_min = -0.5F;
  scan.angle_max = scan.angle_min + static_cast<float>(bins - 1U) * increment;
  scan.angle_increment = increment;
  scan.ranges.assign(bins, std::numeric_limits<float>::infinity());
  return scan;
}

depth_lidar::ClusterConfig testClusterConfig()
{
  depth_lidar::ClusterConfig config;
  config.min_bins = 3;
  config.max_missing_bins = 1;
  config.base_neighbor_distance_m = 0.05;
  config.angular_neighbor_scale = 1.5;
  config.radius_margin_m = 0.0;
  config.min_radius_m = 0.001;
  config.max_radius_m = 1.0;
  return config;
}

} // namespace

TEST(DepthLidarGeometry, SelectsNearestOrFarthestAcrossRoiRows)
{
  auto config = fullRoiConfig();
  const std::vector<std::uint16_t> depth{
    0U, 2000U, 0U,
    0U, 1500U, 0U,
    0U, 400U, 0U,
    0U, 800U, 0U};
  auto scan = depth_lidar::projectDepthToScan(depth.data(), 3, 4, 3, 4.0, 1.0, config);
  ASSERT_EQ(scan.valid_bins, 1U);
  EXPECT_NEAR(scan.ranges[1], 0.4, 1e-6);
  config.range_selection = "farthest";
  scan = depth_lidar::projectDepthToScan(depth.data(), 3, 4, 3, 4.0, 1.0, config);
  ASSERT_EQ(scan.valid_bins, 1U);
  EXPECT_NEAR(scan.ranges[1], 2.0, 1e-6);
  EXPECT_TRUE(std::isinf(scan.ranges[0]));
  EXPECT_TRUE(std::isinf(scan.ranges[2]));
}

TEST(DepthLidarGeometry, FarthestHonorsCorrectedRangeLimitsAndMinimumSupport)
{
  auto config = fullRoiConfig();
  config.range_selection = "farthest";
  config.range_offset_m = 0.1;
  const std::vector<std::uint16_t> depth{
    0U, 0U, 0U,
    0U, 3500U, 0U,
    0U, 2950U, 0U,
    0U, 1000U, 0U,
    0U, 2000U, 0U};
  config.min_points_per_bin = 2;
  auto scan = depth_lidar::projectDepthToScan(depth.data(), 3, 5, 3, 4.0, 1.0, config);
  EXPECT_EQ(scan.valid_input_points, 2U);
  ASSERT_EQ(scan.valid_bins, 1U);
  EXPECT_NEAR(scan.ranges[1], 2.1, 1e-6);
  config.min_points_per_bin = 3;
  scan = depth_lidar::projectDepthToScan(depth.data(), 3, 5, 3, 4.0, 1.0, config);
  EXPECT_EQ(scan.valid_bins, 0U);
  EXPECT_TRUE(std::isinf(scan.ranges[1]));
}

TEST(DepthLidarGeometry, FarthestWithoutValidSamplesStaysEmpty)
{
  auto config = fullRoiConfig();
  config.range_selection = "farthest";
  const std::vector<std::uint16_t> depth{0U, 50U, 4000U};
  const auto scan = depth_lidar::projectDepthToScan(depth.data(), 3, 1, 3, 4.0, 1.0, config);
  EXPECT_EQ(scan.valid_bins, 0U);
  for (const auto range : scan.ranges) {
    EXPECT_TRUE(std::isinf(range));
  }
  config.range_selection = "unknown";
  EXPECT_THROW(depth_lidar::projectDepthToScan(depth.data(), 3, 1, 3, 4.0, 1.0, config),
    std::invalid_argument);
}

TEST(DepthLidarGeometry, ClustersAdjacentBinsAndRejectsSinglePointNoise)
{
  auto scan = makeScan(80U, 0.01F);
  scan.ranges[5] = 1.0F;
  for (std::size_t bin = 20U; bin <= 24U; ++bin) {
    scan.ranges[bin] = 1.2F;
  }
  scan.ranges[22] = std::numeric_limits<float>::infinity();

  const auto obstacles = depth_lidar::clusterScan(scan, testClusterConfig());

  ASSERT_EQ(obstacles.size(), 1U);
  EXPECT_EQ(obstacles.front().support_bins, 4U);
  EXPECT_EQ(obstacles.front().first_bin, 20U);
  EXPECT_EQ(obstacles.front().last_bin, 24U);
}

TEST(DepthLidarGeometry, SplitsClustersAcrossAConfiguredAngularGap)
{
  auto scan = makeScan(80U, 0.01F);
  for (std::size_t bin = 10U; bin <= 13U; ++bin) {
    scan.ranges[bin] = 1.0F;
  }
  for (std::size_t bin = 17U; bin <= 20U; ++bin) {
    scan.ranges[bin] = 1.0F;
  }

  const auto obstacles = depth_lidar::clusterScan(scan, testClusterConfig());

  ASSERT_EQ(obstacles.size(), 2U);
  EXPECT_EQ(obstacles[0].support_bins, 4U);
  EXPECT_EQ(obstacles[1].support_bins, 4U);
}

TEST(DepthLidarGeometry, CompensatesAngularSupportByRangeWhenEstimatingRadius)
{
  auto near_scan = makeScan(100U, 0.01F);
  auto far_scan = makeScan(100U, 0.01F);
  for (std::size_t bin = 30U; bin <= 50U; ++bin) {
    near_scan.ranges[bin] = 1.0F;
  }
  for (std::size_t bin = 35U; bin <= 45U; ++bin) {
    far_scan.ranges[bin] = 2.0F;
  }

  const auto near_obstacles = depth_lidar::clusterScan(near_scan, testClusterConfig());
  const auto far_obstacles = depth_lidar::clusterScan(far_scan, testClusterConfig());

  ASSERT_EQ(near_obstacles.size(), 1U);
  ASSERT_EQ(far_obstacles.size(), 1U);
  EXPECT_NEAR(near_obstacles.front().radius_m, far_obstacles.front().radius_m, 0.01);
}
