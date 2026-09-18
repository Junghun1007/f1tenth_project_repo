#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

namespace bev_handoff
{
// Process-local, immutable depth buffer. owner keeps DepthAI storage alive.
// Millimetres, optical-axis Z, calibrated rectified camera coordinates.
struct DirectDepthFrame
{
  const std::uint8_t * data{nullptr};
  std::size_t size{0}, stride{0};
  int width{0}, height{0};
  std::shared_ptr<const void> owner;
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point captured_at;
  std::array<double,4> intrinsics{}; // fx,fy,cx,cy
  std::array<double,9> rgb_from_depth_rotation{};
  std::array<double,3> rgb_from_depth_translation{};
};
void setObstacleDeviceId(const std::string & id);
std::string obstacleDeviceId();
void publishDirectDepth(std::shared_ptr<const DirectDepthFrame> frame);
std::shared_ptr<const DirectDepthFrame> latestDirectDepth();
struct ObstacleCluster
{
  std::vector<cv::Point2f> surface_xy;
  cv::Point2f center;
  double nearest_range{0};
};
struct ObstacleFrame
{
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point captured_at;
  double x_max{3.0}, y_max{0.6}, meter_per_pixel{0.01};
  int width{120}, height{300};
  double fps{0}, host_ms{0};
  std::vector<ObstacleCluster> clusters;
};
// Small bounded history for asynchronous RGB inference/depth completion.
void publishObstacles(std::shared_ptr<const ObstacleFrame> frame);
void clearObstacles();
std::shared_ptr<const ObstacleFrame> matchingObstacles(
  const std_msgs::msg::Header & image, double max_delta_sec, double max_age_sec);
}  // namespace bev_handoff
