#pragma once
#include "bev_handoff/direct_obstacle_handoff.hpp"

namespace bev_handoff
{
// Preview-only planning channel. Never consumed by the actuator controller.
struct PlanningLane
{
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point received_at;
  std::vector<cv::Point2d> center;
  std::vector<std::vector<cv::Point2d>> boundaries;
  bool valid{false};
  double x_max{3}, y_max{.6}, meter_per_pixel{.01};
  int width{120}, height{300};
};
struct SafetyBox {double x0, x1, y0, y1;};
struct AvoidanceCandidate
{
  std::vector<cv::Point2d> path;
  bool valid{false};
  double offset{0}, max_curvature{0};
  std::string reason;
};
struct AvoidancePreview
{
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point lane_received_at, depth_captured_at;
  double x_max{3}, y_max{.6}, meter_per_pixel{.01};
  int width{120}, height{300};
  std::vector<cv::Point2d> original, selected;
  std::vector<SafetyBox> boxes;
  std::vector<AvoidanceCandidate> candidates;
  std::string status{"WAIT"};
  double recommended_speed{0}, max_curvature{0};
  double display_delta_sec{.16};
};
bool avoidancePreviewEnabled();
void setAvoidancePreviewEnabled(bool enabled);
void publishPlanningLane(std::shared_ptr<const PlanningLane> lane);
std::shared_ptr<const PlanningLane> latestPlanningLane();
void publishAvoidancePreview(std::shared_ptr<const AvoidancePreview> plan);
std::shared_ptr<const AvoidancePreview> latestAvoidancePreview();
}  // namespace bev_handoff
