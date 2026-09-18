#pragma once
#include "auto_control/avoidance_planner.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_msgs/msg/string.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace auto_control
{
// No subscriptions or waits are added to the lane inference/control thread.
class AvoidanceWorker
{
public:
  explicit AvoidanceWorker(rclcpp::Node & node):node_(node)
  {
    avoidance::Options o;
    parameter("vehicle_half_width_m",o.half_width); parameter("vehicle_half_length_m",o.half_length);
    parameter("safety_margin_m",o.margin); parameter("unknown_extent_m",o.unknown_extent);
    parameter("sample_step_m",o.step); parameter("max_offset_m",o.max_offset);
    parameter("offset_step_m",o.offset_step); parameter("transition_m",o.transition);
    parameter("max_curvature_per_m",o.max_curvature); parameter("max_speed_mps",o.max_speed);
    parameter("lateral_acceleration_mps2",o.lateral_acceleration); parameter("deceleration_mps2",o.deceleration);
    parameter("clear_confirm_sec",o.clear_sec);
    parameter("max_age_sec",max_age_); parameter("max_sync_sec",max_sync_);
    parameter("max_fps",max_fps_);
    avoidance::validate(o);
    if (!std::isfinite(max_age_)||max_age_<.05||max_age_>.3||!std::isfinite(max_sync_)||
      max_sync_<.001||max_sync_>.1||!std::isfinite(max_fps_)||max_fps_<1||max_fps_>30) {
      throw std::invalid_argument("avoidance preview age/sync/FPS invalid");
    }
    planner_=std::make_unique<avoidance::Planner>(o);
    // Read on the worker thread: ros2 param set can enable/disable at runtime.
    node_.declare_parameter<bool>("obstacles.avoidance.enabled",false);
    status_=node_.create_publisher<std_msgs::msg::String>("/auto/avoidance_preview/status",rclcpp::QoS(1));
    thread_=std::thread([this]() {run();});
  }
  ~AvoidanceWorker()
  {
    {std::lock_guard<std::mutex> lock(mutex_); stop_=true;}
    wake_.notify_all();
    if (thread_.joinable()) {thread_.join();}
    bev_handoff::setAvoidancePreviewEnabled(false);
  }
private:
  void parameter(const std::string & name,double & value)
  {
    rcl_interfaces::msg::ParameterDescriptor d; d.read_only=true;
    d.description="Preview only; edit obstacle YAML and restart";
    value=node_.declare_parameter<double>("obstacles.avoidance."+name,value,d);
  }
  void report(const std::string & status)
  {
    if (status==last_status_ || !rclcpp::ok()) {return;}
    last_status_=status; std_msgs::msg::String message; message.data=status; status_->publish(message);
    RCLCPP_INFO_THROTTLE(node_.get_logger(),*node_.get_clock(),1000,
      "AVOIDANCE PREVIEW: %s (no actuator output)",status.c_str());
  }
  void unavailable(const std::string & why)
  {
    planner_->unavailable(); bev_handoff::publishAvoidancePreview(nullptr); report(why);
  }
  void run()
  {
    using Clock=std::chrono::steady_clock;
    bool previous_enabled=false;
    std::shared_ptr<const bev_handoff::PlanningLane> previous;
    while (rclcpp::ok()) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (wake_.wait_for(lock,std::chrono::duration<double>(1/max_fps_),[this]() {return stop_;})) {break;}
      }
      if (!rclcpp::ok()) {break;}
      try {
        const bool enabled=node_.get_parameter("obstacles.avoidance.enabled").as_bool();
        if (enabled!=previous_enabled) {
          planner_->reset(); previous.reset(); bev_handoff::setAvoidancePreviewEnabled(enabled);
          previous_enabled=enabled; report(enabled?"WAIT":"OFF");
        }
        if (!enabled) {continue;}
        const auto lane=bev_handoff::latestPlanningLane();
        const auto now=Clock::now();
        const double lane_age=lane?std::chrono::duration<double>(now-lane->received_at).count():1e9;
        const auto existing=bev_handoff::latestAvoidancePreview();
        if (!lane || lane_age<0 || lane_age>max_age_) {unavailable("WAIT: stale lane"); continue;}
        if (existing && std::chrono::duration<double>(now-existing->depth_captured_at).count()>max_age_) {
          unavailable("WAIT: stale depth");
        }
        if (lane==previous) {continue;}
        const auto obstacles=bev_handoff::matchingObstacles(lane->header,max_sync_,max_age_);
        if (!obstacles) {unavailable("WAIT: depth sync"); continue;}
        if (obstacles->width!=lane->width || obstacles->height!=lane->height ||
          std::abs(obstacles->x_max-lane->x_max)>1e-5 || std::abs(obstacles->y_max-lane->y_max)>1e-5 ||
          std::abs(obstacles->meter_per_pixel-lane->meter_per_pixel)>1e-5) {
          unavailable("WAIT: geometry mismatch"); continue;
        }
        previous=lane;
        auto result=std::make_shared<bev_handoff::AvoidancePreview>(planner_->plan(
          *lane,*obstacles,double(obstacles->header.stamp.sec)+obstacles->header.stamp.nanosec*1e-9));
        result->display_delta_sec=std::min(max_age_,1/max_fps_+max_sync_);
        const auto finished=Clock::now();
        if (std::chrono::duration<double>(finished-lane->received_at).count()>max_age_ ||
          std::chrono::duration<double>(finished-obstacles->captured_at).count()>max_age_) {
          unavailable("WAIT: planning expired"); continue;
        }
        report(result->status); bev_handoff::publishAvoidancePreview(std::move(result));
      } catch (const std::exception & e) {
        unavailable("WAIT: planner error");
        RCLCPP_ERROR_THROTTLE(node_.get_logger(),*node_.get_clock(),5000,"Avoidance preview: %s",e.what());
      }
    }
  }
  rclcpp::Node & node_;
  std::unique_ptr<avoidance::Planner> planner_;
  double max_age_{.25},max_sync_{.06},max_fps_{10};
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stop_{false};
  std::string last_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::thread thread_;
};
}
