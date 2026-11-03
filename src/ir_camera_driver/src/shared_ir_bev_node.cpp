// Projects IR frames from camera_driver's existing OAK session. Opens no device.
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/imgproc.hpp>
#include "bev_handoff/direct_obstacle_handoff.hpp"
#include "ir_camera_driver/ir_bev_projector.hpp"

namespace ir_camera_driver
{
class SharedIrBevNode : public rclcpp::Node
{
public:
  explicit SharedIrBevNode(const rclcpp::NodeOptions & options)
  : Node("shared_ir_bev", options)
  {
    standby_=declare_parameter<double>("standby_fps",0.0);
    active_=declare_parameter<double>("active_fps",30.0);
    if (!std::isfinite(standby_) || !std::isfinite(active_) ||
      standby_<0 || active_<=0 || standby_>active_ || active_>30) {
      throw std::invalid_argument("IR rates require 0 <= standby <= active <= 30");
    }
    publisher_=create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_bev_ir",rclcpp::SensorDataQoS().keep_last(1));
    bgr_publisher_=create_publisher<sensor_msgs::msg::Image>(
      declare_parameter<std::string>("model_input_topic","/ir_bev_line_detection/image_bgr"),
      rclcpp::SensorDataQoS().keep_last(1));
    mode_sub_=create_subscription<std_msgs::msg::String>(
      "/tunnel/compute_source",rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        if (message->data!="IR" && message->data!="RGB" && message->data!="BOTH") {return;}
        inside_=message->data!="RGB";
        next_=Clock::time_point{};
      });
    timer_=create_wall_timer(std::chrono::milliseconds(1),[this]() {process();});
  }
private:
  using Clock=std::chrono::steady_clock;
  void process()
  {
    // Keep the camera session/model resident, but do not project/infer images
    // captured with the outside obstacle-detection exposure and dot settings.
    // Prepare the exposure-independent calibration/LUT once while outside.
    // Only the expensive image projection/model input waits for entry.
    if (!inside_ && standby_==0.0 && projector_) {return;}
    const auto now=Clock::now();
    if (now<next_) {return;}
    const auto frame=bev_handoff::latestDirectIr();
    const auto reference=bev_handoff::obstacleReference();
    if (!frame || !reference || frame->captured_at<=last_ ||
      now-frame->captured_at>std::chrono::milliseconds(250) ||
      (inside_ && !frame->tunnel_profile)) {return;}
    try {
      if (!projector_) {
        if (reference->width!=120 || reference->height!=300 ||
          std::abs(reference->x_min)>1e-8 || std::abs(reference->x_max-3.0)>1e-8 ||
          std::abs(reference->y_min+.6)>1e-8 || std::abs(reference->y_max-.6)>1e-8 ||
          std::abs(reference->meter_per_pixel-.01)>1e-8) {
          throw std::runtime_error("IR model requires shared 120x300 / 1cm / front-axle geometry");
        }
        projector_=std::make_unique<IrBevProjector>(IrBevConfig{});
        const auto & k=frame->intrinsics;
        projector_->configure(k[0],k[1],k[2],k[3],frame->mono.cols,frame->mono.rows,
          frame->rgb_from_view,frame->view_center_rgb_m);
        projector_->setMountPose(reference->rotation_vehicle_from_camera,reference->position_vehicle_m);
      }
      if (!inside_ && standby_==0.0) {return;}
      const auto image=projector_->project(frame->mono,frame->current_rgb_to_reference);
      auto message=std::make_unique<sensor_msgs::msg::Image>();
      message->header=frame->header; message->encoding="mono8";
      message->width=image.cols; message->height=image.rows; message->step=image.cols;
      message->data.assign(image.datastart,image.dataend);
      publisher_->publish(std::move(message));
      // Match the existing IR model's three identical input channels without
      // starting the standalone test package's Python adapter process.
      cv::Mat bgr;
      cv::cvtColor(image,bgr,cv::COLOR_GRAY2BGR);
      auto model_input=std::make_unique<sensor_msgs::msg::Image>();
      model_input->header=frame->header; model_input->encoding="bgr8";
      model_input->width=bgr.cols; model_input->height=bgr.rows; model_input->step=bgr.cols*3;
      model_input->data.assign(bgr.datastart,bgr.dataend);
      bgr_publisher_->publish(std::move(model_input));
      last_=frame->captured_at;
      const auto period=std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0/(inside_ ? active_ : standby_)));
      next_=next_==Clock::time_point{} ? now+period : next_+period;
      if (next_<now) {next_=now+period;}
    } catch (const std::exception & e) {
      projector_.reset();
      next_=now+std::chrono::seconds(1);
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),1000,"Shared IR BEV: %s",e.what());
    }
  }
  double standby_,active_;
  bool inside_{false};
  Clock::time_point last_{},next_{};
  std::unique_ptr<IrBevProjector> projector_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bgr_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(ir_camera_driver::SharedIrBevNode)
