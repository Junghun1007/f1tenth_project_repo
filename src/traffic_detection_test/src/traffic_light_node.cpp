#include "bev_handoff/direct_camera_handoff.hpp"
#include "traffic_detection_test/yolox_detector.hpp"
#include "traffic_detection_test/msg/traffic_light_state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace traffic_detection_test
{
using Clock = std::chrono::steady_clock;
using Frame = bev_handoff::DirectCameraFrame;
using State = msg::TrafficLightState;

class TrafficLightNode : public rclcpp::Node
{
public:
  explicit TrafficLightNode(const rclcpp::NodeOptions & options)
  : Node("traffic_light_detector", options)
  {
    model_ = declare_parameter<std::string>("model_path", "");
    if (model_.empty()) {
      model_ = ament_index_cpp::get_package_share_directory("traffic_detection_test") +
        "/models/traffic_light_yolox_s_640x160_batch_1.int8.qdq.onnx";
    }
    cache_ = declare_parameter<std::string>("engine_cache_path", "");
    const int workspace = declare_parameter<int>("tensorrt_workspace_size_mb", 1024);
    const double fps = declare_parameter<double>("inference_fps", 20.0);
    max_age_ = declare_parameter<double>("max_frame_age_sec", 0.25);
    log_interval_ = declare_parameter<double>("status_log_interval_sec", 2.0);
    left_ = declare_parameter<double>("roi_left", 0.0);
    top_ = declare_parameter<double>("roi_top", 0.1625);
    width_ = declare_parameter<double>("roi_width", 1.0);
    height_ = declare_parameter<double>("roi_height", 0.4);
    score_ = declare_parameter<double>("score_threshold", 0.25);
    nms_ = declare_parameter<double>("nms_threshold", 0.65);
    min_s_ = declare_parameter<int>("color_min_saturation", 80);
    min_v_ = declare_parameter<int>("color_min_value", 60);
    for (double v : {fps, max_age_, log_interval_, left_, top_, width_, height_, score_, nms_}) {
      if (!std::isfinite(v)) {throw std::invalid_argument("traffic parameters must be finite");}
    }
    if (fps <= 0 || fps > 80 || workspace <= 0 || max_age_ <= 0 || log_interval_ <= 0 ||
      left_ < 0 || top_ < 0 || width_ <= 0 || height_ <= 0 ||
      left_ + width_ > 1 || top_ + height_ > 1 || score_ <= 0 || score_ > 1 ||
      nms_ < 0 || nms_ > 1 || min_s_ < 0 || min_s_ > 255 || min_v_ < 0 || min_v_ > 255)
    {throw std::invalid_argument("invalid traffic ROI/rate/threshold parameters");}
    workspace_ = static_cast<std::size_t>(workspace) * 1024U * 1024U;
    period_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / fps));
    publisher_ = create_publisher<State>(
      declare_parameter<std::string>("state_topic", "/traffic_light/state"), rclcpp::QoS(1));
    consumer_ = bev_handoff::registerDirectCameraConsumer(
      [this](std::shared_ptr<const Frame> frame) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {contention_drops_.fetch_add(1); return;}
        if (pending_) {replaced_.fetch_add(1);}
        pending_ = std::move(frame);
        condition_.notify_one();
      });
    try {
      worker_ = std::thread([this]() {
        try {run();}
        catch (const std::exception & e) {
          bev_handoff::publishTrafficSignalState(State::UNKNOWN, Clock::time_point{});
          // A publication/shutdown failure must not terminate the shared
          // camera/BEV/lane process through an uncaught worker exception.
          if (rclcpp::ok(get_node_base_interface()->get_context())) {
            RCLCPP_ERROR(get_logger(), "Traffic worker stopped: %s", e.what());
          }
        }
      });
    } catch (...) {
      bev_handoff::unregisterDirectCameraConsumer(consumer_);
      throw;
    }
    RCLCPP_INFO(get_logger(),
      "Traffic observation enabled: INT8 640x160, %.1fHz, shared pre-BEV color input, "
      "no preview/control commands. Engine initialization runs on its own worker.", fps);
  }

  ~TrafficLightNode() override
  {
    bev_handoff::unregisterDirectCameraConsumer(consumer_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    if (worker_.joinable()) {worker_.join();}
    bev_handoff::publishTrafficSignalState(State::UNKNOWN, Clock::time_point{});
  }

private:
  static double milliseconds(Clock::duration elapsed)
  {return std::chrono::duration<double, std::milli>(elapsed).count();}

  std::uint8_t color(const Frame & frame, const TrafficLightDetection & detection)
  {
    const auto & b = detection.box;
    if (!std::isfinite(b.x) || !std::isfinite(b.y) ||
      !std::isfinite(b.width) || !std::isfinite(b.height)) {return State::UNKNOWN;}
    const int left = std::clamp(static_cast<int>(std::floor(b.x)), 0, frame.width) & ~1;
    const int top = std::clamp(static_cast<int>(std::floor(b.y)), 0, frame.height) & ~1;
    const int right = std::min(frame.width,
      (std::clamp(static_cast<int>(std::ceil(b.x + b.width)), 0, frame.width) + 1) & ~1);
    const int bottom = std::min(frame.height,
      (std::clamp(static_cast<int>(std::ceil(b.y + b.height)), 0, frame.height) + 1) & ~1);
    if (right <= left || bottom <= top) {return State::UNKNOWN;}
    // Only the winning box is converted on CPU; no full-frame BGR or preview.
    cv::Mat y(bottom - top, right - left, CV_8UC1,
      const_cast<std::uint8_t *>(frame.nv12) + top * frame.stride + left, frame.stride);
    cv::Mat uv((bottom - top) / 2, (right - left) / 2, CV_8UC2,
      const_cast<std::uint8_t *>(frame.nv12) + frame.height * frame.stride +
      (top / 2) * frame.stride + left, frame.stride);
    cv::cvtColorTwoPlane(y, uv, bgr_, cv::COLOR_YUV2BGR_NV12);
    cv::cvtColor(bgr_, hsv_, cv::COLOR_BGR2HSV);
    std::uint64_t red = 0, green = 0;
    for (int row = 0; row < hsv_.rows; ++row) {
      const auto * pixels = hsv_.ptr<cv::Vec3b>(row);
      for (int col = 0; col < hsv_.cols; ++col) {
        const auto p = pixels[col];
        if (p[1] < min_s_ || p[2] < min_v_) {continue;}
        const std::uint64_t energy = static_cast<unsigned>(p[1]) * p[2];
        if (p[0] <= 20 || p[0] >= 165) {red += energy;}
        else if (p[0] >= 35 && p[0] <= 95) {green += energy;}
      }
    }
    return red > green ? State::RED : green > red ? State::GREEN : State::UNKNOWN;
  }

  void run()
  {
    // Building/deserializing TensorRT never blocks component construction or
    // the BEV executor. Cold engine building can still compete for GPU time.
    std::unique_ptr<YoloxDetector> detector;
    State preparing;
    preparing.capture_age_ms = -1.0F;
    bev_handoff::publishTrafficSignalState(State::UNKNOWN, Clock::time_point{});
    publisher_->publish(preparing);
    try {
      detector = std::make_unique<YoloxDetector>(
        model_, "TENSORRT", cache_, "int8", 640, 160, score_, nms_, workspace_);
      RCLCPP_INFO(get_logger(), "Traffic TensorRT engine ready; awaiting shared color frames.");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Traffic initialization failed: %s", e.what());
      // Stay alive publishing UNKNOWN; do not disturb the lane pipeline.
    }
    auto deadline = Clock::now();
    auto last_log = deadline;
    auto last_publish = deadline - std::chrono::seconds(1);
    std::uint64_t count = 0, stale = 0;
    double sum_ms = 0, max_ms = 0, sum_infer = 0, max_infer = 0, max_age_ms = 0;
    for (;;) {
      std::shared_ptr<const Frame> frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        // Rate limit inference STARTS. There is never a FIFO or catch-up burst.
        condition_.wait_until(lock, deadline, [this]() {return stopping_;});
        if (stopping_) {break;}
        condition_.wait_for(lock, std::chrono::milliseconds(100),
          [this]() {return stopping_ || pending_;});
        if (stopping_) {break;}
        frame = std::move(pending_);
      }
      const auto started = Clock::now();
      deadline = started + period_;
      State message;
      message.state = State::UNKNOWN;
      message.capture_age_ms = -1.0F;
      if (frame) {
        message.header = frame->header;
        message.source_generation = frame->generation;
        const auto age = started - frame->captured_at;
        if (detector && age >= Clock::duration::zero() &&
          age <= std::chrono::duration<double>(max_age_))
        {
          try {
            const cv::Rect roi(
              static_cast<int>(left_ * frame->width), static_cast<int>(top_ * frame->height),
              static_cast<int>(width_ * frame->width), static_cast<int>(height_ * frame->height));
            const auto result = detector->detect_nv12(frame->nv12, frame->size, frame->stride,
              frame->width, frame->height, roi);
            message.inference_ms = result.timing.forward_nanoseconds / 1.0e6;
            if (!result.detections.empty()) {
              const auto best = std::max_element(result.detections.begin(), result.detections.end(),
                [](const auto & a, const auto & b) {return a.score < b.score;});
              message.detection_score = best->score;
              message.state = color(*frame, *best);
            }
          } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
              "Traffic processing failed: %s", e.what());
          }
        } else {++stale;}
        message.capture_age_ms = milliseconds(Clock::now() - frame->captured_at);
        if (message.capture_age_ms < 0 || message.capture_age_ms > max_age_ * 1000) {
          message.state = State::UNKNOWN;
        }
        message.processing_ms = milliseconds(Clock::now() - started);
        ++count;
        sum_ms += message.processing_ms;
        max_ms = std::max(max_ms, static_cast<double>(message.processing_ms));
        sum_infer += message.inference_ms;
        max_infer = std::max(max_infer, static_cast<double>(message.inference_ms));
        max_age_ms = std::max(max_age_ms, static_cast<double>(message.capture_age_ms));
      }
      if (frame || started - last_publish >= std::chrono::milliseconds(100)) {
        bev_handoff::publishTrafficSignalState(message.state, frame ?
          frame->captured_at + std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(max_age_)) : Clock::time_point{});
        publisher_->publish(message);
        last_publish = started;
      }
      const double elapsed = std::chrono::duration<double>(Clock::now() - last_log).count();
      if (elapsed >= log_interval_) {
        RCLCPP_INFO(get_logger(),
          "Traffic | state=%s | input_processed=%.1fHz | inference=%.2f/%.2fms avg/max | "
          "processing=%.2f/%.2fms avg/max | capture_age_max=%.1fms | "
          "replaced=%llu contention=%llu stale_or_unavailable=%llu",
          message.state == State::RED ? "RED" : message.state == State::GREEN ? "GREEN" : "UNKNOWN",
          count / elapsed, count ? sum_infer / count : 0, max_infer,
          count ? sum_ms / count : 0, max_ms, max_age_ms,
          static_cast<unsigned long long>(replaced_.exchange(0)),
          static_cast<unsigned long long>(contention_drops_.exchange(0)),
          static_cast<unsigned long long>(stale));
        if (!count) {
          RCLCPP_WARN(get_logger(), "No shared color frames: load camera_driver in this container with CAM_A.");
        }
        count = stale = 0;
        sum_ms = max_ms = sum_infer = max_infer = max_age_ms = 0;
        last_log = Clock::now();
      }
    }
  }

  std::string model_, cache_;
  std::size_t workspace_;
  double max_age_, log_interval_, left_, top_, width_, height_, score_, nms_;
  int min_s_, min_v_;
  Clock::duration period_;
  std::uint64_t consumer_{0};
  rclcpp::Publisher<State>::SharedPtr publisher_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool stopping_{false};
  std::shared_ptr<const Frame> pending_;
  std::thread worker_;
  std::atomic<std::uint64_t> replaced_{0}, contention_drops_{0};
  cv::Mat bgr_, hsv_;
};
}  // namespace traffic_detection_test
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_detection_test::TrafficLightNode)
