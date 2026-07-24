#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "bev_processor/bev_geometry.hpp"

namespace bev_processor
{

namespace
{

using SteadyClock = std::chrono::steady_clock;

struct BevFrame
{
  cv::Mat image;
  std_msgs::msg::Header header;
  SteadyClock::time_point input_received_at;
  std::uint64_t generation{0U};
};

bool graphicalDisplayAvailable()
{
#ifdef __APPLE__
  return true;
#else
  const char * display = std::getenv("DISPLAY");
  const char * wayland_display = std::getenv("WAYLAND_DISPLAY");
  return
    (display != nullptr && display[0] != '\0') ||
    (wayland_display != nullptr && wayland_display[0] != '\0');
#endif
}

cv::Mat viewBgr8Image(const sensor_msgs::msg::Image & message)
{
  return cv::Mat(
    static_cast<int>(message.height),
    static_cast<int>(message.width),
    CV_8UC3,
    const_cast<std::uint8_t *>(message.data.data()),
    static_cast<std::size_t>(message.step));
}

std::unique_ptr<sensor_msgs::msg::Image> makeBgr8Message(
  const BevFrame & frame,
  const std::string & frame_id)
{
  if (frame.image.type() != CV_8UC3) {
    throw std::invalid_argument("BEV output must be a BGR8 image");
  }

  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header = frame.header;
  message->header.frame_id = frame_id;
  message->height = static_cast<std::uint32_t>(frame.image.rows);
  message->width = static_cast<std::uint32_t>(frame.image.cols);
  message->encoding = sensor_msgs::image_encodings::BGR8;
  message->is_bigendian = false;
  message->step = static_cast<std::uint32_t>(frame.image.cols * 3);
  message->data.resize(
    static_cast<std::size_t>(message->step) *
    static_cast<std::size_t>(message->height));

  for (int row = 0; row < frame.image.rows; ++row) {
    std::memcpy(
      message->data.data() +
      static_cast<std::size_t>(row) * message->step,
      frame.image.ptr(row),
      message->step);
  }
  return message;
}

}  // namespace

class BevProcessorNode final : public rclcpp::Node
{
public:
  BevProcessorNode()
  : Node("bev_processor")
  {
    declareParameters();
    readParameters();
    validateParameters();

    const auto lut = std::make_shared<const RemapLut>(
      generateRemap(camera_model_, bev_config_));
    std::atomic_store_explicit(
      &remap_lut_, lut, std::memory_order_release);

    const int valid_pixels = cv::countNonZero(lut->valid_mask);
    const int output_pixels =
      bev_config_.output_width * bev_config_.output_height;
    valid_lut_percent_ =
      100.0 * static_cast<double>(valid_pixels) /
      static_cast<double>(output_pixels);

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    input_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      image_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        onImage(std::move(message));
      });

    if (publish_enabled_) {
      output_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic_, image_qos);
    }

    processing_thread_ = std::thread(&BevProcessorNode::processingLoop, this);
    if (publish_enabled_) {
      publishing_thread_ = std::thread(&BevProcessorNode::publishingLoop, this);
    }
    if (preview_enabled_) {
      if (graphicalDisplayAvailable()) {
        preview_thread_ = std::thread(&BevProcessorNode::previewLoop, this);
      } else {
        preview_enabled_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Preview disabled because DISPLAY/WAYLAND_DISPLAY is unavailable.");
      }
    }

    status_started_at_ = SteadyClock::now();
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(status_log_interval_sec_),
      std::bind(&BevProcessorNode::logStatus, this));

    RCLCPP_INFO(
      get_logger(),
      "BEV processor started: input=%s (%dx%d BGR8, expected=%.1fHz), "
      "output=%s (%dx%d), "
      "range X=[%.2f, %.2f]m Y=[%.2f, %.2f]m, %.3fm/px, "
      "valid_lut=%.2f%%, processing=event-driven/latest-only, "
      "ROS=%s (max=%.1fHz, 0=unlimited), preview=%s (max=%.1fHz)",
      input_topic_.c_str(),
      camera_model_.image_width,
      camera_model_.image_height,
      expected_input_fps_,
      output_topic_.c_str(),
      bev_config_.output_width,
      bev_config_.output_height,
      bev_config_.x_min_m,
      bev_config_.x_max_m,
      bev_config_.y_min_m,
      bev_config_.y_max_m,
      bev_config_.meter_per_pixel,
      valid_lut_percent_,
      publish_enabled_ ? "on" : "off",
      publish_max_fps_,
      preview_enabled_ ? "on" : "off",
      preview_max_fps_);
  }

  ~BevProcessorNode() override
  {
    stop_.store(true, std::memory_order_release);
    input_cv_.notify_all();
    output_cv_.notify_all();

    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    if (publishing_thread_.joinable()) {
      publishing_thread_.join();
    }
    if (preview_thread_.joinable()) {
      preview_thread_.join();
    }
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("input_topic", "/camera/image_rect");
    declare_parameter<std::string>("output_topic", "/camera/image_bev");
    declare_parameter<std::string>("output_frame_id", "front_axle_bev");
    declare_parameter<double>("expected_input_fps", 120.0);

    declare_parameter<bool>("publish_enabled", true);
    declare_parameter<double>("publish_max_fps", 0.0);
    declare_parameter<bool>("preview_enabled", true);
    declare_parameter<double>("preview_max_fps", 30.0);
    declare_parameter<std::string>("preview_window_name", "BEV image");
    declare_parameter<int>("preview_max_width", 1280);
    declare_parameter<int>("preview_max_height", 900);

    declare_parameter<int>("input_width", 960);
    declare_parameter<int>("input_height", 540);
    declare_parameter<double>("fx", 421.050720215);
    declare_parameter<double>("fy", 378.767059326);
    declare_parameter<double>("cx", 482.274475098);
    declare_parameter<double>("cy", 265.019256592);

    declare_parameter<double>("camera_x_m", 0.0);
    declare_parameter<double>("camera_y_m", 0.0);
    declare_parameter<double>("camera_z_m", 0.20);
    declare_parameter<double>("camera_roll_deg", 0.0);
    declare_parameter<double>("camera_downward_pitch_deg", 14.0);
    declare_parameter<double>("camera_yaw_deg", 0.0);

    declare_parameter<double>("x_min_m", 0.18);
    declare_parameter<double>("x_max_m", 5.0);
    declare_parameter<double>("y_min_m", -0.35);
    declare_parameter<double>("y_max_m", 0.35);
    declare_parameter<double>("meter_per_pixel", 0.01);
    declare_parameter<int>("output_width", 70);
    declare_parameter<int>("output_height", 482);

    declare_parameter<double>("status_log_interval_sec", 5.0);
    declare_parameter<double>("startup_timeout_sec", 5.0);
  }

  void readParameters()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    expected_input_fps_ = get_parameter("expected_input_fps").as_double();

    publish_enabled_ = get_parameter("publish_enabled").as_bool();
    publish_max_fps_ = get_parameter("publish_max_fps").as_double();
    preview_enabled_ = get_parameter("preview_enabled").as_bool();
    preview_max_fps_ = get_parameter("preview_max_fps").as_double();
    preview_window_name_ = get_parameter("preview_window_name").as_string();
    preview_max_width_ =
      static_cast<int>(get_parameter("preview_max_width").as_int());
    preview_max_height_ =
      static_cast<int>(get_parameter("preview_max_height").as_int());

    camera_model_.fx = get_parameter("fx").as_double();
    camera_model_.fy = get_parameter("fy").as_double();
    camera_model_.cx = get_parameter("cx").as_double();
    camera_model_.cy = get_parameter("cy").as_double();
    camera_model_.image_width =
      static_cast<int>(get_parameter("input_width").as_int());
    camera_model_.image_height =
      static_cast<int>(get_parameter("input_height").as_int());
    camera_model_.position_vehicle_m = cv::Vec3d(
      get_parameter("camera_x_m").as_double(),
      get_parameter("camera_y_m").as_double(),
      get_parameter("camera_z_m").as_double());
    camera_model_.rotation_vehicle_from_camera =
      mountRotationVehicleFromCamera(
      degToRad(get_parameter("camera_roll_deg").as_double()),
      degToRad(get_parameter("camera_downward_pitch_deg").as_double()),
      degToRad(get_parameter("camera_yaw_deg").as_double()));

    bev_config_.x_min_m = get_parameter("x_min_m").as_double();
    bev_config_.x_max_m = get_parameter("x_max_m").as_double();
    bev_config_.y_min_m = get_parameter("y_min_m").as_double();
    bev_config_.y_max_m = get_parameter("y_max_m").as_double();
    bev_config_.meter_per_pixel =
      get_parameter("meter_per_pixel").as_double();
    bev_config_.output_width =
      static_cast<int>(get_parameter("output_width").as_int());
    bev_config_.output_height =
      static_cast<int>(get_parameter("output_height").as_int());

    status_log_interval_sec_ =
      get_parameter("status_log_interval_sec").as_double();
    startup_timeout_sec_ = get_parameter("startup_timeout_sec").as_double();
  }

  void validateParameters() const
  {
    if (input_topic_.empty()) {
      throw std::invalid_argument("input_topic must not be empty");
    }
    if (publish_enabled_ && output_topic_.empty()) {
      throw std::invalid_argument(
              "output_topic must not be empty when publishing is enabled");
    }
    if (
      camera_model_.image_width <= 1 ||
      camera_model_.image_height <= 1 ||
      camera_model_.fx <= 0.0 ||
      camera_model_.fy <= 0.0)
    {
      throw std::invalid_argument(
              "input dimensions and focal lengths must be positive");
    }
    if (
      camera_model_.position_vehicle_m[2] <= 0.0 ||
      bev_config_.x_min_m < 0.0 ||
      bev_config_.x_max_m <= bev_config_.x_min_m ||
      bev_config_.y_max_m <= bev_config_.y_min_m ||
      bev_config_.meter_per_pixel <= 0.0 ||
      bev_config_.output_width <= 0 ||
      bev_config_.output_height <= 0)
    {
      throw std::invalid_argument("invalid camera pose or BEV bounds");
    }

    const int expected_width = static_cast<int>(std::llround(
        (bev_config_.y_max_m - bev_config_.y_min_m) /
        bev_config_.meter_per_pixel));
    const int expected_height = static_cast<int>(std::llround(
        (bev_config_.x_max_m - bev_config_.x_min_m) /
        bev_config_.meter_per_pixel));
    if (
      bev_config_.output_width != expected_width ||
      bev_config_.output_height != expected_height)
    {
      throw std::invalid_argument(
              "output_width/output_height do not match BEV bounds and "
              "meter_per_pixel");
    }
    if (
      expected_input_fps_ <= 0.0 ||
      publish_max_fps_ < 0.0 ||
      preview_max_fps_ <= 0.0 ||
      preview_max_width_ <= 0 ||
      preview_max_height_ <= 0 ||
      status_log_interval_sec_ <= 0.0 ||
      startup_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("invalid rate, preview, or status parameter");
    }
  }

  void onImage(sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    received_total_.fetch_add(1U, std::memory_order_relaxed);
    received_interval_.fetch_add(1U, std::memory_order_relaxed);

    const bool dimensions_valid =
      static_cast<int>(message->width) == camera_model_.image_width &&
      static_cast<int>(message->height) == camera_model_.image_height;
    const std::size_t minimum_step =
      static_cast<std::size_t>(camera_model_.image_width) * 3U;
    const bool memory_valid =
      message->step >= minimum_step &&
      message->data.size() >=
      static_cast<std::size_t>(message->step) * message->height;
    if (
      !dimensions_valid ||
      message->encoding != sensor_msgs::image_encodings::BGR8 ||
      !memory_valid)
    {
      invalid_total_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Rejected image: expected %dx%d bgr8, got %ux%u %s "
        "(step=%u, data=%zu).",
        camera_model_.image_width,
        camera_model_.image_height,
        message->width,
        message->height,
        message->encoding.c_str(),
        message->step,
        message->data.size());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      latest_input_ = std::move(message);
      latest_input_received_at_ = SteadyClock::now();
      ++input_generation_;
    }
    accepted_total_.fetch_add(1U, std::memory_order_relaxed);
    input_cv_.notify_one();
  }

  void processingLoop()
  {
    std::uint64_t processed_input_generation = 0U;

    while (!stop_.load(std::memory_order_acquire)) {
      sensor_msgs::msg::Image::ConstSharedPtr input;
      SteadyClock::time_point input_received_at;
      std::uint64_t generation = 0U;
      {
        std::unique_lock<std::mutex> lock(input_mutex_);
        input_cv_.wait(
          lock,
          [this, processed_input_generation]() {
            return
              stop_.load(std::memory_order_acquire) ||
              input_generation_ != processed_input_generation;
          });
        if (stop_.load(std::memory_order_acquire)) {
          break;
        }
        input = latest_input_;
        input_received_at = latest_input_received_at_;
        generation = input_generation_;
      }

      if (generation > processed_input_generation + 1U) {
        skipped_total_.fetch_add(
          generation - processed_input_generation - 1U,
          std::memory_order_relaxed);
        skipped_interval_.fetch_add(
          generation - processed_input_generation - 1U,
          std::memory_order_relaxed);
      }
      processed_input_generation = generation;

      try {
        const auto started_at = SteadyClock::now();
        const auto lut = std::atomic_load_explicit(
          &remap_lut_, std::memory_order_acquire);
        auto output = std::make_shared<BevFrame>();
        output->image = convertToBev(viewBgr8Image(*input), *lut);
        output->header = input->header;
        output->input_received_at = input_received_at;
        output->generation = generation;
        const auto finished_at = SteadyClock::now();

        const auto process_ns = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            finished_at - started_at).count());
        process_ns_interval_.fetch_add(process_ns, std::memory_order_relaxed);
        updateMaximum(process_ns_max_interval_, process_ns);
        processed_total_.fetch_add(1U, std::memory_order_relaxed);
        processed_interval_.fetch_add(1U, std::memory_order_relaxed);

        {
          std::lock_guard<std::mutex> lock(output_mutex_);
          std::atomic_store_explicit(
            &latest_output_,
            std::shared_ptr<const BevFrame>(std::move(output)),
            std::memory_order_release);
        }
        output_cv_.notify_all();
      } catch (const std::exception & exception) {
        processing_error_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "BEV conversion failed: %s",
          exception.what());
      }
    }
  }

  void publishingLoop()
  {
    std::uint64_t last_generation = 0U;
    SteadyClock::time_point last_published_at{};

    while (!stop_.load(std::memory_order_acquire)) {
      const auto frame = waitForNewOutput(last_generation);
      if (!frame) {
        continue;
      }
      last_generation = frame->generation;

      const auto now = SteadyClock::now();
      if (
        publish_max_fps_ > 0.0 &&
        last_published_at.time_since_epoch().count() != 0)
      {
        const auto minimum_period =
          std::chrono::duration<double>(1.0 / publish_max_fps_);
        if (now - last_published_at < minimum_period) {
          publish_throttled_total_.fetch_add(1U, std::memory_order_relaxed);
          continue;
        }
      }

      try {
        output_publisher_->publish(
          makeBgr8Message(*frame, output_frame_id_));
        last_published_at = now;
        published_total_.fetch_add(1U, std::memory_order_relaxed);
        published_interval_.fetch_add(1U, std::memory_order_relaxed);
      } catch (const std::exception & exception) {
        publish_error_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "BEV publish failed: %s",
          exception.what());
      }
    }
  }

  void previewLoop()
  {
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        preview_window_name_,
        std::min(preview_max_width_, bev_config_.output_width * 4),
        std::min(preview_max_height_, bev_config_.output_height));

      const auto preview_period =
        std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(1.0 / preview_max_fps_));
      auto next_preview_at = SteadyClock::now();
      bool window_was_visible = false;

      while (!stop_.load(std::memory_order_acquire)) {
        const auto now = SteadyClock::now();
        if (now < next_preview_at) {
          const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(
            next_preview_at - now);
          const int key = cv::waitKey(
            std::clamp(static_cast<int>(remaining.count()), 1, 10));
          if (key == 27 || key == 'q' || key == 'Q') {
            break;
          }
          continue;
        }

        const auto frame = std::atomic_load_explicit(
          &latest_output_, std::memory_order_acquire);
        if (frame) {
          cv::imshow(preview_window_name_, frame->image);
          previewed_total_.fetch_add(1U, std::memory_order_relaxed);
          previewed_interval_.fetch_add(1U, std::memory_order_relaxed);
          next_preview_at = now + preview_period;
        } else {
          next_preview_at = now + std::chrono::milliseconds(5);
        }

        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q') {
          break;
        }
        const double visible = cv::getWindowProperty(
          preview_window_name_, cv::WND_PROP_VISIBLE);
        if (visible >= 1.0) {
          window_was_visible = true;
        } else if (window_was_visible) {
          break;
        }
      }
    } catch (const cv::Exception & exception) {
      RCLCPP_WARN(
        get_logger(),
        "BEV preview disabled after OpenCV error: %s",
        exception.what());
    }

    try {
      cv::destroyWindow(preview_window_name_);
    } catch (const cv::Exception &) {
    }
    RCLCPP_INFO(
      get_logger(),
      "BEV preview closed; processing and ROS publishing continue.");
  }

  std::shared_ptr<const BevFrame> waitForNewOutput(
    const std::uint64_t last_generation)
  {
    std::unique_lock<std::mutex> lock(output_mutex_);
    output_cv_.wait(
      lock,
      [this, last_generation]() {
        const auto frame = std::atomic_load_explicit(
          &latest_output_, std::memory_order_acquire);
        return
          stop_.load(std::memory_order_acquire) ||
          (frame && frame->generation != last_generation);
      });
    if (stop_.load(std::memory_order_acquire)) {
      return nullptr;
    }
    return std::atomic_load_explicit(
      &latest_output_, std::memory_order_acquire);
  }

  static void updateMaximum(
    std::atomic<std::uint64_t> & target,
    const std::uint64_t candidate)
  {
    auto current = target.load(std::memory_order_relaxed);
    while (
      current < candidate &&
      !target.compare_exchange_weak(
        current,
        candidate,
        std::memory_order_relaxed,
        std::memory_order_relaxed))
    {
    }
  }

  void logStatus()
  {
    const auto now = SteadyClock::now();
    const double elapsed_sec =
      std::chrono::duration<double>(now - status_started_at_).count();
    status_started_at_ = now;

    const auto received =
      received_interval_.exchange(0U, std::memory_order_relaxed);
    const auto processed =
      processed_interval_.exchange(0U, std::memory_order_relaxed);
    const auto skipped =
      skipped_interval_.exchange(0U, std::memory_order_relaxed);
    const auto published =
      published_interval_.exchange(0U, std::memory_order_relaxed);
    const auto previewed =
      previewed_interval_.exchange(0U, std::memory_order_relaxed);
    const auto process_ns =
      process_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto process_ns_max =
      process_ns_max_interval_.exchange(0U, std::memory_order_relaxed);

    double latest_age_ms = 0.0;
    const auto latest = std::atomic_load_explicit(
      &latest_output_, std::memory_order_acquire);
    if (latest) {
      latest_age_ms =
        std::chrono::duration<double, std::milli>(
        now - latest->input_received_at).count();
    }
    const double average_process_ms =
      processed > 0U ?
      static_cast<double>(process_ns) /
      static_cast<double>(processed) / 1.0e6 :
      0.0;

    RCLCPP_INFO(
      get_logger(),
      "BEV status: input=%.1fHz (%llu total), processed=%.1fHz "
      "(%llu total, skipped=%llu), ROS=%.1fHz, preview=%.1fHz, "
      "remap=%.3f/%.3fms avg/max, latest_age=%.2fms, "
      "errors(invalid/process/publish)=%llu/%llu/%llu",
      static_cast<double>(received) / elapsed_sec,
      static_cast<unsigned long long>(
        received_total_.load(std::memory_order_relaxed)),
      static_cast<double>(processed) / elapsed_sec,
      static_cast<unsigned long long>(
        processed_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
        skipped_total_.load(std::memory_order_relaxed)),
      static_cast<double>(published) / elapsed_sec,
      static_cast<double>(previewed) / elapsed_sec,
      average_process_ms,
      static_cast<double>(process_ns_max) / 1.0e6,
      latest_age_ms,
      static_cast<unsigned long long>(
        invalid_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
        processing_error_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(
        publish_error_total_.load(std::memory_order_relaxed)));

    if (
      accepted_total_.load(std::memory_order_relaxed) == 0U &&
      std::chrono::duration<double>(now - node_started_at_).count() >=
      startup_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No valid input received on %s. Check that camera_driver publishing "
        "is enabled and the image is 960x540 bgr8.",
        input_topic_.c_str());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  double expected_input_fps_{120.0};
  bool publish_enabled_{true};
  double publish_max_fps_{0.0};
  bool preview_enabled_{true};
  double preview_max_fps_{30.0};
  std::string preview_window_name_;
  int preview_max_width_{1280};
  int preview_max_height_{900};
  double status_log_interval_sec_{5.0};
  double startup_timeout_sec_{5.0};

  RectifiedCameraModel camera_model_{};
  BevConfig bev_config_{};
  double valid_lut_percent_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex input_mutex_;
  std::condition_variable input_cv_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_input_;
  SteadyClock::time_point latest_input_received_at_;
  std::uint64_t input_generation_{0U};

  std::mutex output_mutex_;
  std::condition_variable output_cv_;
  std::shared_ptr<const BevFrame> latest_output_;
  std::shared_ptr<const RemapLut> remap_lut_;

  std::atomic<bool> stop_{false};
  std::thread processing_thread_;
  std::thread publishing_thread_;
  std::thread preview_thread_;

  const SteadyClock::time_point node_started_at_{SteadyClock::now()};
  SteadyClock::time_point status_started_at_{SteadyClock::now()};

  std::atomic<std::uint64_t> received_total_{0U};
  std::atomic<std::uint64_t> accepted_total_{0U};
  std::atomic<std::uint64_t> processed_total_{0U};
  std::atomic<std::uint64_t> skipped_total_{0U};
  std::atomic<std::uint64_t> published_total_{0U};
  std::atomic<std::uint64_t> previewed_total_{0U};
  std::atomic<std::uint64_t> publish_throttled_total_{0U};
  std::atomic<std::uint64_t> invalid_total_{0U};
  std::atomic<std::uint64_t> processing_error_total_{0U};
  std::atomic<std::uint64_t> publish_error_total_{0U};

  std::atomic<std::uint64_t> received_interval_{0U};
  std::atomic<std::uint64_t> processed_interval_{0U};
  std::atomic<std::uint64_t> skipped_interval_{0U};
  std::atomic<std::uint64_t> published_interval_{0U};
  std::atomic<std::uint64_t> previewed_interval_{0U};
  std::atomic<std::uint64_t> process_ns_interval_{0U};
  std::atomic<std::uint64_t> process_ns_max_interval_{0U};
};

}  // namespace bev_processor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<bev_processor::BevProcessorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("bev_processor"),
      "Failed to start BEV processor: %s",
      exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
