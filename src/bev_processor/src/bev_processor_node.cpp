#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>

#include "bev_processor/bev_geometry.hpp"
#include "bev_processor/cuda_bev_processor.hpp"

namespace bev_processor
{

namespace
{

using SteadyClock = std::chrono::steady_clock;
constexpr double kRadiansToDegrees =
  180.0 / 3.14159265358979323846;

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
  explicit BevProcessorNode(const rclcpp::NodeOptions & options)
  : Node("bev_processor", options)
  {
    declareParameters();
    readParameters();
    validateParameters();

    installProcessor(
      fallback_roll_deg_, fallback_pitch_down_deg_, nullptr);

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    input_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      image_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        onImage(std::move(message));
      });
    if (height_from_topic_enabled_) {
      const auto height_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
      rclcpp::SubscriptionOptions height_options;
      // This component uses intra-process transport for 120 FPS images, but
      // ROS 2 intra-process subscriptions only support volatile durability.
      // Keep the one-shot height on DDS so transient-local delivery works.
      height_options.use_intra_process_comm =
        rclcpp::IntraProcessSetting::Disable;
      height_subscription_ = create_subscription<std_msgs::msg::Float64>(
        height_topic_,
        height_qos,
        [this](std_msgs::msg::Float64::ConstSharedPtr message) {
          onHeight(std::move(message));
        },
        height_options);
    }
    if (imu_attitude_enabled_) {
      const auto imu_qos = rclcpp::SensorDataQoS().keep_last(
        static_cast<std::size_t>(imu_attitude_sample_count_));
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        imu_qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          onImu(std::move(message));
        });
    }

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
    const auto startup_processor = std::atomic_load_explicit(
      &gpu_processor_, std::memory_order_acquire);
    const std::string imu_startup_description =
      imu_attitude_enabled_ ? "on (" + imu_topic_ + ")" : "off";
    const std::string height_startup_description =
      height_from_topic_enabled_ ?
      "waiting (" + height_topic_ + ")" :
      "fallback parameter";

    RCLCPP_INFO(
      get_logger(),
      "==================== BEV PROCESSOR START ====================");
    RCLCPP_INFO(
      get_logger(),
      "BEV processor started: input=%s (%dx%d NV12, expected=%.1fHz), "
      "output=%s (%dx%d), "
      "range X=[%.2f, %.2f]m Y=[%.2f, %.2f]m, %.3fm/px, "
      "camera=(x=%.3f, y=%.3f, z=%.3fm, "
      "roll=%.2f, pitch_down=%.2f, yaw=%.2fdeg), "
      "valid_lut=%.2f%%, GPU=%s, processing=NV12-to-BEV/latest-only, "
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
      get_parameter("camera_x_m").as_double(),
      get_parameter("camera_y_m").as_double(),
      get_parameter("camera_z_m").as_double(),
      get_parameter("camera_roll_deg").as_double(),
      get_parameter("camera_downward_pitch_deg").as_double(),
      get_parameter("camera_yaw_deg").as_double(),
      valid_lut_percent_.load(std::memory_order_relaxed),
      startup_processor->deviceName().c_str(),
      publish_enabled_ ? "on" : "off",
      publish_max_fps_,
      preview_enabled_ ? "on" : "off",
      preview_max_fps_);
    RCLCPP_INFO(
      get_logger(),
      "Camera attitude: IMU=%s, startup samples=%d, "
      "fallback roll=%.2f/pitch_down=%.2f deg, fixed yaw=%.2f deg",
      imu_startup_description.c_str(),
      imu_attitude_sample_count_,
      fallback_roll_deg_,
      fallback_pitch_down_deg_,
      camera_yaw_deg_);
    RCLCPP_INFO(
      get_logger(),
      "Camera height: source=%s, fallback=%.3fm, accepted=[%.3f, %.3f]m",
      height_startup_description.c_str(),
      fallback_camera_height_m_,
      camera_height_min_m_,
      camera_height_max_m_);
    RCLCPP_INFO(
      get_logger(),
      "============================================================");
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
    declare_parameter<int>("preview_max_height", 720);

    declare_parameter<int>("input_width", 1280);
    declare_parameter<int>("input_height", 720);
    declare_parameter<double>("fx", 561.400939941);
    declare_parameter<double>("fy", 561.136352539);
    declare_parameter<double>("cx", 643.032653809);
    declare_parameter<double>("cy", 352.621124268);

    declare_parameter<double>("camera_x_m", 0.0);
    declare_parameter<double>("camera_y_m", 0.0);
    declare_parameter<double>("camera_z_m", 0.20);
    declare_parameter<double>("camera_roll_deg", 0.0);
    declare_parameter<double>("camera_downward_pitch_deg", 14.0);
    declare_parameter<double>("camera_yaw_deg", 0.0);
    declare_parameter<bool>("height_from_topic_enabled", true);
    declare_parameter<std::string>("height_topic", "/camera/height");
    declare_parameter<double>("camera_height_min_m", 0.10);
    declare_parameter<double>("camera_height_max_m", 1.00);
    declare_parameter<bool>("imu_attitude_enabled", true);
    declare_parameter<std::string>("imu_topic", "/camera/imu");
    declare_parameter<int>("imu_attitude_sample_count", 10);
    declare_parameter<double>("imu_attitude_max_stddev_deg", 0.5);
    declare_parameter<double>("imu_accel_min_mps2", 7.5);
    declare_parameter<double>("imu_accel_max_mps2", 12.0);

    declare_parameter<double>("x_min_m", 0.18);
    declare_parameter<double>("x_max_m", 3.0);
    declare_parameter<double>("y_min_m", -1.0);
    declare_parameter<double>("y_max_m", 1.0);
    declare_parameter<double>("meter_per_pixel", 0.01);
    declare_parameter<int>("output_width", 200);
    declare_parameter<int>("output_height", 282);

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
    fallback_camera_height_m_ = camera_model_.position_vehicle_m[2];
    applied_camera_height_m_.store(
      fallback_camera_height_m_, std::memory_order_relaxed);
    fallback_roll_deg_ = get_parameter("camera_roll_deg").as_double();
    fallback_pitch_down_deg_ =
      get_parameter("camera_downward_pitch_deg").as_double();
    camera_yaw_deg_ = get_parameter("camera_yaw_deg").as_double();
    camera_model_.rotation_vehicle_from_camera =
      mountRotationVehicleFromCamera(
      degToRad(fallback_roll_deg_),
      degToRad(fallback_pitch_down_deg_),
      degToRad(camera_yaw_deg_));

    height_from_topic_enabled_ =
      get_parameter("height_from_topic_enabled").as_bool();
    height_topic_ = get_parameter("height_topic").as_string();
    camera_height_min_m_ =
      get_parameter("camera_height_min_m").as_double();
    camera_height_max_m_ =
      get_parameter("camera_height_max_m").as_double();
    imu_attitude_enabled_ =
      get_parameter("imu_attitude_enabled").as_bool();
    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_attitude_sample_count_ = static_cast<int>(
      get_parameter("imu_attitude_sample_count").as_int());
    imu_attitude_max_stddev_deg_ =
      get_parameter("imu_attitude_max_stddev_deg").as_double();
    imu_accel_min_mps2_ =
      get_parameter("imu_accel_min_mps2").as_double();
    imu_accel_max_mps2_ =
      get_parameter("imu_accel_max_mps2").as_double();

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
    if (
      height_from_topic_enabled_ &&
      (height_topic_.empty() ||
      !std::isfinite(camera_height_min_m_) ||
      !std::isfinite(camera_height_max_m_) ||
      camera_height_min_m_ <= 0.0 ||
      camera_height_max_m_ <= camera_height_min_m_))
    {
      throw std::invalid_argument("invalid camera height topic parameter");
    }
    if (
      imu_attitude_enabled_ &&
      (imu_topic_.empty() ||
      imu_attitude_sample_count_ <= 0 ||
      imu_attitude_max_stddev_deg_ <= 0.0 ||
      imu_accel_min_mps2_ <= 0.0 ||
      imu_accel_max_mps2_ <= imu_accel_min_mps2_))
    {
      throw std::invalid_argument("invalid IMU attitude parameter");
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

  void installProcessor(
    const double roll_deg,
    const double pitch_down_deg,
    const char * source)
  {
    auto camera_model = camera_model_;
    camera_model.rotation_vehicle_from_camera =
      mountRotationVehicleFromCamera(
      degToRad(roll_deg),
      degToRad(pitch_down_deg),
      degToRad(camera_yaw_deg_));
    const auto lut = generateRemap(camera_model, bev_config_);
    auto processor = std::make_shared<CudaBevProcessor>(
      camera_model.image_width,
      camera_model.image_height,
      lut.map_x,
      lut.map_y);

    const int valid_pixels = cv::countNonZero(lut.valid_mask);
    const int output_pixels =
      bev_config_.output_width * bev_config_.output_height;
    const double valid_percent =
      100.0 * static_cast<double>(valid_pixels) /
      static_cast<double>(output_pixels);

    std::atomic_store_explicit(
      &gpu_processor_, std::move(processor), std::memory_order_release);
    valid_lut_percent_.store(valid_percent, std::memory_order_relaxed);
    applied_roll_deg_.store(roll_deg, std::memory_order_relaxed);
    applied_pitch_down_deg_.store(
      pitch_down_deg, std::memory_order_relaxed);
    applied_camera_height_m_.store(
      camera_model.position_vehicle_m[2], std::memory_order_relaxed);

    if (source != nullptr) {
      RCLCPP_INFO(
        get_logger(),
        "BEV LUT installed from %s: height=%.3fm, roll=%.3f deg, "
        "pitch_down=%.3f deg, fixed yaw=%.3f deg, valid=%.2f%%",
        source, camera_model.position_vehicle_m[2],
        roll_deg, pitch_down_deg, camera_yaw_deg_, valid_percent);
    }
  }

  void onHeight(std_msgs::msg::Float64::ConstSharedPtr message)
  {
    if (camera_height_finalized_.load(std::memory_order_acquire)) {
      return;
    }

    const double height_m = message->data;
    if (
      !std::isfinite(height_m) ||
      height_m < camera_height_min_m_ ||
      height_m > camera_height_max_m_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected camera height %.4fm from %s; allowed=[%.3f, %.3f]m",
        height_m, height_topic_.c_str(),
        camera_height_min_m_, camera_height_max_m_);
      return;
    }

    std::lock_guard<std::mutex> lock(pose_update_mutex_);
    if (camera_height_finalized_.load(std::memory_order_relaxed)) {
      return;
    }

    const double previous_height_m = camera_model_.position_vehicle_m[2];
    camera_model_.position_vehicle_m[2] = height_m;
    try {
      installProcessor(
        applied_roll_deg_.load(std::memory_order_relaxed),
        applied_pitch_down_deg_.load(std::memory_order_relaxed),
        "height estimator");
      camera_height_applied_.store(true, std::memory_order_release);
      camera_height_finalized_.store(true, std::memory_order_release);
      RCLCPP_INFO(
        get_logger(),
        "Camera height fixed for this run: %.4fm from %s",
        height_m, height_topic_.c_str());
    } catch (const std::exception & exception) {
      camera_model_.position_vehicle_m[2] = previous_height_m;
      RCLCPP_ERROR(
        get_logger(),
        "Could not install height-derived BEV LUT; keeping %.4fm fallback: %s",
        previous_height_m, exception.what());
    }
  }

  void onImu(sensor_msgs::msg::Imu::ConstSharedPtr message)
  {
    if (imu_attitude_finalized_.load(std::memory_order_acquire)) {
      return;
    }

    const double acceleration_x = message->linear_acceleration.x;
    const double acceleration_y = message->linear_acceleration.y;
    const double acceleration_z = message->linear_acceleration.z;
    const double magnitude = std::sqrt(
      acceleration_x * acceleration_x +
      acceleration_y * acceleration_y +
      acceleration_z * acceleration_z);
    if (
      !std::isfinite(magnitude) ||
      magnitude < imu_accel_min_mps2_ ||
      magnitude > imu_accel_max_mps2_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for valid IMU gravity: |a|=%.3f m/s^2, "
        "allowed=[%.3f, %.3f]",
        magnitude, imu_accel_min_mps2_, imu_accel_max_mps2_);
      return;
    }

    // The camera optical frame is RDF: +X right, +Y down, +Z forward.
    // A stationary accelerometer measures specific force opposite gravity,
    // hence an upright, level camera reads approximately (0, -g, 0).
    if (-acceleration_y <= 1.0e-6) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU acceleration does not describe an upright camera-frame pose: "
        "a=[%.3f, %.3f, %.3f] m/s^2",
        acceleration_x, acceleration_y, acceleration_z);
      return;
    }

    const auto attitude = cameraAttitudeFromSpecificForce(
      cv::Vec3d(acceleration_x, acceleration_y, acceleration_z));
    const double roll_deg = attitude.roll * kRadiansToDegrees;
    const double pitch_down_deg = attitude.pitch * kRadiansToDegrees;
    if (!std::isfinite(roll_deg) || !std::isfinite(pitch_down_deg)) {
      return;
    }

    imu_attitude_samples_.emplace_back(roll_deg, pitch_down_deg);
    while (
      static_cast<int>(imu_attitude_samples_.size()) >
      imu_attitude_sample_count_)
    {
      imu_attitude_samples_.pop_front();
    }
    if (
      static_cast<int>(imu_attitude_samples_.size()) <
      imu_attitude_sample_count_)
    {
      return;
    }

    double roll_sum = 0.0;
    double pitch_sum = 0.0;
    for (const auto & sample : imu_attitude_samples_) {
      roll_sum += sample.first;
      pitch_sum += sample.second;
    }
    const double roll_mean =
      roll_sum / static_cast<double>(imu_attitude_samples_.size());
    const double pitch_mean =
      pitch_sum / static_cast<double>(imu_attitude_samples_.size());
    double roll_squared_error_sum = 0.0;
    double pitch_squared_error_sum = 0.0;
    for (const auto & sample : imu_attitude_samples_) {
      const double roll_error = sample.first - roll_mean;
      const double pitch_error = sample.second - pitch_mean;
      roll_squared_error_sum += roll_error * roll_error;
      pitch_squared_error_sum += pitch_error * pitch_error;
    }
    const double roll_stddev = std::sqrt(
      roll_squared_error_sum /
      static_cast<double>(imu_attitude_samples_.size()));
    const double pitch_stddev = std::sqrt(
      pitch_squared_error_sum /
      static_cast<double>(imu_attitude_samples_.size()));
    if (
      roll_stddev > imu_attitude_max_stddev_deg_ ||
      pitch_stddev > imu_attitude_max_stddev_deg_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for stable IMU attitude: roll stddev=%.3f deg, "
        "pitch stddev=%.3f deg, limit=%.3f deg",
        roll_stddev, pitch_stddev, imu_attitude_max_stddev_deg_);
      return;
    }

    try {
      std::lock_guard<std::mutex> lock(pose_update_mutex_);
      if (imu_attitude_finalized_.load(std::memory_order_relaxed)) {
        return;
      }
      installProcessor(roll_mean, pitch_mean, "OAK IMU");
      imu_attitude_applied_.store(true, std::memory_order_release);
      imu_attitude_finalized_.store(true, std::memory_order_release);
      RCLCPP_INFO(
        get_logger(),
        "OAK IMU attitude fixed for this run: samples=%d, "
        "roll=%.3f deg (stddev=%.3f), "
        "pitch_down=%.3f deg (stddev=%.3f). "
        "Yaw remains the configured mounting value %.3f deg.",
        imu_attitude_sample_count_,
        roll_mean, roll_stddev,
        pitch_mean, pitch_stddev,
        camera_yaw_deg_);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not install IMU-derived BEV LUT; keeping parameter fallback: %s",
        exception.what());
      imu_attitude_finalized_.store(true, std::memory_order_release);
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
      static_cast<std::size_t>(camera_model_.image_width);
    const std::size_t nv12_rows =
      static_cast<std::size_t>(camera_model_.image_height) * 3U / 2U;
    const bool memory_valid =
      message->step >= minimum_step &&
      message->data.size() >=
      static_cast<std::size_t>(message->step) * nv12_rows;
    if (
      !dimensions_valid ||
      message->encoding != "nv12" ||
      !memory_valid)
    {
      invalid_total_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Rejected image: expected %dx%d nv12, got %ux%u %s "
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
        auto output = std::make_shared<BevFrame>();
        const auto processor = std::atomic_load_explicit(
          &gpu_processor_, std::memory_order_acquire);
        output->image = processor->process(
          input->data.data(),
          input->data.size(),
          static_cast<std::size_t>(input->step));
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

  static constexpr int kPreviewLeftMargin = 55;
  static constexpr int kPreviewRightMargin = 35;
  static constexpr int kPreviewTopMargin = 24;
  static constexpr int kPreviewBottomMargin = 30;

  static void drawPreviewText(
    cv::Mat & image,
    const std::string & text,
    const cv::Point & origin)
  {
    constexpr double font_scale = 0.32;
    constexpr int font_face = cv::FONT_HERSHEY_SIMPLEX;
    cv::putText(
      image, text, origin, font_face, font_scale,
      cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
  }

  cv::Mat makeCoordinatePreview(const cv::Mat & bev_image) const
  {
    constexpr double grid_step_m = 0.1;
    constexpr double label_step_m = 0.5;
    constexpr double epsilon = 1.0e-9;
    const cv::Scalar margin_color(245, 245, 245);
    const cv::Scalar grid_color(210, 210, 210);
    const cv::Scalar border_color(180, 180, 180);
    const cv::Scalar centerline_color(0, 200, 0);

    cv::Mat preview(
      bev_image.rows + kPreviewTopMargin + kPreviewBottomMargin,
      bev_image.cols + kPreviewLeftMargin + kPreviewRightMargin,
      CV_8UC3,
      margin_color);
    const cv::Rect bev_region(
      kPreviewLeftMargin,
      kPreviewTopMargin,
      bev_image.cols,
      bev_image.rows);
    bev_image.copyTo(preview(bev_region));
    cv::Mat displayed_bev = preview(bev_region);
    cv::Mat grid_overlay = displayed_bev.clone();

    const double first_x =
      std::ceil(bev_config_.x_min_m / grid_step_m) * grid_step_m;
    for (
      double x_m = first_x;
      x_m <= bev_config_.x_max_m + epsilon;
      x_m += grid_step_m)
    {
      const int row = std::clamp(
        static_cast<int>(std::lround(
          (bev_config_.x_max_m - x_m) /
          bev_config_.meter_per_pixel)),
        0,
        displayed_bev.rows - 1);
      cv::line(
        grid_overlay,
        cv::Point(0, row),
        cv::Point(displayed_bev.cols - 1, row),
        grid_color,
        1,
        cv::LINE_AA);
      const double label_units = x_m / label_step_m;
      if (std::abs(label_units - std::round(label_units)) < epsilon) {
        const std::string label = cv::format("X %.1f", x_m);
        int baseline = 0;
        const cv::Size label_size = cv::getTextSize(
          label, cv::FONT_HERSHEY_SIMPLEX, 0.32, 1, &baseline);
        drawPreviewText(
          preview,
          label,
          cv::Point(
            kPreviewLeftMargin - label_size.width - 5,
            kPreviewTopMargin + row + label_size.height / 2));
      }
    }

    cv::line(
      grid_overlay,
      cv::Point(0, displayed_bev.rows - 1),
      cv::Point(displayed_bev.cols - 1, displayed_bev.rows - 1),
      grid_color,
      1,
      cv::LINE_AA);
    const std::string minimum_x_label =
      cv::format("X %.2f", bev_config_.x_min_m);
    int minimum_x_baseline = 0;
    const cv::Size minimum_x_label_size = cv::getTextSize(
      minimum_x_label,
      cv::FONT_HERSHEY_SIMPLEX,
      0.32,
      1,
      &minimum_x_baseline);
    drawPreviewText(
      preview,
      minimum_x_label,
      cv::Point(
        kPreviewLeftMargin - minimum_x_label_size.width - 5,
        kPreviewTopMargin + displayed_bev.rows - 2));

    const double first_y =
      std::ceil(bev_config_.y_min_m / grid_step_m) * grid_step_m;
    for (
      double y_m = first_y;
      y_m <= bev_config_.y_max_m + epsilon;
      y_m += grid_step_m)
    {
      const int column = std::clamp(
        static_cast<int>(std::lround(
          (bev_config_.y_max_m - y_m) /
          bev_config_.meter_per_pixel)),
        0,
        displayed_bev.cols - 1);
      cv::line(
        grid_overlay,
        cv::Point(column, 0),
        cv::Point(column, displayed_bev.rows - 1),
        grid_color,
        1,
        cv::LINE_AA);
      const double label_units = y_m / label_step_m;
      if (std::abs(label_units - std::round(label_units)) < epsilon) {
        const std::string label = cv::format("Y %+.1f", y_m);
        int baseline = 0;
        const cv::Size label_size = cv::getTextSize(
          label, cv::FONT_HERSHEY_SIMPLEX, 0.32, 1, &baseline);
        drawPreviewText(
          preview,
          label,
          cv::Point(
            kPreviewLeftMargin + column - label_size.width / 2,
            kPreviewTopMargin + displayed_bev.rows + 16));
      }
    }

    cv::addWeighted(
      grid_overlay, 0.35, displayed_bev, 0.65, 0.0, displayed_bev);

    const int center_column = std::clamp(
      static_cast<int>(std::lround(
        bev_config_.y_max_m / bev_config_.meter_per_pixel)),
      0,
      displayed_bev.cols - 1);
    cv::line(
      displayed_bev,
      cv::Point(center_column, 0),
      cv::Point(center_column, displayed_bev.rows - 1),
      centerline_color,
      1,
      cv::LINE_AA);
    cv::rectangle(preview, bev_region, border_color, 1, cv::LINE_AA);

    const std::string direction_label = "+X forward / +Y left";
    int direction_baseline = 0;
    const cv::Size direction_size = cv::getTextSize(
      direction_label,
      cv::FONT_HERSHEY_SIMPLEX,
      0.32,
      1,
      &direction_baseline);
    drawPreviewText(
      preview,
      direction_label,
      cv::Point(
        kPreviewLeftMargin +
        (displayed_bev.cols - direction_size.width) / 2,
        15));
    return preview;
  }

  void previewLoop()
  {
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        preview_window_name_,
        std::min(
          preview_max_width_,
          bev_config_.output_width +
          kPreviewLeftMargin + kPreviewRightMargin),
        std::min(
          preview_max_height_,
          bev_config_.output_height +
          kPreviewTopMargin + kPreviewBottomMargin));

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
          cv::Mat preview_image = makeCoordinatePreview(frame->image);
          cv::imshow(preview_window_name_, preview_image);
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
    const bool imu_applied =
      imu_attitude_applied_.load(std::memory_order_acquire);
    const bool height_applied =
      camera_height_applied_.load(std::memory_order_acquire);

    RCLCPP_INFO(
      get_logger(),
      "\nBEV status: input=%.1fHz (%llu total), processed=%.1fHz "
      "(%llu total, skipped=%llu/%llu interval/total), "
      "ROS=%.1fHz, preview=%.1fHz, "
      "gpu=%.3f/%.3fms avg/max, latest_age=%.2fms, "
      "attitude=%s(roll=%.2f,pitch_down=%.2fdeg), "
      "height=%s(%.3fm), "
      "errors(invalid/process/publish)=%llu/%llu/%llu",
      static_cast<double>(received) / elapsed_sec,
      static_cast<unsigned long long>(
        received_total_.load(std::memory_order_relaxed)),
      static_cast<double>(processed) / elapsed_sec,
      static_cast<unsigned long long>(
        processed_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(skipped),
      static_cast<unsigned long long>(
        skipped_total_.load(std::memory_order_relaxed)),
      static_cast<double>(published) / elapsed_sec,
      static_cast<double>(previewed) / elapsed_sec,
      average_process_ms,
      static_cast<double>(process_ns_max) / 1.0e6,
      latest_age_ms,
      imu_applied ? "IMU" : "fallback",
      applied_roll_deg_.load(std::memory_order_relaxed),
      applied_pitch_down_deg_.load(std::memory_order_relaxed),
      height_applied ? "estimator" : "fallback",
      applied_camera_height_m_.load(std::memory_order_relaxed),
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
        "is enabled and the image is 1280x720 nv12.",
        input_topic_.c_str());
    }
    if (
      imu_attitude_enabled_ &&
      !imu_attitude_finalized_.load(std::memory_order_acquire) &&
      std::chrono::duration<double>(now - node_started_at_).count() >=
      startup_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No stable OAK IMU attitude has been accepted on %s; "
        "BEV is still using fallback roll=%.2f/pitch_down=%.2f deg.",
        imu_topic_.c_str(),
        fallback_roll_deg_,
        fallback_pitch_down_deg_);
    }
    if (
      height_from_topic_enabled_ &&
      !camera_height_finalized_.load(std::memory_order_acquire) &&
      std::chrono::duration<double>(now - node_started_at_).count() >=
      startup_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No valid one-shot camera height has been accepted on %s; "
        "BEV is still using %.3fm fallback.",
        height_topic_.c_str(),
        fallback_camera_height_m_);
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
  int preview_max_height_{720};
  double status_log_interval_sec_{5.0};
  double startup_timeout_sec_{5.0};
  bool imu_attitude_enabled_{true};
  std::string imu_topic_{"/camera/imu"};
  int imu_attitude_sample_count_{10};
  double imu_attitude_max_stddev_deg_{0.5};
  double imu_accel_min_mps2_{7.5};
  double imu_accel_max_mps2_{12.0};
  double fallback_roll_deg_{0.0};
  double fallback_pitch_down_deg_{14.0};
  double camera_yaw_deg_{0.0};
  bool height_from_topic_enabled_{true};
  std::string height_topic_{"/camera/height"};
  double camera_height_min_m_{0.10};
  double camera_height_max_m_{1.00};
  double fallback_camera_height_m_{0.20};

  RectifiedCameraModel camera_model_{};
  BevConfig bev_config_{};
  std::atomic<double> valid_lut_percent_{0.0};
  std::atomic<double> applied_roll_deg_{0.0};
  std::atomic<double> applied_pitch_down_deg_{14.0};
  std::atomic<double> applied_camera_height_m_{0.20};
  std::atomic<bool> imu_attitude_applied_{false};
  std::atomic<bool> imu_attitude_finalized_{false};
  std::atomic<bool> camera_height_applied_{false};
  std::atomic<bool> camera_height_finalized_{false};
  std::deque<std::pair<double, double>> imu_attitude_samples_;
  std::shared_ptr<CudaBevProcessor> gpu_processor_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr input_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr height_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::mutex pose_update_mutex_;

  std::mutex input_mutex_;
  std::condition_variable input_cv_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_input_;
  SteadyClock::time_point latest_input_received_at_;
  std::uint64_t input_generation_{0U};

  std::mutex output_mutex_;
  std::condition_variable output_cv_;
  std::shared_ptr<const BevFrame> latest_output_;

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

#ifdef BEV_PROCESSOR_STANDALONE
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<bev_processor::BevProcessorNode>(
        rclcpp::NodeOptions{}));
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
#else
RCLCPP_COMPONENTS_REGISTER_NODE(bev_processor::BevProcessorNode)
#endif
