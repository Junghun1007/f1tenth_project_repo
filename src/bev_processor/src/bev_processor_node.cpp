#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;

double degToRad(const double degrees)
{
  return degrees * kPi / 180.0;
}

cv::Matx33d rotationX(const double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return cv::Matx33d(
    1.0, 0.0, 0.0,
    0.0, c, -s,
    0.0, s, c);
}

cv::Matx33d rotationY(const double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return cv::Matx33d(
    c, 0.0, s,
    0.0, 1.0, 0.0,
    -s, 0.0, c);
}

cv::Matx33d rotationZ(const double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return cv::Matx33d(
    c, -s, 0.0,
    s, c, 0.0,
    0.0, 0.0, 1.0);
}

struct EulerAngles
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

EulerAngles quaternionToEuler(
  const double x, const double y, const double z, const double w)
{
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm < 1.0e-9) {
    throw std::invalid_argument("IMU orientation quaternion has zero length");
  }

  const double qx = x / norm;
  const double qy = y / norm;
  const double qz = z / norm;
  const double qw = w / norm;

  EulerAngles result;
  result.roll = std::atan2(
    2.0 * (qw * qx + qy * qz),
    1.0 - 2.0 * (qx * qx + qy * qy));
  result.pitch = std::asin(
    std::clamp(
      2.0 * (qw * qy - qz * qx), -1.0, 1.0));
  result.yaw = std::atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz));
  return result;
}

struct RectifiedCameraModel
{
  double fx;
  double fy;
  double cx;
  double cy;
  int image_width;
  int image_height;
  cv::Vec3d position_vehicle_m;
  cv::Matx33d rotation_vehicle_from_camera;
};

struct BEVConfig
{
  double x_min_m;
  double x_max_m;
  double y_min_m;
  double y_max_m;
  double meter_per_pixel;
  int output_width;
  int output_height;
};

struct RemapLut
{
  cv::Mat map_x;
  cv::Mat map_y;
  cv::Mat valid_mask;
};

RemapLut generateRemap(
  const RectifiedCameraModel & camera,
  const BEVConfig & bev)
{
  RemapLut lut{
    cv::Mat(bev.output_height, bev.output_width, CV_32FC1, cv::Scalar(-1.0F)),
    cv::Mat(bev.output_height, bev.output_width, CV_32FC1, cv::Scalar(-1.0F)),
    cv::Mat(bev.output_height, bev.output_width, CV_8UC1, cv::Scalar(0))};

  // P_c = R_cv * (P_v - C_v), where R_cv = R_vc^T.
  const cv::Matx33d rotation_camera_from_vehicle =
    camera.rotation_vehicle_from_camera.t();

  for (int v_bev = 0; v_bev < bev.output_height; ++v_bev) {
    const double x_vehicle =
      bev.x_max_m - (static_cast<double>(v_bev) + 0.5) * bev.meter_per_pixel;

    for (int u_bev = 0; u_bev < bev.output_width; ++u_bev) {
      const double y_vehicle =
        bev.y_max_m - (static_cast<double>(u_bev) + 0.5) * bev.meter_per_pixel;
      const cv::Vec3d point_vehicle(x_vehicle, y_vehicle, 0.0);
      const cv::Vec3d point_camera =
        rotation_camera_from_vehicle *
        (point_vehicle - camera.position_vehicle_m);

      if (point_camera[2] <= 1.0e-6) {
        continue;
      }

      const double u_source =
        camera.fx * point_camera[0] / point_camera[2] + camera.cx;
      const double v_source =
        camera.fy * point_camera[1] / point_camera[2] + camera.cy;

      // Keep one-pixel interpolation margin at the right and bottom edges.
      if (
        u_source < 0.0 || v_source < 0.0 ||
        u_source >= static_cast<double>(camera.image_width - 1) ||
        v_source >= static_cast<double>(camera.image_height - 1))
      {
        continue;
      }

      lut.map_x.at<float>(v_bev, u_bev) = static_cast<float>(u_source);
      lut.map_y.at<float>(v_bev, u_bev) = static_cast<float>(v_source);
      lut.valid_mask.at<std::uint8_t>(v_bev, u_bev) = 255U;
    }
  }
  return lut;
}

cv::Mat convertToBEV(const cv::Mat & rectified_image, const RemapLut & lut)
{
  cv::Mat bev_image;
  cv::remap(
    rectified_image,
    bev_image,
    lut.map_x,
    lut.map_y,
    cv::INTER_LINEAR,
    cv::BORDER_CONSTANT,
    cv::Scalar(0, 0, 0));
  return bev_image;
}

}  // namespace

class BevProcessorNode : public rclcpp::Node
{
public:
  BevProcessorNode()
  : Node("bev_processor_node")
  {
    declareParameters();
    readParameters();
    validateParameters();

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      image_qos,
      std::bind(&BevProcessorNode::onImage, this, std::placeholders::_1));
    if (publish_enabled_) {
      image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic_, image_qos);
    }

    if (use_imu_) {
      imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&BevProcessorNode::onImu, this, std::placeholders::_1));
    }

    rebuildRemap();
    initializePreview();
    process_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / processing_rate_hz_),
      std::bind(&BevProcessorNode::processLatestFrame, this));
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(status_log_interval_sec_),
      std::bind(&BevProcessorNode::logStatus, this));
    status_started_at_ = std::chrono::steady_clock::now();
    next_publish_at_ = status_started_at_;
    next_preview_at_ = status_started_at_;

    RCLCPP_INFO(
      get_logger(),
      "BEV processor started: input=%s, output=%s, CAM_A rectified input, "
      "processing=%.1fHz, ROS_publish=%s@%.1fHz, direct_preview=%s@%.1fHz, "
      "output_size=%dx%d, range X=[%.2f, %.2f]m Y=[%.2f, %.2f]m, "
      "resolution=%.3fm/px, imu=%s",
      input_topic_.c_str(), output_topic_.c_str(),
      processing_rate_hz_, publish_enabled_ ? "on" : "off", publish_rate_hz_,
      preview_enabled_ ? "on" : "off", preview_fps_,
      bev_config_.output_width, bev_config_.output_height,
      bev_config_.x_min_m, bev_config_.x_max_m,
      bev_config_.y_min_m, bev_config_.y_max_m,
      bev_config_.meter_per_pixel, use_imu_ ? "on" : "off");
  }

  ~BevProcessorNode() override
  {
    closePreview();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("input_topic", "/image/normal");
    declare_parameter<std::string>("output_topic", "/image/bev");
    declare_parameter<std::string>("output_frame_id", "front_axle_bev");
    declare_parameter<double>("processing_rate_hz", 30.0);
    declare_parameter<bool>("publish_enabled", true);
    declare_parameter<double>("publish_rate_hz", 15.0);
    declare_parameter<bool>("preview_enabled", false);
    declare_parameter<double>("preview_fps", 30.0);
    declare_parameter<std::string>("preview_window_name", "BEV processed image");
    declare_parameter<int>("input_width", 640);
    declare_parameter<int>("input_height", 480);
    declare_parameter<double>("fx", 320.0);
    declare_parameter<double>("fy", 320.0);
    declare_parameter<double>("cx", 319.5);
    declare_parameter<double>("cy", 239.5);
    declare_parameter<double>("camera_x_m", 0.0);
    declare_parameter<double>("camera_y_m", 0.0);
    declare_parameter<double>("camera_z_m", 0.25);
    declare_parameter<double>("camera_roll_deg", 0.0);
    declare_parameter<double>("camera_downward_pitch_deg", 15.0);
    declare_parameter<double>("camera_yaw_deg", 0.0);
    declare_parameter<double>("x_min_m", 0.0);
    declare_parameter<double>("x_max_m", 5.0);
    declare_parameter<double>("y_min_m", -2.0);
    declare_parameter<double>("y_max_m", 2.0);
    declare_parameter<double>("meter_per_pixel", 0.01);
    declare_parameter<int>("output_width", 400);
    declare_parameter<int>("output_height", 500);
    declare_parameter<bool>("use_imu", false);
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<bool>("imu_zero_on_start", true);
    declare_parameter<bool>("use_imu_yaw", false);
    declare_parameter<double>("imu_roll_sign", 1.0);
    declare_parameter<double>("imu_pitch_sign", 1.0);
    declare_parameter<double>("imu_yaw_sign", 1.0);
    declare_parameter<double>("imu_map_update_threshold_deg", 0.25);
    declare_parameter<double>("status_log_interval_sec", 5.0);
  }

  void readParameters()
  {
    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    processing_rate_hz_ = get_parameter("processing_rate_hz").as_double();
    publish_enabled_ = get_parameter("publish_enabled").as_bool();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    preview_enabled_ = get_parameter("preview_enabled").as_bool();
    preview_fps_ = get_parameter("preview_fps").as_double();
    preview_window_name_ = get_parameter("preview_window_name").as_string();
    input_width_ = static_cast<int>(get_parameter("input_width").as_int());
    input_height_ = static_cast<int>(get_parameter("input_height").as_int());
    fx_ = get_parameter("fx").as_double();
    fy_ = get_parameter("fy").as_double();
    cx_ = get_parameter("cx").as_double();
    cy_ = get_parameter("cy").as_double();
    camera_position_ = cv::Vec3d(
      get_parameter("camera_x_m").as_double(),
      get_parameter("camera_y_m").as_double(),
      get_parameter("camera_z_m").as_double());
    camera_roll_rad_ = degToRad(get_parameter("camera_roll_deg").as_double());
    camera_pitch_rad_ =
      degToRad(get_parameter("camera_downward_pitch_deg").as_double());
    camera_yaw_rad_ = degToRad(get_parameter("camera_yaw_deg").as_double());
    bev_config_ = BEVConfig{
      get_parameter("x_min_m").as_double(),
      get_parameter("x_max_m").as_double(),
      get_parameter("y_min_m").as_double(),
      get_parameter("y_max_m").as_double(),
      get_parameter("meter_per_pixel").as_double(),
      static_cast<int>(get_parameter("output_width").as_int()),
      static_cast<int>(get_parameter("output_height").as_int())};
    use_imu_ = get_parameter("use_imu").as_bool();
    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_zero_on_start_ = get_parameter("imu_zero_on_start").as_bool();
    use_imu_yaw_ = get_parameter("use_imu_yaw").as_bool();
    imu_roll_sign_ = get_parameter("imu_roll_sign").as_double();
    imu_pitch_sign_ = get_parameter("imu_pitch_sign").as_double();
    imu_yaw_sign_ = get_parameter("imu_yaw_sign").as_double();
    imu_update_threshold_rad_ =
      degToRad(get_parameter("imu_map_update_threshold_deg").as_double());
    status_log_interval_sec_ =
      get_parameter("status_log_interval_sec").as_double();
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
    if (publish_enabled_ && input_topic_ == output_topic_) {
      throw std::invalid_argument("input_topic and output_topic must differ");
    }
    if (preview_enabled_ && preview_window_name_.empty()) {
      throw std::invalid_argument(
              "preview_window_name must not be empty when preview is enabled");
    }
    if (input_width_ <= 1 || input_height_ <= 1 || fx_ <= 0.0 || fy_ <= 0.0) {
      throw std::invalid_argument("input size and focal lengths must be positive");
    }
    if (
      processing_rate_hz_ <= 0.0 || publish_rate_hz_ <= 0.0 ||
      preview_fps_ <= 0.0 || status_log_interval_sec_ <= 0.0 ||
      bev_config_.meter_per_pixel <= 0.0)
    {
      throw std::invalid_argument("rates and meter_per_pixel must be positive");
    }
    if (
      bev_config_.x_max_m <= bev_config_.x_min_m ||
      bev_config_.y_max_m <= bev_config_.y_min_m)
    {
      throw std::invalid_argument("BEV max bounds must be greater than min bounds");
    }

    const int expected_width = static_cast<int>(std::lround(
        (bev_config_.y_max_m - bev_config_.y_min_m) /
        bev_config_.meter_per_pixel));
    const int expected_height = static_cast<int>(std::lround(
        (bev_config_.x_max_m - bev_config_.x_min_m) /
        bev_config_.meter_per_pixel));
    if (
      expected_width != bev_config_.output_width ||
      expected_height != bev_config_.output_height)
    {
      throw std::invalid_argument(
              "output size does not match BEV bounds and meter_per_pixel");
    }
    if (use_imu_ && imu_topic_.empty()) {
      throw std::invalid_argument("imu_topic must not be empty when use_imu is true");
    }
  }

  cv::Matx33d mountRotationVehicleFromCamera() const
  {
    // At zero angles: camera +X=vehicle -Y, camera +Y=vehicle -Z,
    // camera +Z=vehicle +X. Positive mount pitch points the camera downward.
    const cv::Matx33d optical_to_vehicle(
      0.0, 0.0, 1.0,
      -1.0, 0.0, 0.0,
      0.0, -1.0, 0.0);
    return rotationZ(camera_yaw_rad_) *
           rotationY(camera_pitch_rad_) *
           rotationX(camera_roll_rad_) *
           optical_to_vehicle;
  }

  void rebuildRemap()
  {
    // IMU correction represents vehicle tilt relative to the flat-ground frame.
    // Yaw is normally excluded because BEV stays aligned to vehicle forward.
    const cv::Matx33d tilt =
      rotationZ(use_imu_yaw_ ? applied_imu_.yaw : 0.0) *
      rotationY(applied_imu_.pitch) *
      rotationX(applied_imu_.roll);

    const RectifiedCameraModel camera{
      fx_,
      fy_,
      cx_,
      cy_,
      input_width_,
      input_height_,
      tilt * camera_position_,
      tilt * mountRotationVehicleFromCamera()};
    remap_lut_ = generateRemap(camera, bev_config_);
    remap_ready_ = true;
    ++remap_update_count_;
  }

  void initializePreview()
  {
    if (!preview_enabled_) {
      return;
    }

    const bool display_available =
      std::getenv("DISPLAY") != nullptr ||
      std::getenv("WAYLAND_DISPLAY") != nullptr;
    if (!display_available) {
      preview_enabled_ = false;
      RCLCPP_WARN(
        get_logger(),
        "BEV direct preview disabled because no graphical display is available");
      return;
    }

    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        preview_window_name_,
        bev_config_.output_width,
        bev_config_.output_height);
      const cv::Mat waiting_image(
        bev_config_.output_height,
        bev_config_.output_width,
        CV_8UC3,
        cv::Scalar(0, 0, 0));
      cv::imshow(preview_window_name_, waiting_image);
      cv::waitKey(1);
      preview_window_created_ = true;
    } catch (const cv::Exception & exception) {
      preview_enabled_ = false;
      preview_window_created_ = false;
      RCLCPP_WARN(
        get_logger(),
        "BEV direct preview disabled because its window could not be created: %s",
        exception.what());
    }
  }

  void closePreview()
  {
    if (!preview_window_created_) {
      return;
    }
    try {
      cv::destroyWindow(preview_window_name_);
      cv::waitKey(1);
    } catch (const cv::Exception &) {
    }
    preview_window_created_ = false;
    preview_window_was_visible_ = false;
  }

  void pumpPreviewEvents()
  {
    if (!preview_enabled_ || !preview_window_created_) {
      return;
    }

    try {
      const int key = cv::waitKey(1) & 0xff;
      const double visible = cv::getWindowProperty(
        preview_window_name_, cv::WND_PROP_VISIBLE);
      if (visible >= 1.0) {
        preview_window_was_visible_ = true;
      }

      const bool keyboard_close =
        key == 'q' || key == 'Q' || key == 27;
      const bool window_close =
        preview_window_was_visible_ && visible < 1.0;
      if (keyboard_close || window_close) {
        RCLCPP_INFO(get_logger(), "BEV direct preview closed");
        closePreview();
        preview_enabled_ = false;
      }
    } catch (const cv::Exception & exception) {
      RCLCPP_WARN(
        get_logger(),
        "BEV direct preview disabled after a window error: %s",
        exception.what());
      closePreview();
      preview_enabled_ = false;
    }
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    ++received_count_;
    if (
      static_cast<int>(message->width) != input_width_ ||
      static_cast<int>(message->height) != input_height_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input is %ux%u but K_rect is configured for %dx%d; frame ignored",
        message->width, message->height, input_width_, input_height_);
      return;
    }
    latest_image_ = message;
    new_image_available_ = true;
  }

  void processLatestFrame()
  {
    pumpPreviewEvents();

    if (!new_image_available_ || !latest_image_ || !remap_ready_) {
      return;
    }

    const auto message = latest_image_;
    new_image_available_ = false;
    try {
      const auto cv_input = cv_bridge::toCvShare(message, "bgr8");
      const cv::Mat bev = convertToBEV(cv_input->image, remap_lut_);
      ++processed_count_;
      ++processed_status_count_;

      const auto now = std::chrono::steady_clock::now();
      if (
        preview_enabled_ && preview_window_created_ &&
        now >= next_preview_at_)
      {
        cv::imshow(preview_window_name_, bev);
        ++previewed_count_;
        ++previewed_status_count_;
        next_preview_at_ =
          now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / preview_fps_));
      }

      if (
        publish_enabled_ && image_publisher_ &&
        now >= next_publish_at_)
      {
        auto output =
          cv_bridge::CvImage(message->header, "bgr8", bev).toImageMsg();
        output->header.frame_id = output_frame_id_;
        image_publisher_->publish(*output);
        ++published_count_;
        ++published_status_count_;
        next_publish_at_ =
          now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / publish_rate_hz_));
      }
    } catch (const cv_bridge::Exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Image conversion failed: %s", exception.what());
    } catch (const cv::Exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "BEV remap failed: %s", exception.what());
    }
  }

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr message)
  {
    EulerAngles measured;
    try {
      measured = quaternionToEuler(
        message->orientation.x,
        message->orientation.y,
        message->orientation.z,
        message->orientation.w);
    } catch (const std::exception & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU orientation ignored: %s", exception.what());
      return;
    }

    if (!imu_reference_ready_) {
      imu_reference_ = imu_zero_on_start_ ? measured : EulerAngles{};
      imu_reference_ready_ = true;
      RCLCPP_INFO(
        get_logger(),
        "IMU orientation received; roll/pitch reference initialized");
    }

    EulerAngles candidate{
      imu_roll_sign_ * (measured.roll - imu_reference_.roll),
      imu_pitch_sign_ * (measured.pitch - imu_reference_.pitch),
      imu_yaw_sign_ * (measured.yaw - imu_reference_.yaw)};
    if (!use_imu_yaw_) {
      candidate.yaw = 0.0;
    }

    const double largest_change = std::max(
    {
      std::abs(candidate.roll - applied_imu_.roll),
      std::abs(candidate.pitch - applied_imu_.pitch),
      std::abs(candidate.yaw - applied_imu_.yaw)});
    if (largest_change < imu_update_threshold_rad_) {
      return;
    }

    applied_imu_ = candidate;
    rebuildRemap();
  }

  void logStatus()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::max(
      std::chrono::duration<double>(now - status_started_at_).count(),
      1.0e-6);
    const int valid_pixels = remap_ready_ ?
      cv::countNonZero(remap_lut_.valid_mask) : 0;
    const int total_pixels =
      bev_config_.output_width * bev_config_.output_height;
    const double valid_percent =
      total_pixels > 0 ? 100.0 * valid_pixels / total_pixels : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "BEV active: received=%llu, processed=%.1fHz/%llu, "
      "ROS_publish=%s/%.1fHz/%llu, direct_preview=%s/%.1fHz/%llu, "
      "valid_area=%.1f%%, LUT_updates=%llu",
      static_cast<unsigned long long>(received_count_),
      processed_status_count_ / elapsed,
      static_cast<unsigned long long>(processed_count_),
      publish_enabled_ ? "on" : "off",
      published_status_count_ / elapsed,
      static_cast<unsigned long long>(published_count_),
      preview_enabled_ ? "on" : "off",
      previewed_status_count_ / elapsed,
      static_cast<unsigned long long>(previewed_count_),
      valid_percent,
      static_cast<unsigned long long>(remap_update_count_));
    processed_status_count_ = 0;
    published_status_count_ = 0;
    previewed_status_count_ = 0;
    status_started_at_ = now;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  double processing_rate_hz_{30.0};
  bool publish_enabled_{true};
  double publish_rate_hz_{15.0};
  bool preview_enabled_{false};
  double preview_fps_{30.0};
  std::string preview_window_name_;
  bool preview_window_created_{false};
  bool preview_window_was_visible_{false};
  int input_width_{640};
  int input_height_{480};
  double fx_{320.0};
  double fy_{320.0};
  double cx_{319.5};
  double cy_{239.5};
  cv::Vec3d camera_position_{0.0, 0.0, 0.25};
  double camera_roll_rad_{0.0};
  double camera_pitch_rad_{0.0};
  double camera_yaw_rad_{0.0};
  BEVConfig bev_config_{};

  bool use_imu_{false};
  std::string imu_topic_;
  bool imu_zero_on_start_{true};
  bool use_imu_yaw_{false};
  double imu_roll_sign_{1.0};
  double imu_pitch_sign_{1.0};
  double imu_yaw_sign_{1.0};
  double imu_update_threshold_rad_{degToRad(0.25)};
  double status_log_interval_sec_{5.0};
  bool imu_reference_ready_{false};
  EulerAngles imu_reference_{};
  EulerAngles applied_imu_{};

  RemapLut remap_lut_;
  bool remap_ready_{false};
  std::uint64_t remap_update_count_{0};
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  bool new_image_available_{false};
  std::uint64_t received_count_{0};
  std::uint64_t processed_count_{0};
  std::uint64_t published_count_{0};
  std::uint64_t previewed_count_{0};
  std::uint64_t processed_status_count_{0};
  std::uint64_t published_status_count_{0};
  std::uint64_t previewed_status_count_{0};
  std::chrono::steady_clock::time_point status_started_at_;
  std::chrono::steady_clock::time_point next_publish_at_;
  std::chrono::steady_clock::time_point next_preview_at_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BevProcessorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("bev_processor_node"),
      "Node initialization failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
