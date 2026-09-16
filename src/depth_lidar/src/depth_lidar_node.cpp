#include "depth_lidar/depth_lidar_geometry.hpp"
#include "depth_lidar/depth_lidar_preview.hpp"
#include "depth_lidar/depth_lidar_stabilization.hpp"

#include <depthai/depthai.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <atomic>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace depth_lidar
{

namespace
{
using namespace std::chrono_literals;
constexpr std::uint32_t kColorSensorWidth = 1280U;
constexpr std::uint32_t kColorSensorHeight = 800U;

struct NodeConfig
{
  double camera_fps{60.0};
  std::string camera_resolution{"400p"};
  std::string depth_mode{"high_density"};
  int confidence_threshold{200};
  bool left_right_check{true};
  bool subpixel{false};
  bool extended_disparity{false};
  std::string median_filter{"3x3"};
  ProjectionConfig projection;
  ClusterConfig cluster;
  GroundConfig ground;
  StabilizationConfig stabilization;
  bool nv12_enabled{true};
  double nv12_fps{60.0};
  int nv12_width{1280};
  int nv12_height{800};
  bool preview_enabled{true};
  bool preview_gui{false};
  double preview_fps{10.0};
  int preview_size_px{700};
  int preview_scale{3};
  bool stereo_preview_enabled{false};
  bool stereo_preview_gui{false};
  double stereo_preview_fps{15.0};
  double bev_x_min_m{0.0};
  double bev_x_max_m{3.0};
  double bev_y_min_m{-0.6};
  double bev_y_max_m{0.6};
  double bev_meter_per_pixel{0.01};
  double sensor_x_m{-0.16};
  double sensor_y_m{0.0};
  double sensor_yaw_deg{0.0};
  double metrics_interval_sec{1.0};
  std::string frame_id{"depth_lidar"};
};

bool isOneOf(const std::string & value, const std::vector<std::string> & choices)
{
  return std::find(choices.begin(), choices.end(), value) != choices.end();
}

bool cameraConfigChanged(const NodeConfig & lhs, const NodeConfig & rhs)
{
  return lhs.camera_fps != rhs.camera_fps || lhs.camera_resolution != rhs.camera_resolution
         || lhs.depth_mode != rhs.depth_mode || lhs.confidence_threshold != rhs.confidence_threshold
         || lhs.left_right_check != rhs.left_right_check || lhs.subpixel != rhs.subpixel
         || lhs.extended_disparity != rhs.extended_disparity
         || lhs.median_filter != rhs.median_filter || lhs.nv12_enabled != rhs.nv12_enabled
         || lhs.nv12_fps != rhs.nv12_fps || lhs.nv12_width != rhs.nv12_width
         || lhs.nv12_height != rhs.nv12_height
         || lhs.stereo_preview_enabled != rhs.stereo_preview_enabled;
}

bool validateNodeConfig(const NodeConfig & config, std::string & reason)
{
  if (config.camera_fps < 1.0 || config.camera_fps > 120.0) {
    reason = "camera.fps must be in [1.0, 120.0]";
    return false;
  }
  if (!isOneOf(config.camera_resolution, {"400p", "480p", "720p", "800p"})) {
    reason = "camera.resolution must be one of: 400p, 480p, 720p, 800p";
    return false;
  }
  if (!isOneOf(config.depth_mode, {"default", "high_density", "high_accuracy"})) {
    reason = "depth.mode must be one of: default, high_density, high_accuracy";
    return false;
  }
  if (config.confidence_threshold < 0 || config.confidence_threshold > 255) {
    reason = "depth.confidence_threshold must be in [0, 255]";
    return false;
  }
  if (config.subpixel && config.extended_disparity) {
    reason = "depth.subpixel and depth.extended_disparity cannot both be enabled";
    return false;
  }
  if (!isOneOf(config.median_filter, {"off", "3x3", "5x5", "7x7"})) {
    reason = "depth.median_filter must be one of: off, 3x3, 5x5, 7x7";
    return false;
  }
  if (config.preview_fps <= 0.0 || config.preview_fps > 120.0) {
    reason = "preview.fps must be in (0.0, 120.0]";
    return false;
  }
  if (!std::isfinite(config.stereo_preview_fps)
      || config.stereo_preview_fps <= 0.0 || config.stereo_preview_fps > 120.0) {
    reason = "stereo_preview.fps must be in (0.0, 120.0]";
    return false;
  }
  if (config.preview_size_px < 240 || config.preview_size_px > 2000) {
    reason = "preview.size_px must be in [240, 2000]";
    return false;
  }
  if (config.metrics_interval_sec < 0.1 || config.metrics_interval_sec > 60.0) {
    reason = "metrics.print_interval_sec must be in [0.1, 60.0]";
    return false;
  }
  if (config.frame_id.empty()) {
    reason = "frame_id cannot be empty";
    return false;
  }
  if (config.nv12_fps < 1.0 || config.nv12_fps > 120.0) {
    reason = "nv12.fps must be in [1.0, 120.0]";
    return false;
  }
  if (config.nv12_width < 2 || config.nv12_height < 2
      || config.nv12_width > static_cast<int>(kColorSensorWidth)
      || config.nv12_height > static_cast<int>(kColorSensorHeight) || config.nv12_width % 2 != 0
      || config.nv12_height % 2 != 0)
  {
    reason = "nv12 width and height must be even and no larger than 1280x800";
    return false;
  }
  if (config.preview_scale < 1 || config.preview_scale > 8) {
    reason = "preview.scale must be in [1, 8]";
    return false;
  }
  if (!std::isfinite(config.bev_x_min_m) || !std::isfinite(config.bev_x_max_m)
      || !std::isfinite(config.bev_y_min_m) || !std::isfinite(config.bev_y_max_m)
      || !std::isfinite(config.bev_meter_per_pixel) || config.bev_x_min_m < 0.0
      || config.bev_x_max_m <= config.bev_x_min_m || config.bev_y_max_m <= config.bev_y_min_m
      || config.bev_meter_per_pixel <= 0.0)
  {
    reason = "BEV extents must be finite and ordered with positive meter_per_pixel";
    return false;
  }
  const double bev_width = (config.bev_y_max_m - config.bev_y_min_m) / config.bev_meter_per_pixel;
  const double bev_height = (config.bev_x_max_m - config.bev_x_min_m) / config.bev_meter_per_pixel;
  if (bev_width < 1.0 || bev_height < 1.0 || bev_width > 4096.0 || bev_height > 4096.0) {
    reason = "BEV output dimensions must be in [1, 4096]";
    return false;
  }
  if (!std::isfinite(config.sensor_x_m) || !std::isfinite(config.sensor_y_m)
      || !std::isfinite(config.sensor_yaw_deg))
  {
    reason = "sensor pose must be finite";
    return false;
  }
  if (!validateProjectionConfig(config.projection, reason)) {
    return false;
  }
  return validateClusterConfig(config.cluster, reason) && validateGroundConfig(config.ground, reason)
    && validateStabilizationConfig(config.stabilization, reason);
}

std::pair<std::uint32_t, std::uint32_t> parseResolution(const std::string & value)
{
  if (value == "400p") {
    return {640U, 400U};
  }
  if (value == "480p") {
    return {640U, 480U};
  }
  if (value == "720p") {
    return {1280U, 720U};
  }
  if (value == "800p") {
    return {1280U, 800U};
  }
  throw std::invalid_argument("unsupported camera resolution: " + value);
}

dai::node::StereoDepth::PresetMode parseDepthMode(const std::string & value)
{
  if (value == "default") {
    return dai::node::StereoDepth::PresetMode::DEFAULT;
  }
  if (value == "high_accuracy") {
    return dai::node::StereoDepth::PresetMode::FAST_ACCURACY;
  }
  if (value == "high_density") {
    return dai::node::StereoDepth::PresetMode::FAST_DENSITY;
  }
  throw std::invalid_argument("unsupported depth mode: " + value);
}

dai::StereoDepthConfig::MedianFilter parseMedianFilter(const std::string & value)
{
  if (value == "off") {
    return dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF;
  }
  if (value == "3x3") {
    return dai::StereoDepthConfig::MedianFilter::KERNEL_3x3;
  }
  if (value == "5x5") {
    return dai::StereoDepthConfig::MedianFilter::KERNEL_5x5;
  }
  if (value == "7x7") {
    return dai::StereoDepthConfig::MedianFilter::KERNEL_7x7;
  }
  throw std::invalid_argument("unsupported median filter: " + value);
}

cv::Mat makeScanPreview(const DetectionResult & detection,
  const std::string & ground_status,
  const NodeConfig & config,
  const double depth_rx_fps,
  const double nv12_rx_fps,
  const double object_processing_fps,
  const double object_processing_ms,
  const bool waiting_for_depth = false)
{
  cv::Mat image = makeRadarPreview(detection, config.preview_size_px, config.projection.max_range_m,
    ground_status.rfind("VALID |", 0) == 0);
  const int height = image.rows;
  const auto draw_status = [&](const std::string & text, const int row) {
    int baseline = 0;
    const auto size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.36, 1, &baseline);
    const double scale = 0.36 * std::min(1.0,
      static_cast<double>(image.cols - 16) / std::max(1, size.width));
    cv::putText(image, text, cv::Point(8, row), cv::FONT_HERSHEY_SIMPLEX,
      scale, cv::Scalar(65, 65, 65), 1, cv::LINE_AA);
  };
  std::ostringstream receive_status;
  receive_status << std::fixed << std::setprecision(1) << "DEPTH RX " << depth_rx_fps
                 << " FPS | NV12 RX " << nv12_rx_fps << " FPS";
  draw_status(receive_status.str(), height - 28);
  std::ostringstream processing_status;
  processing_status << std::fixed << std::setprecision(1) << "CLUSTER+POS " << object_processing_fps
                    << " FPS (" << std::setprecision(3) << object_processing_ms << " ms AVG) | OBJECTS "
                    << detection.obstacles.size();
  draw_status(processing_status.str(), height - 10);
  draw_status(std::string(config.stereo_preview_enabled ? "C: CAMERA OFF" : "C: CAMERA ON")
    + " | GROUND: LIVE MSAC", 54);
  draw_status(ground_status + (waiting_for_depth ? " | WAITING FOR DEPTH" : ""), 72);
  return image;
}

// View only; the owning ImgFrame stays alive until rendering completes.
cv::Mat stereoGrayFrame(dai::ImgFrame & frame)
{
  const int width = static_cast<int>(frame.getWidth());
  const int height = static_cast<int>(frame.getHeight());
  const auto type = frame.getType();
  const std::size_t stride = frame.getStride() == 0U
    ? static_cast<std::size_t>(std::max(0, width)) : frame.getStride();
  auto && bytes = frame.getData();
  if (width <= 0 || height <= 0 || stride < static_cast<std::size_t>(width)
      || (type != dai::ImgFrame::Type::RAW8 && type != dai::ImgFrame::Type::GRAY8
          && type != dai::ImgFrame::Type::YUV400p)
      || bytes.size() < stride * static_cast<std::size_t>(height - 1) + width)
  {
    throw std::runtime_error("invalid rectified grayscale frame");
  }
  return cv::Mat(height, width, CV_8UC1, bytes.data(), stride);
}

sensor_msgs::msg::Image matToImageMessage(const cv::Mat & image,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::Image message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.height = static_cast<std::uint32_t>(image.rows);
  message.width = static_cast<std::uint32_t>(image.cols);
  message.encoding = sensor_msgs::image_encodings::BGR8;
  message.is_bigendian = false;
  message.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.cols * image.elemSize());
  const std::size_t bytes = message.step * message.height;
  message.data.resize(bytes);
  std::memcpy(message.data.data(), image.data, bytes);
  return message;
}

sensor_msgs::msg::PointCloud2 obstacleMessage(const DetectionResult & detection,
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(message);
  using Field = sensor_msgs::msg::PointField;
  modifier.setPointCloud2Fields(6, "x", 1, Field::FLOAT32, "y", 1, Field::FLOAT32,
    "z", 1, Field::FLOAT32, "radius", 1, Field::FLOAT32, "point_count", 1, Field::UINT32,
    "observation_age_sec", 1, Field::FLOAT32);
  modifier.resize(detection.obstacles.size());
  message.is_dense = true;
  // Avoid creating iterators over an empty cloud on ROS versions whose iterator
  // implementation takes &data.front(). The empty message still clears results.
  if (detection.obstacles.empty()) { return message; }
  sensor_msgs::PointCloud2Iterator<float> x(message, "x"), y(message, "y"), z(message, "z"), radius(message, "radius");
  sensor_msgs::PointCloud2Iterator<std::uint32_t> count(message, "point_count");
  sensor_msgs::PointCloud2Iterator<float> age(message, "observation_age_sec");
  for (const auto & obstacle : detection.obstacles) {
    *x = static_cast<float>(obstacle.forward_m);
    *y = static_cast<float>(obstacle.left_m);
    *z = 0.0F;
    *radius = static_cast<float>(obstacle.radius_m);
    *count = static_cast<std::uint32_t>(obstacle.support_points);
    *age = static_cast<float>(obstacle.observation_age_sec);
    ++x; ++y; ++z; ++radius; ++count; ++age;
  }
  return message;
}

} // namespace

class DepthLidarNode : public rclcpp::Node
{
public:
  DepthLidarNode() : Node("depth_lidar")
  {
    config_.camera_fps = declare_parameter<double>("camera.fps", config_.camera_fps);
    config_.camera_resolution =
      declare_parameter<std::string>("camera.resolution", config_.camera_resolution);
    config_.depth_mode = declare_parameter<std::string>("depth.mode", config_.depth_mode);
    config_.confidence_threshold =
      declare_parameter<int>("depth.confidence_threshold", config_.confidence_threshold);
    config_.left_right_check =
      declare_parameter<bool>("depth.left_right_check", config_.left_right_check);
    config_.subpixel = declare_parameter<bool>("depth.subpixel", config_.subpixel);
    config_.extended_disparity =
      declare_parameter<bool>("depth.extended_disparity", config_.extended_disparity);
    config_.median_filter =
      declare_parameter<std::string>("depth.median_filter", config_.median_filter);
    config_.projection.roi_width_ratio =
      declare_parameter<double>("roi.width_ratio", config_.projection.roi_width_ratio);
    config_.projection.roi_height_ratio =
      declare_parameter<double>("roi.height_ratio", config_.projection.roi_height_ratio);
    config_.projection.roi_bottom_offset_ratio =
      declare_parameter<double>("roi.bottom_offset_ratio",
        config_.projection.roi_bottom_offset_ratio);
    config_.projection.min_range_m =
      declare_parameter<double>("range.min_m", config_.projection.min_range_m);
    config_.projection.max_range_m =
      declare_parameter<double>("range.max_m", config_.projection.max_range_m);
    config_.projection.range_offset_m =
      declare_parameter<double>("range.offset_m", config_.projection.range_offset_m);
    config_.projection.pixel_stride =
      declare_parameter<int>("points.pixel_stride", config_.projection.pixel_stride);
    config_.cluster.min_points = declare_parameter<int>("cluster.min_points", config_.cluster.min_points);
    config_.cluster.neighbor_distance_m =
      declare_parameter<double>("cluster.neighbor_distance_m", config_.cluster.neighbor_distance_m);
    config_.cluster.radius_margin_m =
      declare_parameter<double>("cluster.radius_margin_m", config_.cluster.radius_margin_m);
    config_.cluster.min_radius_m =
      declare_parameter<double>("cluster.min_radius_m", config_.cluster.min_radius_m);
    config_.ground.roi_width_ratio =
      declare_parameter<double>("ground.roi_width_ratio", config_.ground.roi_width_ratio);
    config_.ground.roi_height_ratio =
      declare_parameter<double>("ground.roi_height_ratio", config_.ground.roi_height_ratio);
    config_.ground.roi_bottom_offset_ratio =
      declare_parameter<double>("ground.roi_bottom_offset_ratio", config_.ground.roi_bottom_offset_ratio);
    config_.ground.pixel_stride =
      declare_parameter<int>("ground.pixel_stride", config_.ground.pixel_stride);
    config_.ground.max_samples =
      declare_parameter<int>("ground.max_samples", config_.ground.max_samples);
    config_.ground.max_iterations =
      declare_parameter<int>("ground.max_iterations", config_.ground.max_iterations);
    config_.ground.min_depth_m =
      declare_parameter<double>("ground.min_depth_m", config_.ground.min_depth_m);
    config_.ground.max_depth_m =
      declare_parameter<double>("ground.max_depth_m", config_.ground.max_depth_m);
    config_.ground.inlier_distance_m =
      declare_parameter<double>("ground.inlier_distance_m", config_.ground.inlier_distance_m);
    config_.ground.min_inlier_points =
      declare_parameter<int>("ground.min_inlier_points", config_.ground.min_inlier_points);
    config_.ground.min_inlier_ratio =
      declare_parameter<double>("ground.min_inlier_ratio", config_.ground.min_inlier_ratio);
    config_.ground.min_spread_m =
      declare_parameter<double>("ground.min_spread_m", config_.ground.min_spread_m);
    config_.ground.max_rmse_m =
      declare_parameter<double>("ground.max_rmse_m", config_.ground.max_rmse_m);
    config_.ground.reference_up_x =
      declare_parameter<double>("ground.reference_up_x", config_.ground.reference_up_x);
    config_.ground.reference_up_y =
      declare_parameter<double>("ground.reference_up_y", config_.ground.reference_up_y);
    config_.ground.reference_up_z =
      declare_parameter<double>("ground.reference_up_z", config_.ground.reference_up_z);
    config_.ground.max_tilt_deg =
      declare_parameter<double>("ground.max_tilt_deg", config_.ground.max_tilt_deg);
    config_.ground.min_camera_height_m =
      declare_parameter<double>("ground.min_camera_height_m", config_.ground.min_camera_height_m);
    config_.ground.max_camera_height_m =
      declare_parameter<double>("ground.max_camera_height_m", config_.ground.max_camera_height_m);
    config_.ground.min_height_m =
      declare_parameter<double>("ground.min_height_m", config_.ground.min_height_m);
    config_.ground.max_height_m =
      declare_parameter<double>("ground.max_height_m", config_.ground.max_height_m);
    config_.ground.noise_scale =
      declare_parameter<double>("ground.noise_scale", config_.ground.noise_scale);
    config_.ground.release_ratio =
      declare_parameter<double>("ground.release_ratio", config_.ground.release_ratio);
    config_.ground.reset_history_angle_deg =
      declare_parameter<double>("ground.reset_history_angle_deg", config_.ground.reset_history_angle_deg);
    config_.ground.reset_history_height_m =
      declare_parameter<double>("ground.reset_history_height_m", config_.ground.reset_history_height_m);
    config_.stabilization.enabled =
      declare_parameter<bool>("stabilization.enabled", config_.stabilization.enabled);
    config_.stabilization.confirm_hits =
      declare_parameter<int>("stabilization.confirm_hits", config_.stabilization.confirm_hits);
    config_.stabilization.window_frames =
      declare_parameter<int>("stabilization.window_frames", config_.stabilization.window_frames);
    config_.stabilization.hold_sec =
      declare_parameter<double>("stabilization.hold_sec", config_.stabilization.hold_sec);
    config_.stabilization.match_distance_m =
      declare_parameter<double>("stabilization.match_distance_m", config_.stabilization.match_distance_m);
    config_.stabilization.max_frame_gap_sec =
      declare_parameter<double>("stabilization.max_frame_gap_sec", config_.stabilization.max_frame_gap_sec);
    config_.nv12_enabled = declare_parameter<bool>("nv12.enabled", config_.nv12_enabled);
    config_.nv12_fps = declare_parameter<double>("nv12.fps", config_.nv12_fps);
    config_.nv12_width = declare_parameter<int>("nv12.width", config_.nv12_width);
    config_.nv12_height = declare_parameter<int>("nv12.height", config_.nv12_height);
    config_.preview_enabled = declare_parameter<bool>("preview.enabled", config_.preview_enabled);
    config_.preview_gui = declare_parameter<bool>("preview.gui", config_.preview_gui);
    config_.preview_fps = declare_parameter<double>("preview.fps", config_.preview_fps);
    config_.preview_size_px = declare_parameter<int>("preview.size_px", config_.preview_size_px);
    config_.preview_scale = declare_parameter<int>("preview.scale", config_.preview_scale);
    config_.stereo_preview_enabled =
      declare_parameter<bool>("stereo_preview.enabled", config_.stereo_preview_enabled);
    config_.stereo_preview_gui =
      declare_parameter<bool>("stereo_preview.gui", config_.stereo_preview_gui);
    config_.stereo_preview_fps =
      declare_parameter<double>("stereo_preview.fps", config_.stereo_preview_fps);
    config_.bev_x_min_m = declare_parameter<double>("bev.x_min_m", config_.bev_x_min_m);
    config_.bev_x_max_m = declare_parameter<double>("bev.x_max_m", config_.bev_x_max_m);
    config_.bev_y_min_m = declare_parameter<double>("bev.y_min_m", config_.bev_y_min_m);
    config_.bev_y_max_m = declare_parameter<double>("bev.y_max_m", config_.bev_y_max_m);
    config_.bev_meter_per_pixel =
      declare_parameter<double>("bev.meter_per_pixel", config_.bev_meter_per_pixel);
    config_.sensor_x_m = declare_parameter<double>("sensor.x_m", config_.sensor_x_m);
    config_.sensor_y_m = declare_parameter<double>("sensor.y_m", config_.sensor_y_m);
    config_.sensor_yaw_deg = declare_parameter<double>("sensor.yaw_deg", config_.sensor_yaw_deg);
    config_.metrics_interval_sec =
      declare_parameter<double>("metrics.print_interval_sec", config_.metrics_interval_sec);
    config_.frame_id = declare_parameter<std::string>("frame_id", config_.frame_id);

    std::string reason;
    if (!validateNodeConfig(config_, reason)) {
      throw std::invalid_argument("invalid initial parameter: " + reason);
    }
    for (const auto & entry : get_node_parameters_interface()->get_parameter_overrides()) {
      if (entry.first.rfind("floor.", 0) == 0) {
        throw std::invalid_argument("floor.* parameters were removed; use the current ground.* YAML");
      }
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    obstacles_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/obstacles", qos);
    ground_status_publisher_ = create_publisher<std_msgs::msg::String>("~/ground_status",
      rclcpp::QoS(1).reliable().transient_local());
    ground_valid_publisher_ = create_publisher<std_msgs::msg::Bool>("~/ground_valid",
      rclcpp::QoS(1).reliable().transient_local());
    preview_publisher_ = create_publisher<sensor_msgs::msg::Image>("~/preview", qos);
    stereo_preview_publisher_ = create_publisher<sensor_msgs::msg::Image>("~/stereo_preview", qos);
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&DepthLidarNode::onParameters, this, std::placeholders::_1));
    worker_ = std::thread(&DepthLidarNode::cameraLoop, this);
  }

  ~DepthLidarNode() override
  {
    stop_requested_.store(true);
    restart_requested_.store(true);
    if (worker_.joinable()) {
      worker_.join();
    }
    cv::destroyAllWindows();
  }

private:
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    NodeConfig previous;
    NodeConfig next;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      previous = config_;
      next = config_;
    }

    try {
      for (const auto & parameter : parameters) {
        const std::string & name = parameter.get_name();
        if (name == "camera.fps") {
          next.camera_fps = parameter.as_double();
        } else if (name == "camera.resolution") {
          next.camera_resolution = parameter.as_string();
        } else if (name == "depth.mode") {
          next.depth_mode = parameter.as_string();
        } else if (name == "depth.confidence_threshold") {
          next.confidence_threshold = static_cast<int>(parameter.as_int());
        } else if (name == "depth.left_right_check") {
          next.left_right_check = parameter.as_bool();
        } else if (name == "depth.subpixel") {
          next.subpixel = parameter.as_bool();
        } else if (name == "depth.extended_disparity") {
          next.extended_disparity = parameter.as_bool();
        } else if (name == "depth.median_filter") {
          next.median_filter = parameter.as_string();
        } else if (name == "roi.width_ratio") {
          next.projection.roi_width_ratio = parameter.as_double();
        } else if (name == "roi.height_ratio") {
          next.projection.roi_height_ratio = parameter.as_double();
        } else if (name == "roi.bottom_offset_ratio") {
          next.projection.roi_bottom_offset_ratio = parameter.as_double();
        } else if (name == "range.min_m") {
          next.projection.min_range_m = parameter.as_double();
        } else if (name == "range.max_m") {
          next.projection.max_range_m = parameter.as_double();
        } else if (name == "range.offset_m") {
          next.projection.range_offset_m = parameter.as_double();
        } else if (name == "points.pixel_stride") {
          next.projection.pixel_stride = static_cast<int>(parameter.as_int());
        } else if (name == "cluster.min_points") {
          next.cluster.min_points = static_cast<int>(parameter.as_int());
        } else if (name == "cluster.neighbor_distance_m") {
          next.cluster.neighbor_distance_m = parameter.as_double();
        } else if (name == "cluster.radius_margin_m") {
          next.cluster.radius_margin_m = parameter.as_double();
        } else if (name == "cluster.min_radius_m") {
          next.cluster.min_radius_m = parameter.as_double();
        } else if (name == "ground.roi_width_ratio") {
          next.ground.roi_width_ratio = parameter.as_double();
        } else if (name == "ground.roi_height_ratio") {
          next.ground.roi_height_ratio = parameter.as_double();
        } else if (name == "ground.roi_bottom_offset_ratio") {
          next.ground.roi_bottom_offset_ratio = parameter.as_double();
        } else if (name == "ground.pixel_stride") {
          next.ground.pixel_stride = static_cast<int>(parameter.as_int());
        } else if (name == "ground.max_samples") {
          next.ground.max_samples = static_cast<int>(parameter.as_int());
        } else if (name == "ground.max_iterations") {
          next.ground.max_iterations = static_cast<int>(parameter.as_int());
        } else if (name == "ground.min_depth_m") {
          next.ground.min_depth_m = parameter.as_double();
        } else if (name == "ground.max_depth_m") {
          next.ground.max_depth_m = parameter.as_double();
        } else if (name == "ground.inlier_distance_m") {
          next.ground.inlier_distance_m = parameter.as_double();
        } else if (name == "ground.min_inlier_points") {
          next.ground.min_inlier_points = static_cast<int>(parameter.as_int());
        } else if (name == "ground.min_inlier_ratio") {
          next.ground.min_inlier_ratio = parameter.as_double();
        } else if (name == "ground.min_spread_m") {
          next.ground.min_spread_m = parameter.as_double();
        } else if (name == "ground.max_rmse_m") {
          next.ground.max_rmse_m = parameter.as_double();
        } else if (name == "ground.reference_up_x") {
          next.ground.reference_up_x = parameter.as_double();
        } else if (name == "ground.reference_up_y") {
          next.ground.reference_up_y = parameter.as_double();
        } else if (name == "ground.reference_up_z") {
          next.ground.reference_up_z = parameter.as_double();
        } else if (name == "ground.max_tilt_deg") {
          next.ground.max_tilt_deg = parameter.as_double();
        } else if (name == "ground.min_camera_height_m") {
          next.ground.min_camera_height_m = parameter.as_double();
        } else if (name == "ground.max_camera_height_m") {
          next.ground.max_camera_height_m = parameter.as_double();
        } else if (name == "ground.min_height_m") {
          next.ground.min_height_m = parameter.as_double();
        } else if (name == "ground.max_height_m") {
          next.ground.max_height_m = parameter.as_double();
        } else if (name == "ground.noise_scale") {
          next.ground.noise_scale = parameter.as_double();
        } else if (name == "ground.release_ratio") {
          next.ground.release_ratio = parameter.as_double();
        } else if (name == "ground.reset_history_angle_deg") {
          next.ground.reset_history_angle_deg = parameter.as_double();
        } else if (name == "ground.reset_history_height_m") {
          next.ground.reset_history_height_m = parameter.as_double();
        } else if (name == "stabilization.enabled") {
          next.stabilization.enabled = parameter.as_bool();
        } else if (name == "stabilization.confirm_hits") {
          next.stabilization.confirm_hits = static_cast<int>(parameter.as_int());
        } else if (name == "stabilization.window_frames") {
          next.stabilization.window_frames = static_cast<int>(parameter.as_int());
        } else if (name == "stabilization.hold_sec") {
          next.stabilization.hold_sec = parameter.as_double();
        } else if (name == "stabilization.match_distance_m") {
          next.stabilization.match_distance_m = parameter.as_double();
        } else if (name == "stabilization.max_frame_gap_sec") {
          next.stabilization.max_frame_gap_sec = parameter.as_double();
        } else if (name == "nv12.enabled") {
          next.nv12_enabled = parameter.as_bool();
        } else if (name == "nv12.fps") {
          next.nv12_fps = parameter.as_double();
        } else if (name == "nv12.width") {
          next.nv12_width = static_cast<int>(parameter.as_int());
        } else if (name == "nv12.height") {
          next.nv12_height = static_cast<int>(parameter.as_int());
        } else if (name == "preview.enabled") {
          next.preview_enabled = parameter.as_bool();
        } else if (name == "preview.gui") {
          next.preview_gui = parameter.as_bool();
        } else if (name == "preview.fps") {
          next.preview_fps = parameter.as_double();
        } else if (name == "preview.size_px") {
          next.preview_size_px = static_cast<int>(parameter.as_int());
        } else if (name == "preview.scale") {
          next.preview_scale = static_cast<int>(parameter.as_int());
        } else if (name == "stereo_preview.enabled") {
          next.stereo_preview_enabled = parameter.as_bool();
        } else if (name == "stereo_preview.gui") {
          next.stereo_preview_gui = parameter.as_bool();
        } else if (name == "stereo_preview.fps") {
          next.stereo_preview_fps = parameter.as_double();
        } else if (name == "bev.x_min_m") {
          next.bev_x_min_m = parameter.as_double();
        } else if (name == "bev.x_max_m") {
          next.bev_x_max_m = parameter.as_double();
        } else if (name == "bev.y_min_m") {
          next.bev_y_min_m = parameter.as_double();
        } else if (name == "bev.y_max_m") {
          next.bev_y_max_m = parameter.as_double();
        } else if (name == "bev.meter_per_pixel") {
          next.bev_meter_per_pixel = parameter.as_double();
        } else if (name == "sensor.x_m") {
          next.sensor_x_m = parameter.as_double();
        } else if (name == "sensor.y_m") {
          next.sensor_y_m = parameter.as_double();
        } else if (name == "sensor.yaw_deg") {
          next.sensor_yaw_deg = parameter.as_double();
        } else if (name == "metrics.print_interval_sec") {
          next.metrics_interval_sec = parameter.as_double();
        } else if (name == "frame_id") {
          next.frame_id = parameter.as_string();
        }
      }
    } catch (const rclcpp::ParameterTypeException & error) {
      result.reason = error.what();
      return result;
    }

    if (!validateNodeConfig(next, result.reason)) {
      return result;
    }

    const bool restart_camera = cameraConfigChanged(previous, next);
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config_ = next;
    }
    if (restart_camera) {
      restart_requested_.store(true);
      RCLCPP_INFO(get_logger(), "Camera parameter changed; restarting the DepthAI pipeline");
    }
    result.successful = true;
    result.reason.clear();
    return result;
  }

  NodeConfig configSnapshot() const
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
  }

  void configurePipeline(dai::Pipeline & pipeline,
    const NodeConfig & config,
    std::shared_ptr<dai::node::Camera> & left,
    std::shared_ptr<dai::node::Camera> & right,
    std::shared_ptr<dai::node::StereoDepth> & stereo,
    dai::Node::Output *& nv12_output)
  {
    const auto resolution = parseResolution(config.camera_resolution);
    const float camera_fps = static_cast<float>(config.camera_fps);
    left = pipeline.create<dai::node::Camera>();
    right = pipeline.create<dai::node::Camera>();
    left->build(dai::CameraBoardSocket::CAM_B, resolution, camera_fps);
    right->build(dai::CameraBoardSocket::CAM_C, resolution, camera_fps);
    auto * left_output = left->requestOutput(resolution);
    auto * right_output = right->requestOutput(resolution);
    stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->build(*left_output, *right_output, parseDepthMode(config.depth_mode));
    stereo->initialConfig->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
    stereo->initialConfig->setConfidenceThreshold(config.confidence_threshold);
    stereo->initialConfig->setMedianFilter(parseMedianFilter(config.median_filter));
    stereo->setLeftRightCheck(config.left_right_check);
    stereo->setSubpixel(config.subpixel);
    stereo->setExtendedDisparity(config.extended_disparity);
    // Keep both ground/detection ROIs in the right rectified preview perspective.
    stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT);

    nv12_output = nullptr;
    if (config.nv12_enabled) {
      auto color = pipeline.create<dai::node::Camera>();
      color->build(dai::CameraBoardSocket::CAM_A,
        std::make_pair(kColorSensorWidth, kColorSensorHeight),
        static_cast<float>(config.nv12_fps));
      nv12_output =
        color->requestOutput(std::make_pair(static_cast<std::uint32_t>(config.nv12_width),
                               static_cast<std::uint32_t>(config.nv12_height)),
          dai::ImgFrame::Type::NV12,
          dai::ImgResizeMode::CROP,
          static_cast<float>(config.nv12_fps));
    }
  }

  void cameraLoop()
  {
    bool radar_window_open = false;
    bool stereo_window_open = false;
    GroundPlane previous_ground;
    ClusterStabilizer stabilizer;
    std::vector<std::uint8_t> foreground_mask;
    std::string active_stabilization_context;
    bool have_detection_time = false;
    double previous_detection_time = 0.0;
    const auto clear_history = [&]() {
      stabilizer.clear();
      foreground_mask.clear();
      previous_ground = GroundPlane{};
      active_stabilization_context.clear();
      have_detection_time = false;
    };
    std::string ground_status;
    std::size_t published_obstacle_count = 0U;
    // Idle expiry redraws use the last measured averages, not fabricated zeros.
    double measured_depth_rx_fps = 0.0;
    double measured_nv12_rx_fps = 0.0;
    double measured_object_processing_fps = 0.0;
    double measured_object_processing_ms = 0.0;
    const auto set_ground_status = [&](const std::string & status) {
      if (ground_status == status) { return; }
      ground_status = status;
      std_msgs::msg::String message;
      message.data = status;
      ground_status_publisher_->publish(message);
      std_msgs::msg::Bool validity;
      validity.data = status.rfind("VALID |", 0) == 0;
      ground_valid_publisher_->publish(validity);
    };
    const auto publish_status_result = [&](const NodeConfig & config, const DetectionResult & detection,
        bool waiting_for_depth) {
      const auto stamp = now();
      obstacles_publisher_->publish(obstacleMessage(detection, stamp, config.frame_id));
      published_obstacle_count = detection.obstacles.size();
      if (config.preview_enabled) {
        auto preview = makeScanPreview(detection, ground_status, config,
          measured_depth_rx_fps, measured_nv12_rx_fps,
          measured_object_processing_fps, measured_object_processing_ms, waiting_for_depth);
        preview_publisher_->publish(matToImageMessage(preview, stamp, config.frame_id));
        if (config.preview_gui) {
          cv::imshow("depth_lidar radar preview", preview);
          radar_window_open = true;
        }
      }
    };
    const auto publish_empty = [&](const NodeConfig & config) {
      publish_status_result(config, DetectionResult{}, false);
    };
    set_ground_status("WAITING FOR CAMERA | LIVE MSAC");
    while (rclcpp::ok() && !stop_requested_.load()) {
      restart_requested_.store(false);
      const NodeConfig startup_config = configSnapshot();
      try {
        clear_history();
        measured_depth_rx_fps = 0.0;
        measured_nv12_rx_fps = 0.0;
        measured_object_processing_fps = 0.0;
        measured_object_processing_ms = 0.0;
        set_ground_status("WAITING FOR CAMERA | HISTORY CLEARED");
        publish_empty(startup_config);
        auto device = std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
        dai::Pipeline pipeline(device);
        pipeline.setXLinkChunkSize(0);
        std::shared_ptr<dai::node::Camera> left;
        std::shared_ptr<dai::node::Camera> right;
        std::shared_ptr<dai::node::StereoDepth> stereo;
        dai::Node::Output * nv12_output = nullptr;
        configurePipeline(pipeline, startup_config, left, right, stereo, nv12_output);

        auto depth_queue = stereo->depth.createOutputQueue(1, false);
        std::shared_ptr<dai::MessageQueue> stereo_left_queue;
        std::shared_ptr<dai::MessageQueue> stereo_right_queue;
        if (startup_config.stereo_preview_enabled) {
          stereo_left_queue = stereo->rectifiedLeft.createOutputQueue(1, false);
          stereo_right_queue = stereo->rectifiedRight.createOutputQueue(1, false);
        }
        std::shared_ptr<dai::MessageQueue> nv12_queue;
        if (nv12_output != nullptr) {
          nv12_queue = nv12_output->createOutputQueue(1, false);
        }
        pipeline.build();
        const auto depth_bridge = stereo->depth.getXLinkBridge();
        if (!depth_bridge || !depth_bridge->xLinkOut) {
          throw std::runtime_error("DepthAI did not create the depth XLink bridge");
        }
        depth_bridge->xLinkOut->input.setMaxSize(1);
        depth_bridge->xLinkOut->input.setBlocking(false);
        if (nv12_output != nullptr) {
          const auto bridge = nv12_output->getXLinkBridge();
          if (!bridge || !bridge->xLinkOut) {
            throw std::runtime_error("DepthAI did not create the NV12 XLink bridge");
          }
          bridge->xLinkOut->input.setMaxSize(1);
          bridge->xLinkOut->input.setBlocking(false);
        }
        if (startup_config.stereo_preview_enabled) {
          for (auto * output : {&stereo->rectifiedLeft, &stereo->rectifiedRight}) {
            const auto bridge = output->getXLinkBridge();
            if (!bridge || !bridge->xLinkOut) {
              throw std::runtime_error("DepthAI did not create a rectified image XLink bridge");
            }
            bridge->xLinkOut->input.setMaxSize(1);
            bridge->xLinkOut->input.setBlocking(false);
          }
        }
        pipeline.start();
        RCLCPP_INFO(get_logger(),
          "DepthAI started: depth=%s @ %.1f FPS, mode=%s, NV12=%s",
          startup_config.camera_resolution.c_str(),
          startup_config.camera_fps,
          startup_config.depth_mode.c_str(),
          startup_config.nv12_enabled ? "enabled" : "disabled");

        CameraGeometry camera;
        std::atomic_bool nv12_receiver_stop{false};
        std::atomic_bool nv12_receiver_failed{false};
        std::atomic<std::uint64_t> nv12_received_total{0U};
        std::thread nv12_receiver;
        if (nv12_queue) {
          nv12_receiver = std::thread(
            [this, nv12_queue, &nv12_receiver_stop, &nv12_receiver_failed, &nv12_received_total]() {
              try {
                while (rclcpp::ok() && !stop_requested_.load() && !restart_requested_.load()
                       && !nv12_receiver_stop.load())
                {
                  auto frame = nv12_queue->tryGet<dai::ImgFrame>();
                  if (!frame) {
                    std::this_thread::sleep_for(250us);
                    continue;
                  }
                  if (frame->getType() != dai::ImgFrame::Type::NV12 || frame->getData().empty()) {
                    throw std::runtime_error("camera returned an invalid NV12 host frame");
                  }
                  nv12_received_total.fetch_add(1U, std::memory_order_relaxed);
                }
              } catch (const std::exception & error) {
                RCLCPP_ERROR(get_logger(), "NV12 receiver error: %s", error.what());
                nv12_receiver_failed.store(true);
              }
            });
        }

        bool intrinsics_logged = false;
        auto metrics_start = std::chrono::steady_clock::now();
        auto preview_last = metrics_start - 1s;
        auto stereo_preview_last = metrics_start - 1s;
        std::shared_ptr<dai::ImgFrame> stereo_left_frame;
        std::shared_ptr<dai::ImgFrame> stereo_right_frame;
        int latest_depth_width = 0;
        int latest_depth_height = 0;
        std::size_t metric_frames = 0;
        double metric_delay_sum_ms = 0.0;
        double metric_object_processing_sum_ms = 0.0;
        std::uint64_t previous_nv12_total = 0U;
        std::size_t last_foreground_points = 0;
        std::size_t last_obstacle_count = 0;
        double measured_delay_ms = 0.0;
        auto last_valid_depth_rx = std::chrono::steady_clock::now();
        bool depth_stale = false;

        std::exception_ptr processing_error;
        try {
          while (rclcpp::ok() && !stop_requested_.load() && !restart_requested_.load()) {
            if (nv12_receiver_failed.load()) {
              throw std::runtime_error("NV12 receiver stopped unexpectedly");
            }
            const NodeConfig display_config = configSnapshot();
            if (radar_window_open && (!display_config.preview_enabled || !display_config.preview_gui)) {
              cv::destroyWindow("depth_lidar radar preview");
              radar_window_open = false;
            }
            if (stereo_window_open
                && (!display_config.stereo_preview_enabled || !display_config.stereo_preview_gui))
            {
              cv::destroyWindow("depth_lidar stereo ROI");
              stereo_window_open = false;
            }
            if (radar_window_open || stereo_window_open) {
              const int key = cv::waitKey(1) & 0xff;
              if (key == 'c' || key == 'C') {
                // Use the ROS parameter path so GUI and command-line state agree.
                const auto result = set_parameters_atomically({rclcpp::Parameter(
                  "stereo_preview.enabled", !display_config.stereo_preview_enabled)});
                if (!result.successful) {
                  RCLCPP_WARN(get_logger(), "Could not toggle stereo preview: %s", result.reason.c_str());
                }
                continue;
              }
            }
            if (stereo_left_queue && stereo_right_queue && display_config.stereo_preview_enabled) {
              if (auto frame = stereo_left_queue->tryGet<dai::ImgFrame>()) {
                stereo_left_frame = std::move(frame);
              }
              if (auto frame = stereo_right_queue->tryGet<dai::ImgFrame>()) {
                stereo_right_frame = std::move(frame);
              }
              // Never block depth processing while waiting for the matching eye.
              if (stereo_left_frame && stereo_right_frame) {
                const auto left_sequence = stereo_left_frame->getSequenceNum();
                const auto right_sequence = stereo_right_frame->getSequenceNum();
                if (left_sequence < right_sequence) {
                  stereo_left_frame.reset();
                } else if (right_sequence < left_sequence) {
                  stereo_right_frame.reset();
                } else if (latest_depth_width > 0 && latest_depth_height > 0) {
                  const auto display_now = std::chrono::steady_clock::now();
                  if (display_now - stereo_preview_last
                      >= std::chrono::duration<double>(1.0 / display_config.stereo_preview_fps))
                  {
                    const auto & roi_config = display_config.projection;
                    const auto roi = computeRoi(latest_depth_width, latest_depth_height,
                      roi_config.roi_width_ratio, roi_config.roi_height_ratio,
                      roi_config.roi_bottom_offset_ratio);
                    const auto ground_roi = computeRoi(latest_depth_width, latest_depth_height,
                      display_config.ground.roi_width_ratio, display_config.ground.roi_height_ratio,
                      display_config.ground.roi_bottom_offset_ratio);
                    cv::Mat preview = makeStereoPreview(stereoGrayFrame(*stereo_left_frame),
                      stereoGrayFrame(*stereo_right_frame), roi, latest_depth_width, latest_depth_height, &ground_roi);
                    stereo_preview_publisher_->publish(
                      matToImageMessage(preview, now(), display_config.frame_id));
                    if (display_config.stereo_preview_gui) {
                      cv::imshow("depth_lidar stereo ROI", preview);
                      stereo_window_open = true;
                    }
                    stereo_preview_last = display_now;
                  }
                  stereo_left_frame.reset();
                  stereo_right_frame.reset();
                }
              }
            }
            if (!depth_stale && std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_valid_depth_rx).count()
                > display_config.stabilization.max_frame_gap_sec) {
              clear_history();
              depth_stale = true;
              set_ground_status("DEPTH STALE | WAITING FOR FRAME");
              publish_empty(display_config);
            }
            auto depth_frame = depth_queue->tryGet<dai::ImgFrame>();
            if (!depth_frame) {
              if (published_obstacle_count > 0U && display_config.stabilization.enabled
                  && display_config.stabilization.hold_sec > 0.0) {
                const double clock_sec = std::chrono::duration<double>(
                  std::chrono::steady_clock::now().time_since_epoch()).count();
                const auto remaining = stabilizer.snapshot(clock_sec, display_config.stabilization);
                if (remaining.obstacles.size() < published_obstacle_count) {
                  publish_status_result(display_config, remaining, true);
                }
              }
              std::this_thread::sleep_for(1ms);
              continue;
            }

            const auto processing_start = std::chrono::steady_clock::now();
            // Track TTL begins when a frame is available to the host. Both update
            // and idle snapshot use this clock; capture timestamps below still
            // reject stale/replayed frames. USB latency must not consume hold_sec.
            const double observation_time_sec = std::chrono::duration<double>(
              processing_start.time_since_epoch()).count();
            const int depth_width = depth_frame->getWidth();
            const int depth_height = depth_frame->getHeight();
            const auto & depth_data = depth_frame->getData();
            const std::size_t packed_stride = static_cast<std::size_t>(std::max(0, depth_width)) * sizeof(std::uint16_t);
            const std::size_t depth_stride = depth_frame->getStride() == 0U ? packed_stride : depth_frame->getStride();
            if (depth_width <= 0 || depth_height <= 0 || depth_stride < packed_stride
                || depth_stride % sizeof(std::uint16_t) != 0U
                || depth_frame->getType() != dai::ImgFrame::Type::RAW16
                || depth_data.size() < depth_stride * static_cast<std::size_t>(depth_height - 1) + packed_stride)
            {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid RAW16 depth frame/stride");
              continue;
            }
            const double frame_time_sec = std::chrono::duration<double>(
              depth_frame->getTimestamp().time_since_epoch()).count();
            if (std::chrono::duration<double>(std::chrono::steady_clock::now()
                - depth_frame->getTimestamp()).count() > display_config.stabilization.max_frame_gap_sec) {
              continue;
            }
            last_valid_depth_rx = std::chrono::steady_clock::now();
            depth_stale = false;
            latest_depth_width = camera.width = depth_width;
            latest_depth_height = camera.height = depth_height;
            const auto & transformation = depth_frame->getTransformation();
            if (!transformation.isValid()) {
              throw std::runtime_error("depth frame has no valid rectified intrinsics");
            }
            const auto intrinsics = transformation.getIntrinsicMatrix();
            camera.fx = intrinsics[0][0]; camera.fy = intrinsics[1][1];
            camera.cx = intrinsics[0][2]; camera.cy = intrinsics[1][2];
            if (!intrinsics_logged) {
              RCLCPP_INFO(get_logger(), "Depth %dx%d, fx=%.2f fy=%.2f cx=%.2f cy=%.2f; per-frame MSAC ground",
                depth_width, depth_height, camera.fx, camera.fy, camera.cx, camera.cy);
              intrinsics_logged = true;
            }
            const NodeConfig current = configSnapshot();
            const auto * depth = reinterpret_cast<const std::uint16_t *>(depth_data.data());
            const auto stride_elements = depth_stride / sizeof(std::uint16_t);
            std::ostringstream context;
            context << std::setprecision(17) << camera.width << '|' << camera.height
              << '|' << camera.fx << '|' << camera.fy << '|' << camera.cx << '|' << camera.cy;
            // Configuration changes clear transient labels/tracks. Plane estimation
            // always uses this frame alone, including after a failed estimate.
            context << '|' << current.frame_id << '|' << current.projection.roi_width_ratio
              << '|' << current.projection.roi_height_ratio << '|' << current.projection.roi_bottom_offset_ratio
              << '|' << current.projection.pixel_stride << '|' << current.projection.min_range_m
              << '|' << current.projection.max_range_m << '|' << current.projection.range_offset_m
              << '|' << current.cluster.min_points << '|' << current.cluster.neighbor_distance_m
              << '|' << current.cluster.radius_margin_m << '|' << current.cluster.min_radius_m
              << '|' << current.stabilization.enabled << '|' << current.stabilization.confirm_hits
              << '|' << current.stabilization.window_frames << '|' << current.stabilization.hold_sec
              << '|' << current.stabilization.match_distance_m << '|' << current.stabilization.max_frame_gap_sec;
            context
              << '|' << current.ground.roi_width_ratio << '|' << current.ground.roi_height_ratio << '|' << current.ground.roi_bottom_offset_ratio
              << '|' << current.ground.pixel_stride << '|' << current.ground.max_samples << '|' << current.ground.max_iterations
              << '|' << current.ground.min_depth_m << '|' << current.ground.max_depth_m << '|' << current.ground.inlier_distance_m
              << '|' << current.ground.min_inlier_points << '|' << current.ground.min_inlier_ratio << '|' << current.ground.min_spread_m
              << '|' << current.ground.max_rmse_m << '|' << current.ground.reference_up_x << '|' << current.ground.reference_up_y
              << '|' << current.ground.reference_up_z << '|' << current.ground.max_tilt_deg << '|' << current.ground.min_camera_height_m
              << '|' << current.ground.max_camera_height_m << '|' << current.ground.min_height_m << '|' << current.ground.max_height_m
              << '|' << current.ground.noise_scale << '|' << current.ground.release_ratio << '|' << current.ground.reset_history_angle_deg
              << '|' << current.ground.reset_history_height_m;
            if (active_stabilization_context != context.str()
                || (have_detection_time && frame_time_sec - previous_detection_time
                    > current.stabilization.max_frame_gap_sec)) {
              clear_history();
              active_stabilization_context = context.str();
            }
            if (have_detection_time && frame_time_sec <= previous_detection_time) {
              // Replayed frames must not re-confirm a track or preserve pixel labels.
              clear_history();
              set_ground_status("INVALID | NON-MONOTONIC FRAME TIME");
              publish_empty(current);
              continue;
            }
            previous_detection_time = frame_time_sec;
            have_detection_time = true;
            const auto ground = estimateGroundPlane(depth, stride_elements, camera, current.ground);
            if (ground.valid) {
              if (previous_ground.valid && ground.changedFrom(previous_ground, current.ground)) {
                // Height labels depend on the plane, but tracks use camera XY.
                // Reclassify pixels without forcing unchanged objects to reconfirm.
                foreground_mask.clear();
              }
              previous_ground = ground;
              std::ostringstream status;
              status << "VALID | MSAC " << ground.inlier_points << "/" << ground.sample_points
                << std::fixed << std::setprecision(3) << " | H " << ground.offset_m
                << "m | RMSE " << ground.rmse_m << "m";
              set_ground_status(status.str());
            } else {
              // A failed fit is a missed observation, not a coordinate change.
              // Never reuse its plane or pixel labels. Keep confirmed tracks only
              // within hold_sec, without refreshing their last observation time.
              foreground_mask.clear();
              previous_ground = GroundPlane{};
              set_ground_status("INVALID | " + ground.reason);
            }
            const auto raw_detection = detectForeground(depth, stride_elements, camera, ground,
              current.ground, current.projection, current.cluster,
              current.stabilization.enabled ? &foreground_mask : nullptr);
            // Invalid ground produces an empty raw detection; update counts a miss
            // and expires tracks normally while ground_valid remains false.
            const auto detection = stabilizer.update(raw_detection, observation_time_sec, current.stabilization);
            const auto object_processing_end = std::chrono::steady_clock::now();
            const double object_processing_ms =
              std::chrono::duration<double, std::milli>(object_processing_end - processing_start)
                .count();
            const builtin_interfaces::msg::Time ros_stamp = now();

            obstacles_publisher_->publish(obstacleMessage(detection, ros_stamp, current.frame_id));
            published_obstacle_count = detection.obstacles.size();

            const auto before_preview = std::chrono::steady_clock::now();
            const double delay_ms = std::max(0.0,
              std::chrono::duration<double, std::milli>(
                before_preview - depth_frame->getTimestamp())
                .count());
            ++metric_frames;
            metric_delay_sum_ms += delay_ms;
            metric_object_processing_sum_ms += object_processing_ms;
            last_foreground_points = detection.points.size();
            last_obstacle_count = detection.obstacles.size();

            const double metric_elapsed_sec =
              std::chrono::duration<double>(before_preview - metrics_start).count();
            if (metric_elapsed_sec >= current.metrics_interval_sec) {
              measured_depth_rx_fps = static_cast<double>(metric_frames) / metric_elapsed_sec;
              const std::uint64_t nv12_total = nv12_received_total.load(std::memory_order_relaxed);
              measured_nv12_rx_fps =
                startup_config.nv12_enabled
                  ? static_cast<double>(nv12_total - previous_nv12_total) / metric_elapsed_sec
                  : 0.0;
              previous_nv12_total = nv12_total;
              measured_delay_ms = metric_delay_sum_ms / static_cast<double>(metric_frames);
              measured_object_processing_ms =
                metric_object_processing_sum_ms / static_cast<double>(metric_frames);
              measured_object_processing_fps =
                measured_object_processing_ms > 0.0 ? 1000.0 / measured_object_processing_ms : 0.0;
              const double fps_achievement_percent =
                measured_depth_rx_fps / startup_config.camera_fps * 100.0;
              RCLCPP_INFO(get_logger(),
                "depth RX %.1f FPS (%.1f%%) | NV12 RX %.1f/%.1f FPS | "
                "object processing %.3f ms / %.1f FPS | objects %zu | delay "
                "%.2f ms | "
                "foreground points %zu | ROI %dx%d | %s",
                measured_depth_rx_fps,
                fps_achievement_percent,
                measured_nv12_rx_fps,
                startup_config.nv12_enabled ? startup_config.nv12_fps : 0.0,
                measured_object_processing_ms,
                measured_object_processing_fps,
                last_obstacle_count,
                measured_delay_ms,
                last_foreground_points,
                detection.roi.width,
                detection.roi.height,
                ground_status.c_str());
              metrics_start = before_preview;
              metric_frames = 0;
              metric_delay_sum_ms = 0.0;
              metric_object_processing_sum_ms = 0.0;
            }

            const bool preview_due = (before_preview - preview_last)
                                     >= std::chrono::duration<double>(1.0 / current.preview_fps);
            if (current.preview_enabled && preview_due) {
              cv::Mat preview = makeScanPreview(detection,
                ground_status,
                current,
                measured_depth_rx_fps,
                measured_nv12_rx_fps,
                measured_object_processing_fps,
                measured_object_processing_ms);
              preview_publisher_->publish(matToImageMessage(preview, ros_stamp, current.frame_id));
              if (current.preview_gui) {
                cv::imshow("depth_lidar radar preview", preview);
                radar_window_open = true;
              }
              preview_last = before_preview;
            }
          }
        } catch (...) {
          processing_error = std::current_exception();
        }

        nv12_receiver_stop.store(true);
        if (nv12_receiver.joinable()) {
          nv12_receiver.join();
        }
        pipeline.stop();
        if (stereo_window_open) {
          cv::destroyWindow("depth_lidar stereo ROI");
          stereo_window_open = false;
        }
        if (processing_error) {
          std::rethrow_exception(processing_error);
        }
      } catch (const std::exception & error) {
        clear_history();
        set_ground_status("CAMERA UNAVAILABLE | RECONNECTING");
        obstacles_publisher_->publish(obstacleMessage(DetectionResult{}, now(), configSnapshot().frame_id));
        published_obstacle_count = 0U;
        RCLCPP_ERROR(get_logger(), "DepthAI pipeline error: %s", error.what());
        for (int i = 0; i < 10 && rclcpp::ok() && !stop_requested_.load(); ++i) {
          std::this_thread::sleep_for(100ms);
        }
      }
    }
  }

  mutable std::mutex config_mutex_;
  NodeConfig config_;
  std::atomic_bool stop_requested_{false};
  std::atomic_bool restart_requested_{false};
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ground_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ground_valid_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_preview_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

} // namespace depth_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<depth_lidar::DepthLidarNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("depth_lidar"), "Fatal error: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
