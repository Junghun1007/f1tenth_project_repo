#include "depth_lidar/depth_lidar_geometry.hpp"

#include <depthai/depthai.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
constexpr double kPi = 3.14159265358979323846;
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
  bool nv12_enabled{true};
  double nv12_fps{60.0};
  int nv12_width{1280};
  int nv12_height{800};
  bool preview_enabled{true};
  bool preview_gui{false};
  double preview_fps{10.0};
  int preview_size_px{700};
  int preview_scale{3};
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
         || lhs.nv12_height != rhs.nv12_height;
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
  if (config.cluster.min_bins > config.projection.scan_bins) {
    reason = "cluster.min_bins cannot exceed scan.bins";
    return false;
  }
  return validateClusterConfig(config.cluster, reason);
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

std::pair<double, double>
cameraPointToVehicle(const NodeConfig & config, const double forward_m, const double left_m)
{
  const double yaw = config.sensor_yaw_deg * kPi / 180.0;
  return {config.sensor_x_m + std::cos(yaw) * forward_m - std::sin(yaw) * left_m,
    config.sensor_y_m + std::sin(yaw) * forward_m + std::cos(yaw) * left_m};
}

std::vector<ObstacleCircle> obstaclesToVehicle(const NodeConfig & config,
  const std::vector<ObstacleCircle> & camera_obstacles)
{
  std::vector<ObstacleCircle> vehicle_obstacles = camera_obstacles;
  for (auto & obstacle : vehicle_obstacles) {
    const auto position = cameraPointToVehicle(config, obstacle.forward_m, obstacle.left_m);
    obstacle.forward_m = position.first;
    obstacle.left_m = position.second;
  }
  return vehicle_obstacles;
}

cv::Point bevPoint(const NodeConfig & config, const double forward_m, const double left_m)
{
  const double pixels_per_meter =
    static_cast<double>(config.preview_scale) / config.bev_meter_per_pixel;
  return cv::Point(
    static_cast<int>(std::lround((config.bev_y_max_m - left_m) * pixels_per_meter - 0.5)),
    static_cast<int>(std::lround((config.bev_x_max_m - forward_m) * pixels_per_meter - 0.5)));
}

cv::Mat makeBevPreview(const ScanProjection & projection,
  const std::vector<ObstacleCircle> & obstacles,
  const NodeConfig & config,
  const double depth_rx_fps,
  const double nv12_rx_fps,
  const double object_processing_fps,
  const double object_processing_ms)
{
  const int width = static_cast<int>(std::lround(
                      (config.bev_y_max_m - config.bev_y_min_m) / config.bev_meter_per_pixel))
                    * config.preview_scale;
  const int height = static_cast<int>(std::lround(
                       (config.bev_x_max_m - config.bev_x_min_m) / config.bev_meter_per_pixel))
                     * config.preview_scale;
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  const cv::Scalar grid_color(48, 48, 48);
  const cv::Scalar center_color(78, 78, 78);
  const double pixels_per_meter =
    static_cast<double>(config.preview_scale) / config.bev_meter_per_pixel;

  for (double x = std::ceil(config.bev_x_min_m * 2.0) / 2.0; x <= config.bev_x_max_m + 1.0e-9;
    x += 0.5)
  {
    const int row = bevPoint(config, x, 0.0).y;
    cv::line(image, cv::Point(0, row), cv::Point(width - 1, row), grid_color, 1, cv::LINE_AA);
  }
  const int center_column = bevPoint(config, config.bev_x_min_m, 0.0).x;
  cv::line(image,
    cv::Point(center_column, 0),
    cv::Point(center_column, height - 1),
    center_color,
    1,
    cv::LINE_AA);

  for (const auto & obstacle : obstacles) {
    const cv::Point center = bevPoint(config, obstacle.forward_m, obstacle.left_m);
    const int radius_px =
      std::max(1, static_cast<int>(std::lround(obstacle.radius_m * pixels_per_meter)));
    cv::circle(image, center, radius_px, cv::Scalar(0, 55, 105), cv::FILLED, cv::LINE_AA);
    cv::circle(image, center, radius_px, cv::Scalar(0, 150, 255), 2, cv::LINE_AA);
    if (center.x >= 0 && center.x < image.cols && center.y >= 0 && center.y < image.rows) {
      std::ostringstream label;
      label << std::fixed << std::setprecision(2) << "x" << obstacle.forward_m << " y"
            << obstacle.left_m << " r" << obstacle.radius_m;
      cv::putText(image,
        label.str(),
        center + cv::Point(5, -5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.30,
        cv::Scalar(220, 220, 220),
        1,
        cv::LINE_AA);
    }
  }

  for (std::size_t i = 0; i < projection.ranges.size(); ++i) {
    const float range = projection.ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    const double angle = static_cast<double>(projection.angle_min)
                         + static_cast<double>(i) * static_cast<double>(projection.angle_increment);
    const auto point_vehicle = cameraPointToVehicle(config,
      static_cast<double>(range) * std::cos(angle),
      static_cast<double>(range) * std::sin(angle));
    const cv::Point point = bevPoint(config, point_vehicle.first, point_vehicle.second);
    if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows) {
      cv::circle(image, point, 2, cv::Scalar(255, 255, 0), cv::FILLED, cv::LINE_AA);
    }
  }
  cv::Point vehicle = bevPoint(config, 0.0, 0.0);
  vehicle.x = std::clamp(vehicle.x, 0, width - 1);
  vehicle.y = std::clamp(vehicle.y, 0, height - 1);
  cv::circle(image, vehicle, 4, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);

  cv::putText(image,
    "DEPTH LIDAR / BEV",
    cv::Point(8, 18),
    cv::FONT_HERSHEY_SIMPLEX,
    0.45,
    cv::Scalar(230, 230, 230),
    1,
    cv::LINE_AA);
  std::ostringstream receive_status;
  receive_status << std::fixed << std::setprecision(1) << "DEPTH RX " << depth_rx_fps
                 << " FPS | NV12 RX " << nv12_rx_fps << " FPS";
  cv::putText(image,
    receive_status.str(),
    cv::Point(8, height - 28),
    cv::FONT_HERSHEY_SIMPLEX,
    0.36,
    cv::Scalar(230, 230, 230),
    1,
    cv::LINE_AA);
  std::ostringstream processing_status;
  processing_status << std::fixed << std::setprecision(1) << "CLUSTER+POS " << object_processing_fps
                    << " FPS (" << std::setprecision(3) << object_processing_ms << " ms AVG) | N "
                    << obstacles.size();
  cv::putText(image,
    processing_status.str(),
    cv::Point(8, height - 10),
    cv::FONT_HERSHEY_SIMPLEX,
    0.36,
    cv::Scalar(230, 230, 230),
    1,
    cv::LINE_AA);
  return image;
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
    config_.projection.scan_bins =
      declare_parameter<int>("scan.bins", config_.projection.scan_bins);
    config_.projection.pixel_stride =
      declare_parameter<int>("scan.pixel_stride", config_.projection.pixel_stride);
    config_.projection.min_points_per_bin =
      declare_parameter<int>("scan.min_points_per_bin", config_.projection.min_points_per_bin);
    config_.cluster.min_bins = declare_parameter<int>("cluster.min_bins", config_.cluster.min_bins);
    config_.cluster.max_missing_bins =
      declare_parameter<int>("cluster.max_missing_bins", config_.cluster.max_missing_bins);
    config_.cluster.base_neighbor_distance_m =
      declare_parameter<double>("cluster.base_neighbor_distance_m",
        config_.cluster.base_neighbor_distance_m);
    config_.cluster.angular_neighbor_scale =
      declare_parameter<double>("cluster.angular_neighbor_scale",
        config_.cluster.angular_neighbor_scale);
    config_.cluster.radius_margin_m =
      declare_parameter<double>("cluster.radius_margin_m", config_.cluster.radius_margin_m);
    config_.cluster.min_radius_m =
      declare_parameter<double>("cluster.min_radius_m", config_.cluster.min_radius_m);
    config_.cluster.max_radius_m =
      declare_parameter<double>("cluster.max_radius_m", config_.cluster.max_radius_m);
    config_.nv12_enabled = declare_parameter<bool>("nv12.enabled", config_.nv12_enabled);
    config_.nv12_fps = declare_parameter<double>("nv12.fps", config_.nv12_fps);
    config_.nv12_width = declare_parameter<int>("nv12.width", config_.nv12_width);
    config_.nv12_height = declare_parameter<int>("nv12.height", config_.nv12_height);
    config_.preview_enabled = declare_parameter<bool>("preview.enabled", config_.preview_enabled);
    config_.preview_gui = declare_parameter<bool>("preview.gui", config_.preview_gui);
    config_.preview_fps = declare_parameter<double>("preview.fps", config_.preview_fps);
    config_.preview_size_px = declare_parameter<int>("preview.size_px", config_.preview_size_px);
    config_.preview_scale = declare_parameter<int>("preview.scale", config_.preview_scale);
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

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("~/scan", qos);
    preview_publisher_ = create_publisher<sensor_msgs::msg::Image>("~/preview", qos);
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
        } else if (name == "scan.bins") {
          next.projection.scan_bins = static_cast<int>(parameter.as_int());
        } else if (name == "scan.pixel_stride") {
          next.projection.pixel_stride = static_cast<int>(parameter.as_int());
        } else if (name == "scan.min_points_per_bin") {
          next.projection.min_points_per_bin = static_cast<int>(parameter.as_int());
        } else if (name == "cluster.min_bins") {
          next.cluster.min_bins = static_cast<int>(parameter.as_int());
        } else if (name == "cluster.max_missing_bins") {
          next.cluster.max_missing_bins = static_cast<int>(parameter.as_int());
        } else if (name == "cluster.base_neighbor_distance_m") {
          next.cluster.base_neighbor_distance_m = parameter.as_double();
        } else if (name == "cluster.angular_neighbor_scale") {
          next.cluster.angular_neighbor_scale = parameter.as_double();
        } else if (name == "cluster.radius_margin_m") {
          next.cluster.radius_margin_m = parameter.as_double();
        } else if (name == "cluster.min_radius_m") {
          next.cluster.min_radius_m = parameter.as_double();
        } else if (name == "cluster.max_radius_m") {
          next.cluster.max_radius_m = parameter.as_double();
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
    stereo->initialConfig->setConfidenceThreshold(config.confidence_threshold);
    stereo->initialConfig->setMedianFilter(parseMedianFilter(config.median_filter));
    stereo->setLeftRightCheck(config.left_right_check);
    stereo->setSubpixel(config.subpixel);
    stereo->setExtendedDisparity(config.extended_disparity);
    stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::CENTER);

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
    while (rclcpp::ok() && !stop_requested_.load()) {
      restart_requested_.store(false);
      const NodeConfig startup_config = configSnapshot();
      try {
        auto device = std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
        dai::Pipeline pipeline(device);
        pipeline.setXLinkChunkSize(0);
        std::shared_ptr<dai::node::Camera> left;
        std::shared_ptr<dai::node::Camera> right;
        std::shared_ptr<dai::node::StereoDepth> stereo;
        dai::Node::Output * nv12_output = nullptr;
        configurePipeline(pipeline, startup_config, left, right, stereo, nv12_output);

        auto depth_queue = stereo->depth.createOutputQueue(1, false);
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
        pipeline.start();
        RCLCPP_INFO(get_logger(),
          "DepthAI started: depth=%s @ %.1f FPS, mode=%s, NV12=%s",
          startup_config.camera_resolution.c_str(),
          startup_config.camera_fps,
          startup_config.depth_mode.c_str(),
          startup_config.nv12_enabled ? "enabled" : "disabled");

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

        bool intrinsics_ready = false;
        double fx = 0.0;
        double cx = 0.0;
        auto metrics_start = std::chrono::steady_clock::now();
        auto preview_last = metrics_start - 1s;
        std::size_t metric_frames = 0;
        double metric_delay_sum_ms = 0.0;
        double metric_object_processing_sum_ms = 0.0;
        std::uint64_t previous_nv12_total = 0U;
        std::size_t last_valid_bins = 0;
        std::size_t last_obstacle_count = 0;
        double measured_depth_rx_fps = 0.0;
        double measured_nv12_rx_fps = 0.0;
        double measured_object_processing_fps = 0.0;
        double measured_object_processing_ms = 0.0;
        double measured_delay_ms = 0.0;

        std::exception_ptr processing_error;
        try {
          while (rclcpp::ok() && !stop_requested_.load() && !restart_requested_.load()) {
            if (nv12_receiver_failed.load()) {
              throw std::runtime_error("NV12 receiver stopped unexpectedly");
            }
            auto depth_frame = depth_queue->tryGet<dai::ImgFrame>();
            if (!depth_frame) {
              std::this_thread::sleep_for(1ms);
              continue;
            }

            const auto processing_start = std::chrono::steady_clock::now();
            const int depth_width = depth_frame->getWidth();
            const int depth_height = depth_frame->getHeight();
            const auto & depth_data = depth_frame->getData();
            const std::size_t expected_depth_bytes = static_cast<std::size_t>(depth_width)
                                                     * static_cast<std::size_t>(depth_height)
                                                     * sizeof(std::uint16_t);
            if (depth_width <= 0 || depth_height <= 0 || depth_data.size() < expected_depth_bytes) {
              RCLCPP_WARN_THROTTLE(get_logger(),
                *get_clock(),
                2000,
                "Expected a non-empty packed 16-bit depth frame");
              continue;
            }

            if (!intrinsics_ready) {
              const auto intrinsics =
                device->readCalibration().getCameraIntrinsics(dai::CameraBoardSocket::CAM_C,
                  depth_width,
                  depth_height);
              fx = static_cast<double>(intrinsics.at(0).at(0));
              cx = static_cast<double>(intrinsics.at(0).at(2));
              if (!std::isfinite(fx) || fx <= 0.0 || !std::isfinite(cx)) {
                throw std::runtime_error("invalid right-camera intrinsics from device calibration");
              }
              intrinsics_ready = true;
              RCLCPP_INFO(get_logger(),
                "Depth frame %dx%d, fx=%.2f, cx=%.2f",
                depth_width,
                depth_height,
                fx,
                cx);
            }

            const NodeConfig current = configSnapshot();
            const auto projection =
              projectDepthToScan(reinterpret_cast<const std::uint16_t *>(depth_data.data()),
                depth_width,
                depth_height,
                static_cast<std::size_t>(depth_width),
                fx,
                cx,
                current.projection);
            const auto camera_obstacles = clusterScan(projection, current.cluster);
            const auto obstacles = obstaclesToVehicle(current, camera_obstacles);
            const auto object_processing_end = std::chrono::steady_clock::now();
            const double object_processing_ms =
              std::chrono::duration<double, std::milli>(object_processing_end - processing_start)
                .count();
            const builtin_interfaces::msg::Time ros_stamp = now();

            sensor_msgs::msg::LaserScan scan;
            scan.header.stamp = ros_stamp;
            scan.header.frame_id = current.frame_id;
            scan.angle_min = projection.angle_min;
            scan.angle_max = projection.angle_max;
            scan.angle_increment = projection.angle_increment;
            scan.time_increment = 0.0F;
            scan.scan_time = static_cast<float>(1.0 / startup_config.camera_fps);
            scan.range_min = static_cast<float>(current.projection.min_range_m);
            scan.range_max = static_cast<float>(current.projection.max_range_m);
            scan.ranges = projection.ranges;
            scan_publisher_->publish(std::move(scan));

            const auto before_preview = std::chrono::steady_clock::now();
            const double delay_ms = std::max(0.0,
              std::chrono::duration<double, std::milli>(
                before_preview - depth_frame->getTimestamp())
                .count());
            ++metric_frames;
            metric_delay_sum_ms += delay_ms;
            metric_object_processing_sum_ms += object_processing_ms;
            last_valid_bins = projection.valid_bins;
            last_obstacle_count = obstacles.size();

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
                "valid bins %zu/%d | ROI %dx%d",
                measured_depth_rx_fps,
                fps_achievement_percent,
                measured_nv12_rx_fps,
                startup_config.nv12_enabled ? startup_config.nv12_fps : 0.0,
                measured_object_processing_ms,
                measured_object_processing_fps,
                last_obstacle_count,
                measured_delay_ms,
                last_valid_bins,
                current.projection.scan_bins,
                projection.roi.width,
                projection.roi.height);
              metrics_start = before_preview;
              metric_frames = 0;
              metric_delay_sum_ms = 0.0;
              metric_object_processing_sum_ms = 0.0;
            }

            const bool preview_due = (before_preview - preview_last)
                                     >= std::chrono::duration<double>(1.0 / current.preview_fps);
            if (current.preview_enabled && preview_due) {
              cv::Mat preview = makeBevPreview(projection,
                obstacles,
                current,
                measured_depth_rx_fps,
                measured_nv12_rx_fps,
                measured_object_processing_fps,
                measured_object_processing_ms);
              preview_publisher_->publish(matToImageMessage(preview, ros_stamp, current.frame_id));
              if (current.preview_gui) {
                cv::imshow("depth_lidar BEV preview", preview);
                cv::waitKey(1);
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
        if (processing_error) {
          std::rethrow_exception(processing_error);
        }
      } catch (const std::exception & error) {
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
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

} // namespace depth_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<depth_lidar::DepthLidarNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("depth_lidar"), "Fatal error: %s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
