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
  bool preview_enabled{true};
  bool preview_gui{false};
  double preview_fps{10.0};
  int preview_size_px{700};
  double metrics_interval_sec{1.0};
  std::string frame_id{"depth_lidar"};
};

bool isOneOf(const std::string & value, const std::vector<std::string> & choices)
{
  return std::find(choices.begin(), choices.end(), value) != choices.end();
}

bool cameraConfigChanged(const NodeConfig & lhs, const NodeConfig & rhs)
{
  return lhs.camera_fps != rhs.camera_fps ||
         lhs.camera_resolution != rhs.camera_resolution ||
         lhs.depth_mode != rhs.depth_mode ||
         lhs.confidence_threshold != rhs.confidence_threshold ||
         lhs.left_right_check != rhs.left_right_check ||
         lhs.subpixel != rhs.subpixel ||
         lhs.extended_disparity != rhs.extended_disparity ||
         lhs.median_filter != rhs.median_filter;
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
  return validateProjectionConfig(config.projection, reason);
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

cv::Point topViewPoint(
  const cv::Point & origin, const double pixels_per_meter,
  const double forward_m, const double left_m)
{
  return cv::Point(
    static_cast<int>(std::lround(origin.x - left_m * pixels_per_meter)),
    static_cast<int>(std::lround(origin.y - forward_m * pixels_per_meter)));
}

cv::Mat makeTopView(
  const ScanProjection & projection, const NodeConfig & config,
  const double measured_fps, const double measured_delay_ms)
{
  const int size = config.preview_size_px;
  cv::Mat image(size, size, CV_8UC3, cv::Scalar(255, 255, 255));
  const cv::Point origin(size / 2, size - 35);
  const double usable_radius = static_cast<double>(size - 70);
  const double pixels_per_meter = usable_radius / config.projection.max_range_m;
  const cv::Scalar grid_color(205, 205, 205);
  const cv::Scalar axis_color(90, 90, 90);

  for (int ring = 1; ring <= 4; ++ring) {
    const double range_m = config.projection.max_range_m * static_cast<double>(ring) / 4.0;
    const int radius = static_cast<int>(std::lround(range_m * pixels_per_meter));
    cv::ellipse(image, origin, cv::Size(radius, radius), 0.0, 180.0, 360.0, grid_color, 1);
    std::ostringstream label;
    label << std::fixed << std::setprecision(1) << range_m << "m";
    cv::putText(
      image, label.str(), cv::Point(origin.x + 5, origin.y - radius + 15),
      cv::FONT_HERSHEY_SIMPLEX, 0.42, axis_color, 1, cv::LINE_AA);
  }

  for (int degrees = -75; degrees <= 75; degrees += 15) {
    const double angle = static_cast<double>(degrees) * kPi / 180.0;
    const cv::Point end = topViewPoint(
      origin, pixels_per_meter,
      config.projection.max_range_m * std::cos(angle),
      config.projection.max_range_m * std::sin(angle));
    cv::line(image, origin, end, grid_color, 1, cv::LINE_AA);
  }
  cv::arrowedLine(
    image, origin, topViewPoint(origin, pixels_per_meter, config.projection.max_range_m, 0.0),
    axis_color, 2, cv::LINE_AA, 0, 0.025);
  cv::putText(
    image, "x: forward", cv::Point(origin.x + 8, 23), cv::FONT_HERSHEY_SIMPLEX,
    0.48, axis_color, 1, cv::LINE_AA);
  cv::putText(
    image, "y: left", cv::Point(12, origin.y - 8), cv::FONT_HERSHEY_SIMPLEX,
    0.48, axis_color, 1, cv::LINE_AA);

  for (std::size_t i = 0; i < projection.ranges.size(); ++i) {
    const float range = projection.ranges[i];
    if (!std::isfinite(range)) {
      continue;
    }
    const double angle = static_cast<double>(projection.angle_min) +
      static_cast<double>(i) * static_cast<double>(projection.angle_increment);
    const cv::Point point = topViewPoint(
      origin, pixels_per_meter,
      static_cast<double>(range) * std::cos(angle),
      static_cast<double>(range) * std::sin(angle));
    if (point.x >= 0 && point.x < image.cols && point.y >= 0 && point.y < image.rows) {
      cv::circle(image, point, 2, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    }
  }
  cv::circle(image, origin, 5, cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_AA);

  std::ostringstream status;
  status << std::fixed << std::setprecision(1) << "FPS " << measured_fps << "  delay " <<
    measured_delay_ms << " ms";
  cv::putText(
    image, status.str(), cv::Point(12, size - 10), cv::FONT_HERSHEY_SIMPLEX,
    0.48, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
  return image;
}

sensor_msgs::msg::Image matToImageMessage(
  const cv::Mat & image, const builtin_interfaces::msg::Time & stamp,
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

}  // namespace

class DepthLidarNode : public rclcpp::Node
{
public:
  DepthLidarNode()
  : Node("depth_lidar")
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
    config_.projection.roi_bottom_offset_ratio = declare_parameter<double>(
      "roi.bottom_offset_ratio", config_.projection.roi_bottom_offset_ratio);
    config_.projection.min_range_m =
      declare_parameter<double>("range.min_m", config_.projection.min_range_m);
    config_.projection.max_range_m =
      declare_parameter<double>("range.max_m", config_.projection.max_range_m);
    config_.projection.scan_bins =
      declare_parameter<int>("scan.bins", config_.projection.scan_bins);
    config_.projection.pixel_stride =
      declare_parameter<int>("scan.pixel_stride", config_.projection.pixel_stride);
    config_.projection.min_points_per_bin = declare_parameter<int>(
      "scan.min_points_per_bin", config_.projection.min_points_per_bin);
    config_.preview_enabled =
      declare_parameter<bool>("preview.enabled", config_.preview_enabled);
    config_.preview_gui = declare_parameter<bool>("preview.gui", config_.preview_gui);
    config_.preview_fps = declare_parameter<double>("preview.fps", config_.preview_fps);
    config_.preview_size_px =
      declare_parameter<int>("preview.size_px", config_.preview_size_px);
    config_.metrics_interval_sec = declare_parameter<double>(
      "metrics.print_interval_sec", config_.metrics_interval_sec);
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
        } else if (name == "scan.bins") {
          next.projection.scan_bins = static_cast<int>(parameter.as_int());
        } else if (name == "scan.pixel_stride") {
          next.projection.pixel_stride = static_cast<int>(parameter.as_int());
        } else if (name == "scan.min_points_per_bin") {
          next.projection.min_points_per_bin = static_cast<int>(parameter.as_int());
        } else if (name == "preview.enabled") {
          next.preview_enabled = parameter.as_bool();
        } else if (name == "preview.gui") {
          next.preview_gui = parameter.as_bool();
        } else if (name == "preview.fps") {
          next.preview_fps = parameter.as_double();
        } else if (name == "preview.size_px") {
          next.preview_size_px = static_cast<int>(parameter.as_int());
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

  void configurePipeline(
    dai::Pipeline & pipeline, const NodeConfig & config,
    std::shared_ptr<dai::node::Camera> & left,
    std::shared_ptr<dai::node::Camera> & right,
    std::shared_ptr<dai::node::StereoDepth> & stereo)
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
    stereo->setDepthAlign(
      dai::StereoDepthConfig::AlgorithmControl::DepthAlign::CENTER);
  }

  void cameraLoop()
  {
    while (rclcpp::ok() && !stop_requested_.load()) {
      restart_requested_.store(false);
      const NodeConfig startup_config = configSnapshot();
      try {
        auto device = std::make_shared<dai::Device>();
        dai::Pipeline pipeline(device);
        std::shared_ptr<dai::node::Camera> left;
        std::shared_ptr<dai::node::Camera> right;
        std::shared_ptr<dai::node::StereoDepth> stereo;
        configurePipeline(pipeline, startup_config, left, right, stereo);

        auto depth_queue = stereo->depth.createOutputQueue(1, false);
        pipeline.start();
        RCLCPP_INFO(
          get_logger(), "DepthAI started: %s @ %.1f FPS, mode=%s",
          startup_config.camera_resolution.c_str(), startup_config.camera_fps,
          startup_config.depth_mode.c_str());

        bool intrinsics_ready = false;
        double fx = 0.0;
        double cx = 0.0;
        auto metrics_start = std::chrono::steady_clock::now();
        auto preview_last = metrics_start - 1s;
        std::size_t metric_frames = 0;
        double metric_delay_sum_ms = 0.0;
        double metric_processing_sum_ms = 0.0;
        std::size_t last_valid_bins = 0;
        double measured_fps = 0.0;
        double measured_delay_ms = 0.0;

        while (rclcpp::ok() && !stop_requested_.load() && !restart_requested_.load()) {
          auto depth_frame = depth_queue->tryGet<dai::ImgFrame>();
          if (!depth_frame) {
            std::this_thread::sleep_for(1ms);
            continue;
          }

          const auto processing_start = std::chrono::steady_clock::now();
          const int depth_width = depth_frame->getWidth();
          const int depth_height = depth_frame->getHeight();
          const auto & depth_data = depth_frame->getData();
          const std::size_t expected_depth_bytes =
            static_cast<std::size_t>(depth_width) * static_cast<std::size_t>(depth_height) *
            sizeof(std::uint16_t);
          if (depth_width <= 0 || depth_height <= 0 || depth_data.size() < expected_depth_bytes) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000, "Expected a non-empty packed 16-bit depth frame");
            continue;
          }

          if (!intrinsics_ready) {
            const auto intrinsics = device->readCalibration().getCameraIntrinsics(
              dai::CameraBoardSocket::CAM_C, depth_width, depth_height);
            fx = static_cast<double>(intrinsics.at(0).at(0));
            cx = static_cast<double>(intrinsics.at(0).at(2));
            if (!std::isfinite(fx) || fx <= 0.0 || !std::isfinite(cx)) {
              throw std::runtime_error("invalid right-camera intrinsics from device calibration");
            }
            intrinsics_ready = true;
            RCLCPP_INFO(
              get_logger(), "Depth frame %dx%d, fx=%.2f, cx=%.2f",
              depth_width, depth_height, fx, cx);
          }

          const NodeConfig current = configSnapshot();
          const auto projection = projectDepthToScan(
            reinterpret_cast<const std::uint16_t *>(depth_data.data()), depth_width, depth_height,
            static_cast<std::size_t>(depth_width), fx, cx, current.projection);
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
          const double preliminary_delay_ms = std::max(
            0.0, std::chrono::duration<double, std::milli>(
              before_preview - depth_frame->getTimestamp()).count());
          const bool preview_due =
            (before_preview - preview_last) >=
            std::chrono::duration<double>(1.0 / current.preview_fps);
          if (current.preview_enabled && preview_due) {
            cv::Mat preview = makeTopView(
              projection, current, measured_fps,
              measured_delay_ms > 0.0 ? measured_delay_ms : preliminary_delay_ms);
            preview_publisher_->publish(matToImageMessage(preview, ros_stamp, current.frame_id));
            if (current.preview_gui) {
              cv::imshow("depth_lidar top view", preview);
              cv::waitKey(1);
            }
            preview_last = before_preview;
          }

          const auto processing_end = std::chrono::steady_clock::now();
          const double processing_ms = std::chrono::duration<double, std::milli>(
            processing_end - processing_start).count();
          const double delay_ms = std::max(
            0.0, std::chrono::duration<double, std::milli>(
              processing_end - depth_frame->getTimestamp()).count());
          ++metric_frames;
          metric_delay_sum_ms += delay_ms;
          metric_processing_sum_ms += processing_ms;
          last_valid_bins = projection.valid_bins;

          const double metric_elapsed_sec = std::chrono::duration<double>(
            processing_end - metrics_start).count();
          if (metric_elapsed_sec >= current.metrics_interval_sec) {
            measured_fps = static_cast<double>(metric_frames) / metric_elapsed_sec;
            measured_delay_ms = metric_delay_sum_ms / static_cast<double>(metric_frames);
            const double processing_average_ms =
              metric_processing_sum_ms / static_cast<double>(metric_frames);
            const double fps_achievement_percent =
              measured_fps / startup_config.camera_fps * 100.0;
            RCLCPP_INFO(
              get_logger(),
              "target FPS %.1f | actual FPS %.1f | achievement %.1f%% | delay %.2f ms | "
              "processing %.2f ms | valid bins %zu/%d | ROI %dx%d",
              startup_config.camera_fps, measured_fps, fps_achievement_percent,
              measured_delay_ms, processing_average_ms, last_valid_bins,
              current.projection.scan_bins, projection.roi.width, projection.roi.height);
            metrics_start = processing_end;
            metric_frames = 0;
            metric_delay_sum_ms = 0.0;
            metric_processing_sum_ms = 0.0;
          }
        }
        pipeline.stop();
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

}  // namespace depth_lidar

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
