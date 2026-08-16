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
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "bev_processor/bev_geometry.hpp"
#include "bev_processor/bev_lane_reconstructor.hpp"
#include "bev_processor/cuda_bev_processor.hpp"
#include "bev_processor/oak_startup_measurement.hpp"
#include "camera_driver/msg/bev_input.hpp"

namespace bev_processor
{

namespace
{

using SteadyClock = std::chrono::steady_clock;

struct BevFrame
{
  cv::Mat image;
  cv::Mat lane_mask;
  cv::Mat left_lane_mask;
  cv::Mat right_lane_mask;
  std::vector<BevLaneReconstruction::SlidingWindow> lane_sliding_windows;
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

std::unique_ptr<sensor_msgs::msg::Image> makeMono8Message(
  const BevFrame & frame,
  const std::string & frame_id)
{
  if (frame.lane_mask.type() != CV_8UC1) {
    throw std::invalid_argument("BEV lane output must be a MONO8 image");
  }

  auto message = std::make_unique<sensor_msgs::msg::Image>();
  message->header = frame.header;
  message->header.frame_id = frame_id;
  message->height = static_cast<std::uint32_t>(frame.lane_mask.rows);
  message->width = static_cast<std::uint32_t>(frame.lane_mask.cols);
  message->encoding = sensor_msgs::image_encodings::MONO8;
  message->is_bigendian = false;
  message->step = static_cast<std::uint32_t>(frame.lane_mask.cols);
  message->data.resize(
    static_cast<std::size_t>(message->step) *
    static_cast<std::size_t>(message->height));

  for (int row = 0; row < frame.lane_mask.rows; ++row) {
    std::memcpy(
      message->data.data() +
      static_cast<std::size_t>(row) * message->step,
      frame.lane_mask.ptr(row),
      message->step);
  }
  return message;
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

cv::Mat makeExpandedBevPreview(
  const cv::Mat & bev_bgr,
  const int lateral_margin_px)
{
  if (bev_bgr.type() != CV_8UC3 || lateral_margin_px < 0) {
    throw std::invalid_argument(
            "expanded BEV preview expects BGR8 and a non-negative margin");
  }
  cv::Mat expanded = cv::Mat::zeros(
    bev_bgr.rows, bev_bgr.cols + 2 * lateral_margin_px, CV_8UC3);
  bev_bgr.copyTo(expanded(cv::Rect(
      lateral_margin_px, 0, bev_bgr.cols, bev_bgr.rows)));
  return expanded;
}

cv::Mat makeLaneOverlayPreview(
  const cv::Mat & expanded_bev_bgr,
  const cv::Mat & left_lane_mask,
  const cv::Mat & right_lane_mask,
  const cv::Mat & centerline_mask,
  const double alpha)
{
  if (
    expanded_bev_bgr.type() != CV_8UC3 ||
    left_lane_mask.type() != CV_8UC1 ||
    right_lane_mask.type() != CV_8UC1 ||
    centerline_mask.type() != CV_8UC1 ||
    expanded_bev_bgr.size() != left_lane_mask.size() ||
    expanded_bev_bgr.size() != right_lane_mask.size() ||
    expanded_bev_bgr.size() != centerline_mask.size())
  {
    throw std::invalid_argument(
            "lane preview overlay expects matching BGR8 and MONO8 images");
  }
  cv::Mat highlighted = expanded_bev_bgr.clone();
  // OpenCV uses BGR: left lane is blue and right lane is red.
  highlighted.setTo(cv::Scalar(255, 0, 0), left_lane_mask);
  highlighted.setTo(cv::Scalar(0, 0, 255), right_lane_mask);
  // The drive centerline is yellow so it remains distinct from the green
  // vehicle Y=0 reference drawn by makeCoordinatePreview().
  highlighted.setTo(cv::Scalar(0, 255, 255), centerline_mask);
  cv::Mat blended;
  cv::addWeighted(
    highlighted, alpha, expanded_bev_bgr, 1.0 - alpha, 0.0, blended);
  return blended;
}

cv::Point laneMetricToPixel(
  const cv::Point2d & metric_point,
  const BevConfig & config,
  const double lateral_margin_m)
{
  return cv::Point(
    static_cast<int>(std::lround(
      (config.y_max_m + lateral_margin_m - metric_point.y) /
      config.meter_per_pixel - 0.5)),
    static_cast<int>(std::lround(
      (config.x_max_m - metric_point.x) / config.meter_per_pixel - 0.5)));
}

cv::Mat makeSlidingWindowPreview(
  const cv::Mat & bev_bgr,
  const std::vector<BevLaneReconstruction::SlidingWindow> & windows,
  const BevConfig & config,
  const double lateral_margin_m)
{
  if (bev_bgr.type() != CV_8UC3) {
    throw std::invalid_argument(
            "sliding-window preview expects a BGR8 BEV image");
  }
  cv::Mat preview = bev_bgr.clone();
  for (const auto & window : windows) {
    std::vector<cv::Point> pixels;
    pixels.reserve(window.corners.size());
    for (const auto & corner : window.corners) {
      pixels.push_back(laneMetricToPixel(corner, config, lateral_margin_m));
    }
    const cv::Scalar color = window.left_lane ?
      (window.measurement_found ?
      cv::Scalar(255, 255, 0) : cv::Scalar(140, 140, 0)) :
      (window.measurement_found ?
      cv::Scalar(255, 0, 255) : cv::Scalar(140, 0, 140));
    cv::polylines(
      preview, std::vector<std::vector<cv::Point>>{pixels}, true,
      color, 1, cv::LINE_AA);
  }
  return preview;
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
    initializeInputCropGeometry();
    if (lane_reconstruction_enabled_) {
      lane_reconstructor_ = std::make_unique<BevLaneReconstructor>(
        lane_reconstructor_config_);
    }

    if (performance_measurement_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "Performance measurement mode enabled: all GUI previews are off; "
        "stabilized-input and BEV-ready pipeline metrics will be reported.");
    }

    if (startup_measurement_config_.manual_camera_height_enabled) {
      RCLCPP_INFO(
        get_logger(),
        "Using manual camera height %.4fm and measuring startup roll/pitch "
        "from the calibrated IMU. Keep the vehicle stationary.",
        startup_measurement_config_.manual_camera_height_m);
      RCLCPP_INFO(
        get_logger(),
        "Measurement quality: warmup=%.1fs, IMU=%d samples, "
        "depth/IR ground-plane measurement=disabled, attitude=imu.",
        startup_measurement_config_.warmup_sec,
        startup_measurement_config_.imu_sample_count);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Measuring startup camera height from the OAK stereo ground plane "
        "and roll/pitch from the configured '%s' source. "
        "Keep the vehicle stationary and the center view on flat ground.",
        startupAttitudeSourceName(startup_measurement_config_.attitude_source));
      RCLCPP_INFO(
        get_logger(),
        "Measurement quality: warmup=%.1fs, IR-dot=%.2f, IMU=%d samples, "
        "stereo=%dx%d@%.1fHz/5-bit-subpixel/shift=%d, "
        "depth ROI=%dx%d step=%d (%d valid points minimum), "
        "RANSAC=%d iterations, stable planes=%d frames, attitude=%s.",
        startup_measurement_config_.warmup_sec,
        startup_measurement_config_.ir_dot_projector_intensity,
        startup_measurement_config_.imu_sample_count,
        startup_measurement_config_.stereo_width,
        startup_measurement_config_.stereo_height,
        startup_measurement_config_.stereo_fps,
        startup_measurement_config_.stereo_disparity_shift,
        startup_measurement_config_.roi_width,
        startup_measurement_config_.roi_height,
        startup_measurement_config_.point_sample_step,
        startup_measurement_config_.minimum_valid_points,
        startup_measurement_config_.plane_ransac_iterations,
        startup_measurement_config_.stable_plane_frame_count,
        startupAttitudeSourceName(startup_measurement_config_.attitude_source));
    }
    const auto measurement =
      measureOakStartupExtrinsics(startup_measurement_config_);
    camera_model_.position_vehicle_m[2] = measurement.height_m;
    startup_roll_deg_ = measurement.roll_deg;
    startup_pitch_down_deg_ = measurement.pitch_down_deg;
    RCLCPP_INFO(
      get_logger(),
      "BEV_STARTUP_MEASUREMENT: height_source=%s, attitude_source=%s, "
      "height=%.4fm, roll=%.3fdeg, "
      "pitch=%.3fdeg, downward_pitch=%.3fdeg, "
      "height_stddev=%.4fm, plane_normal_RMS=%.3fdeg",
      measurement.height_source.c_str(),
      measurement.attitude_source.c_str(),
      measurement.height_m,
      measurement.roll_deg,
      -measurement.pitch_down_deg,
      measurement.pitch_down_deg,
      measurement.height_stddev_m,
      measurement.plane_normal_rms_deg);
    RCLCPP_INFO(
      get_logger(),
      "Startup IMU: measured=(roll=%.3f,pitch_down=%.3fdeg), "
      "corrected=(roll=%.3f,pitch_down=%.3fdeg), "
      "direction_RMS=%.3fdeg, gyro_mean/stddev=%.3f/%.3fdegps",
      measurement.imu_roll_deg,
      measurement.imu_pitch_down_deg,
      measurement.corrected_imu_roll_deg,
      measurement.corrected_imu_pitch_down_deg,
      measurement.imu_direction_rms_deg,
      measurement.imu_gyroscope_mean_degps,
      measurement.imu_gyroscope_stddev_degps);
    if (!startup_measurement_config_.manual_camera_height_enabled) {
      RCLCPP_INFO(
        get_logger(),
        "Startup attitude selection: selected=%s, "
        "IMU corrected=(roll=%.3f,pitch_down=%.3fdeg), "
        "depth=(roll=%.3f,pitch_down=%.3fdeg), difference=%.3fdeg",
        measurement.attitude_source.c_str(),
        measurement.corrected_imu_roll_deg,
        measurement.corrected_imu_pitch_down_deg,
        measurement.depth_roll_deg,
        measurement.depth_pitch_down_deg,
        measurement.plane_imu_difference_deg);
      RCLCPP_INFO(
        get_logger(),
        "Startup ground-plane diagnostics: depth=%.3fm, "
        "points=%zu, inliers=%zu (%.1f%%), residual_MAD=%.4fm",
        measurement.median_depth_m,
        measurement.valid_point_count,
        measurement.plane_inlier_count,
        100.0 * measurement.plane_inlier_ratio,
        measurement.plane_residual_mad_m);
    }
    installProcessor(
      startup_roll_deg_,
      startup_pitch_down_deg_,
      startup_measurement_config_.manual_camera_height_enabled ?
      "IMU attitude + manual camera height" :
      measurement.attitude_source == "depth" ?
      "depth-plane attitude + depth-plane offset height" :
      "IMU attitude + depth-plane offset height");

    auto reference_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    reference_qos.reliable().transient_local();
    // Humble's intra-process path cannot provide transient-local durability
    // to the camera driver that is loaded after this one-shot publication.
    rclcpp::PublisherOptions reference_publisher_options;
    reference_publisher_options.use_intra_process_comm =
      rclcpp::IntraProcessSetting::Disable;
    startup_ground_reference_publisher_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(
      startup_ground_reference_topic_, reference_qos,
      reference_publisher_options);
    publishStartupGroundReference(measurement.attitude_source);

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    input_subscription_ = create_subscription<camera_driver::msg::BevInput>(
      input_topic_,
      image_qos,
      [this](camera_driver::msg::BevInput::ConstSharedPtr message) {
        onBevInput(std::move(message));
      });
    if (publish_enabled_) {
      output_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic_, image_qos);
    }
    if (lane_reconstruction_enabled_) {
      lane_output_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        lane_output_topic_, image_qos);
    }

    processing_thread_ = std::thread(&BevProcessorNode::processingLoop, this);
    if (publish_enabled_ || lane_reconstruction_enabled_) {
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
    RCLCPP_INFO(
      get_logger(),
      "==================== BEV PROCESSOR START ====================");
    RCLCPP_INFO(
      get_logger(),
      "BEV processor started: input=%s "
      "(%dx%d bottom crop of %dx%d, top=%dpx, expected=%.1fHz), "
      "output=%s (%dx%d), "
      "range X=[%.2f, %.2f]m Y=[%.2f, %.2f]m, %.3fm/px, "
      "camera=(x=%.3f, y=%.3f, z=%.3fm, "
      "roll=%.2f, pitch_down=%.2f, yaw=%.2fdeg), "
      "valid_lut=%.2f%%, GPU=%s, processing=NV12-to-BEV/latest-only, "
      "ROS=%s (max=%.1fHz, 0=unlimited), preview=%s (max=%.1fHz)",
      input_topic_.c_str(),
      input_crop_width_,
      input_crop_height_,
      camera_model_.image_width,
      camera_model_.image_height,
      input_crop_top_,
      expected_input_fps_,
      output_topic_.c_str(),
      bev_config_.output_width,
      bev_config_.output_height,
      bev_config_.x_min_m,
      bev_config_.x_max_m,
      bev_config_.y_min_m,
      bev_config_.y_max_m,
      bev_config_.meter_per_pixel,
      camera_model_.position_vehicle_m[0],
      camera_model_.position_vehicle_m[1],
      camera_model_.position_vehicle_m[2],
      applied_roll_deg_.load(std::memory_order_relaxed),
      applied_pitch_down_deg_.load(std::memory_order_relaxed),
      camera_yaw_deg_,
      valid_lut_percent_.load(std::memory_order_relaxed),
      startup_processor->deviceName().c_str(),
      publish_enabled_ ? "on" : "off",
      publish_max_fps_,
      preview_enabled_ ? "on" : "off",
      preview_max_fps_);
    if (lane_reconstruction_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "BEV lane reconstruction: output=%s mono8, "
        "seed/trusted X=[%.2f, %.2f]m, track to %.2fm, "
        "maximum extrapolation=%.2fm, brightness near/far=%d/%d, "
        "window near/far=%.2f/%.2fm, pixel weight=%.2f, "
        "expected width=%.3fm, line=%.3fm",
        lane_output_topic_.c_str(),
        lane_reconstructor_config_.observation_minimum_x_m,
        lane_reconstructor_config_.observation_maximum_x_m,
        lane_reconstructor_config_.reconstruction_maximum_x_m,
        lane_reconstructor_config_.maximum_extrapolation_m,
        lane_reconstructor_config_.minimum_brightness,
        lane_reconstructor_config_.far_minimum_brightness,
        lane_reconstructor_config_.sliding_window_half_width_near_m,
        lane_reconstructor_config_.sliding_window_half_width_far_m,
        lane_reconstructor_config_.sliding_window_measurement_weight,
        lane_reconstructor_config_.expected_lane_width_m,
        lane_reconstructor_config_.output_line_thickness_m);
      RCLCPP_INFO(
        get_logger(),
        "BEV lane appearance gate: saturation<=%d, local contrast>=%d, "
        "background<=%d, tracked width near/far=%.2f/%.2fm, "
        "preview overlay=%s result-only=%s alpha=%.2f, lateral margin=%.2fm",
        lane_reconstructor_config_.maximum_saturation,
        lane_reconstructor_config_.minimum_local_contrast,
        lane_reconstructor_config_.maximum_local_background_brightness,
        lane_reconstructor_config_.tracked_lane_mark_width_near_m,
        lane_reconstructor_config_.tracked_lane_mark_width_far_m,
        lane_preview_enabled_ ? "on" : "off",
        lane_preview_result_only_enabled_ ? "on" : "off",
        lane_preview_overlay_alpha_,
        lane_reconstructor_config_.output_lateral_margin_m);
      RCLCPP_INFO(
        get_logger(),
        "BEV lane continuity: temporal=%s, lateral jump near/far="
        "%.2f/%.2fm, heading jump=%.1fdeg, confirm/hold=%d/%d frames, "
        "centerline pair smooth=%.2f/%.2f, transition=%.2fm decay=%.2f, "
        "single-boundary=%s side-lock=%s(pair/reset=%d/%d frames, "
        "window=%.2fm curvature<=%.2f/m step<=%.1fdeg)",
        lane_reconstructor_config_.temporal_tracking_enabled ? "on" : "off",
        lane_reconstructor_config_.temporal_maximum_lateral_jump_near_m,
        lane_reconstructor_config_.temporal_maximum_lateral_jump_far_m,
        lane_reconstructor_config_.temporal_maximum_heading_jump_deg,
        lane_reconstructor_config_.temporal_confirmation_frames,
        lane_reconstructor_config_.temporal_hold_frames,
        lane_reconstructor_config_.centerline_midpoint_smoothing_weight,
        lane_reconstructor_config_.centerline_temporal_current_weight,
        lane_reconstructor_config_.centerline_transition_maximum_correction_m,
        lane_reconstructor_config_.centerline_transition_correction_decay,
        lane_reconstructor_config_.centerline_from_single_boundary_enabled ?
        "on" : "off",
        lane_reconstructor_config_.single_boundary_side_lock_enabled ?
        "on" : "off",
        lane_reconstructor_config_.single_boundary_side_pair_confirmation_frames,
        lane_reconstructor_config_.single_boundary_side_lock_reset_frames,
        lane_reconstructor_config_.centerline_tangent_window_m,
        lane_reconstructor_config_.centerline_maximum_curvature_per_m,
        lane_reconstructor_config_.centerline_maximum_heading_step_deg);
    }
    RCLCPP_INFO(
      get_logger(),
      "Startup extrinsics: height_source=%s, "
      "attitude_source=%s, "
      "height=%.4fm, roll=%.3fdeg, "
      "pitch=%.3fdeg, downward_pitch=%.3fdeg, fixed yaw=%.3fdeg. "
      "The LUT remains fixed after this measurement.",
      measurement.height_source.c_str(),
      measurement.attitude_source.c_str(),
      camera_model_.position_vehicle_m[2],
      applied_roll_deg_.load(std::memory_order_relaxed),
      -applied_pitch_down_deg_.load(std::memory_order_relaxed),
      applied_pitch_down_deg_.load(std::memory_order_relaxed),
      camera_yaw_deg_);
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
    // A missing or node-name-mismatched YAML must not fall back silently to
    // C++ defaults because the measured pose and BEV bounds are safety-critical.
    declare_parameter<int>("configuration_version", 0);
    declare_parameter<bool>("performance_measurement_enabled", false);

    declare_parameter<std::string>("input_topic", "/camera/bev_input");
    declare_parameter<std::string>("output_topic", "/camera/image_bev");
    declare_parameter<std::string>("output_frame_id", "front_axle_bev");
    declare_parameter<std::string>(
      "startup_ground_reference_topic", "/camera/startup_ground_normal");
    declare_parameter<std::string>(
      "startup_ground_reference_frame_id", "camera_optical_frame");
    declare_parameter<double>("expected_input_fps", 80.0);
    declare_parameter<double>("input_bottom_fraction", 0.70);

    declare_parameter<bool>("publish_enabled", true);
    declare_parameter<double>("publish_max_fps", 0.0);
    declare_parameter<bool>("preview_enabled", true);
    declare_parameter<double>("preview_max_fps", 60.0);
    declare_parameter<std::string>("preview_window_name", "BEV image");
    declare_parameter<int>("preview_max_width", 1280);
    declare_parameter<int>("preview_max_height", 720);

    declare_parameter<int>("input_width", 1280);
    declare_parameter<int>("input_height", 720);
    declare_parameter<double>("fx", 701.751174926);
    declare_parameter<double>("fy", 701.420440674);
    declare_parameter<double>("cx", 643.032653809);
    declare_parameter<double>("cy", 352.621124268);

    declare_parameter<double>("camera_x_m", 0.0);
    declare_parameter<double>("camera_y_m", 0.0);
    declare_parameter<double>("camera_yaw_deg", 0.0);

    declare_parameter<double>("measurement_stereo_fps", 30.0);
    declare_parameter<int>("measurement_stereo_width", 1280);
    declare_parameter<int>("measurement_stereo_height", 800);
    declare_parameter<int>("measurement_depth_queue_size", 2);
    declare_parameter<int>("measurement_stereo_subpixel_fractional_bits", 5);
    declare_parameter<int>("measurement_stereo_left_right_check_threshold", 5);
    declare_parameter<int>("measurement_stereo_confidence_threshold", 55);
    declare_parameter<int>("measurement_stereo_disparity_shift", 0);
    declare_parameter<double>("measurement_imu_rate_hz", 400.0);
    declare_parameter<int>("measurement_imu_queue_size", 200);
    declare_parameter<int>("measurement_imu_max_batch_reports", 5);
    declare_parameter<double>("measurement_maximum_imu_pair_skew_sec", 0.003);
    declare_parameter<double>("measurement_warmup_sec", 2.0);
    declare_parameter<double>(
      "measurement_ir_dot_projector_intensity", 1.0);
    declare_parameter<bool>("manual_camera_height_enabled", false);
    declare_parameter<double>("manual_camera_height_m", 0.20);
    declare_parameter<int>("measurement_roi_width", 456);
    declare_parameter<int>("measurement_roi_height", 228);
    declare_parameter<int>("measurement_point_sample_step", 2);
    declare_parameter<int>("measurement_minimum_valid_points", 5080);
    declare_parameter<double>("measurement_minimum_depth_m", 0.30);
    declare_parameter<double>("measurement_maximum_depth_m", 3.00);
    declare_parameter<double>("measurement_minimum_height_m", 0.10);
    declare_parameter<double>("measurement_maximum_height_m", 0.40);
    declare_parameter<int>("measurement_plane_ransac_iterations", 200);
    declare_parameter<double>(
      "measurement_plane_inlier_threshold_m", 0.008);
    declare_parameter<int>("measurement_plane_minimum_inliers", 3656);
    declare_parameter<double>(
      "measurement_plane_minimum_inlier_ratio", 0.70);
    declare_parameter<double>(
      "measurement_plane_maximum_residual_mad_m", 0.005);
    declare_parameter<double>(
      "measurement_plane_maximum_imu_difference_deg", 5.0);
    declare_parameter<std::string>("measurement_attitude_source", "depth");
    declare_parameter<double>("measurement_imu_roll_bias_deg", 0.0);
    declare_parameter<double>("measurement_imu_pitch_bias_deg", 0.0);
    declare_parameter<int>("measurement_imu_sample_count", 1200);
    declare_parameter<double>(
      "measurement_imu_max_direction_rms_deg", 0.50);
    declare_parameter<double>("measurement_imu_accel_min_mps2", 8.30);
    declare_parameter<double>("measurement_imu_accel_max_mps2", 11.30);
    declare_parameter<double>(
      "measurement_imu_gyroscope_mean_maximum_degps", 0.80);
    declare_parameter<double>(
      "measurement_imu_gyroscope_stddev_maximum_degps", 1.40);
    declare_parameter<int>("measurement_stable_plane_frame_count", 45);
    declare_parameter<double>("measurement_maximum_height_stddev_m", 0.003);
    declare_parameter<double>(
      "measurement_maximum_plane_normal_rms_deg", 0.25);
    declare_parameter<double>("measurement_timeout_sec", 45.0);

    declare_parameter<double>("x_min_m", 0.0);
    declare_parameter<double>("x_max_m", 3.0);
    declare_parameter<double>("y_min_m", -0.6);
    declare_parameter<double>("y_max_m", 0.6);
    declare_parameter<double>("meter_per_pixel", 0.01);
    declare_parameter<int>("output_width", 120);
    declare_parameter<int>("output_height", 300);

    // Bright-lane reconstruction is intentionally independent from the CUDA
    // warp. The raw color BEV remains available on output_topic while this
    // stage publishes a clean MONO8 drive-centerline image on lane_output_topic.
    declare_parameter<bool>("lane_reconstruction_enabled", true);
    declare_parameter<std::string>(
      "lane_output_topic", "/camera/image_bev_lane");
    declare_parameter<bool>("lane_preview_enabled", true);
    declare_parameter<bool>("lane_preview_result_only_enabled", false);
    declare_parameter<bool>("lane_preview_sliding_windows_enabled", false);
    declare_parameter<double>("preview_x_origin_m", 0.02);
    declare_parameter<double>("lane_output_lateral_margin_m", 0.70);
    declare_parameter<double>("lane_preview_overlay_alpha", 0.8);
    declare_parameter<int>("lane_minimum_brightness", 160);
    declare_parameter<int>("lane_far_minimum_brightness", 110);
    declare_parameter<int>("lane_maximum_saturation", 80);
    declare_parameter<int>("lane_brightness_blur_kernel", 1);
    declare_parameter<double>("lane_vertical_close_m", 0.05);
    declare_parameter<double>("lane_minimum_mark_width_m", 0.015);
    declare_parameter<double>("lane_maximum_mark_width_m", 0.030);
    declare_parameter<int>("lane_minimum_local_contrast", 55);
    declare_parameter<int>(
      "lane_maximum_local_background_brightness", 140);
    declare_parameter<double>("lane_local_background_band_m", 0.04);
    declare_parameter<double>(
      "lane_tracked_mark_width_near_m", 0.11);
    declare_parameter<double>(
      "lane_tracked_mark_width_far_m", 0.22);
    declare_parameter<double>(
      "lane_measurement_lateral_gate_near_m", 0.05);
    declare_parameter<double>(
      "lane_measurement_lateral_gate_far_m", 0.10);
    declare_parameter<int>("lane_row_step_px", 2);
    declare_parameter<double>("lane_observation_minimum_x_m", 0.20);
    declare_parameter<double>("lane_observation_maximum_x_m", 1.30);
    declare_parameter<double>("lane_reconstruction_minimum_x_m", 0.10);
    declare_parameter<double>("lane_reconstruction_maximum_x_m", 3.0);
    declare_parameter<double>("lane_maximum_extrapolation_m", 0.0);
    declare_parameter<double>("lane_sliding_window_step_m", 0.04);
    declare_parameter<double>("lane_sliding_window_length_m", 0.18);
    declare_parameter<double>("lane_sliding_window_half_width_near_m", 0.08);
    declare_parameter<double>("lane_sliding_window_half_width_far_m", 0.14);
    declare_parameter<double>("lane_sliding_window_measurement_weight", 0.88);
    declare_parameter<double>("lane_sliding_window_heading_weight", 0.50);
    declare_parameter<double>("lane_maximum_tracking_arc_length_m", 3.20);
    declare_parameter<double>("lane_maximum_gap_fill_m", 0.30);
    declare_parameter<double>("lane_measured_point_smoothing_weight", 0.60);
    declare_parameter<int>("lane_minimum_window_pixel_count", 6);
    declare_parameter<double>("lane_expected_width_m", 0.65);
    declare_parameter<double>("lane_width_tolerance_m", 0.08);
    declare_parameter<double>("lane_initial_center_tolerance_m", 0.60);
    declare_parameter<double>("lane_single_initial_tolerance_m", 0.20);
    declare_parameter<double>("lane_maximum_tracking_gap_m", 0.16);
    declare_parameter<int>("lane_minimum_points", 6);
    declare_parameter<int>("lane_minimum_counterpart_points", 3);
    declare_parameter<bool>("lane_allow_single_lane", true);
    declare_parameter<bool>(
      "lane_centerline_from_single_boundary_enabled", true);
    declare_parameter<bool>(
      "lane_centerline_preserve_reference_shape", false);
    declare_parameter<bool>("lane_single_boundary_side_lock_enabled", true);
    declare_parameter<int>(
      "lane_single_boundary_side_pair_confirmation_frames", 2);
    declare_parameter<int>(
      "lane_single_boundary_side_lock_reset_frames", 30);
    declare_parameter<double>(
      "lane_centerline_midpoint_smoothing_weight", 0.45);
    declare_parameter<double>(
      "lane_centerline_temporal_current_weight", 0.60);
    declare_parameter<double>(
      "lane_centerline_transition_maximum_correction_m", 0.15);
    declare_parameter<double>(
      "lane_centerline_transition_correction_decay", 0.70);
    declare_parameter<double>("lane_centerline_tangent_window_m", 0.20);
    declare_parameter<double>(
      "lane_centerline_maximum_curvature_per_m", 1.25);
    declare_parameter<double>(
      "lane_centerline_maximum_heading_step_deg", 8.0);
    declare_parameter<bool>("lane_temporal_tracking_enabled", false);
    declare_parameter<double>(
      "lane_temporal_maximum_lateral_jump_near_m", 0.06);
    declare_parameter<double>(
      "lane_temporal_maximum_lateral_jump_far_m", 0.12);
    declare_parameter<double>(
      "lane_temporal_maximum_heading_jump_deg", 15.0);
    declare_parameter<int>("lane_temporal_confirmation_frames", 4);
    declare_parameter<int>("lane_temporal_hold_frames", 0);
    declare_parameter<double>("lane_output_line_thickness_m", 0.02);

    declare_parameter<double>("status_log_interval_sec", 5.0);
    declare_parameter<double>("startup_timeout_sec", 12.0);
    declare_parameter<double>("stabilization_settle_sec", 5.5);
  }

  void readParameters()
  {
    configuration_version_ = static_cast<int>(
      get_parameter("configuration_version").as_int());
    performance_measurement_enabled_ =
      get_parameter("performance_measurement_enabled").as_bool();

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    startup_ground_reference_topic_ =
      get_parameter("startup_ground_reference_topic").as_string();
    startup_ground_reference_frame_id_ =
      get_parameter("startup_ground_reference_frame_id").as_string();
    expected_input_fps_ = get_parameter("expected_input_fps").as_double();
    input_bottom_fraction_ =
      get_parameter("input_bottom_fraction").as_double();

    publish_enabled_ = get_parameter("publish_enabled").as_bool();
    publish_max_fps_ = get_parameter("publish_max_fps").as_double();
    preview_enabled_ = get_parameter("preview_enabled").as_bool();
    preview_max_fps_ = get_parameter("preview_max_fps").as_double();
    preview_window_name_ = get_parameter("preview_window_name").as_string();
    preview_max_width_ =
      static_cast<int>(get_parameter("preview_max_width").as_int());
    preview_max_height_ =
      static_cast<int>(get_parameter("preview_max_height").as_int());
    if (performance_measurement_enabled_) {
      preview_enabled_ = false;
    }

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
      0.0);
    camera_yaw_deg_ = get_parameter("camera_yaw_deg").as_double();
    camera_model_.rotation_vehicle_from_camera =
      mountRotationVehicleFromCamera(
      0.0,
      0.0,
      degToRad(camera_yaw_deg_));

    startup_measurement_config_.stereo_fps =
      get_parameter("measurement_stereo_fps").as_double();
    startup_measurement_config_.stereo_width = static_cast<int>(
      get_parameter("measurement_stereo_width").as_int());
    startup_measurement_config_.stereo_height = static_cast<int>(
      get_parameter("measurement_stereo_height").as_int());
    startup_measurement_config_.depth_queue_size = static_cast<int>(
      get_parameter("measurement_depth_queue_size").as_int());
    startup_measurement_config_.stereo_subpixel_fractional_bits =
      static_cast<int>(get_parameter(
        "measurement_stereo_subpixel_fractional_bits").as_int());
    startup_measurement_config_.stereo_left_right_check_threshold =
      static_cast<int>(get_parameter(
        "measurement_stereo_left_right_check_threshold").as_int());
    startup_measurement_config_.stereo_confidence_threshold =
      static_cast<int>(get_parameter(
        "measurement_stereo_confidence_threshold").as_int());
    startup_measurement_config_.stereo_disparity_shift =
      static_cast<int>(get_parameter(
        "measurement_stereo_disparity_shift").as_int());
    startup_measurement_config_.imu_rate_hz =
      get_parameter("measurement_imu_rate_hz").as_double();
    startup_measurement_config_.imu_queue_size = static_cast<int>(
      get_parameter("measurement_imu_queue_size").as_int());
    startup_measurement_config_.imu_max_batch_reports = static_cast<int>(
      get_parameter("measurement_imu_max_batch_reports").as_int());
    startup_measurement_config_.maximum_imu_pair_skew_sec =
      get_parameter("measurement_maximum_imu_pair_skew_sec").as_double();
    startup_measurement_config_.warmup_sec =
      get_parameter("measurement_warmup_sec").as_double();
    startup_measurement_config_.ir_dot_projector_intensity =
      get_parameter(
      "measurement_ir_dot_projector_intensity").as_double();
    startup_measurement_config_.manual_camera_height_enabled =
      get_parameter("manual_camera_height_enabled").as_bool();
    startup_measurement_config_.manual_camera_height_m =
      get_parameter("manual_camera_height_m").as_double();
    startup_measurement_config_.roi_width = static_cast<int>(
      get_parameter("measurement_roi_width").as_int());
    startup_measurement_config_.roi_height = static_cast<int>(
      get_parameter("measurement_roi_height").as_int());
    startup_measurement_config_.point_sample_step = static_cast<int>(
      get_parameter("measurement_point_sample_step").as_int());
    startup_measurement_config_.minimum_valid_points = static_cast<int>(
      get_parameter("measurement_minimum_valid_points").as_int());
    startup_measurement_config_.minimum_depth_m =
      get_parameter("measurement_minimum_depth_m").as_double();
    startup_measurement_config_.maximum_depth_m =
      get_parameter("measurement_maximum_depth_m").as_double();
    startup_measurement_config_.minimum_height_m =
      get_parameter("measurement_minimum_height_m").as_double();
    startup_measurement_config_.maximum_height_m =
      get_parameter("measurement_maximum_height_m").as_double();
    startup_measurement_config_.plane_ransac_iterations = static_cast<int>(
      get_parameter("measurement_plane_ransac_iterations").as_int());
    startup_measurement_config_.plane_inlier_threshold_m =
      get_parameter("measurement_plane_inlier_threshold_m").as_double();
    startup_measurement_config_.plane_minimum_inliers = static_cast<int>(
      get_parameter("measurement_plane_minimum_inliers").as_int());
    startup_measurement_config_.plane_minimum_inlier_ratio =
      get_parameter("measurement_plane_minimum_inlier_ratio").as_double();
    startup_measurement_config_.plane_maximum_residual_mad_m =
      get_parameter(
      "measurement_plane_maximum_residual_mad_m").as_double();
    startup_measurement_config_.plane_maximum_imu_difference_deg =
      get_parameter(
      "measurement_plane_maximum_imu_difference_deg").as_double();
    startup_measurement_config_.attitude_source =
      parseStartupAttitudeSource(
      get_parameter("measurement_attitude_source").as_string());
    startup_measurement_config_.imu_roll_bias_deg =
      get_parameter("measurement_imu_roll_bias_deg").as_double();
    startup_measurement_config_.imu_pitch_bias_deg =
      get_parameter("measurement_imu_pitch_bias_deg").as_double();
    startup_measurement_config_.imu_sample_count = static_cast<int>(
      get_parameter("measurement_imu_sample_count").as_int());
    startup_measurement_config_.imu_max_direction_rms_deg =
      get_parameter(
      "measurement_imu_max_direction_rms_deg").as_double();
    startup_measurement_config_.imu_accel_min_mps2 =
      get_parameter("measurement_imu_accel_min_mps2").as_double();
    startup_measurement_config_.imu_accel_max_mps2 =
      get_parameter("measurement_imu_accel_max_mps2").as_double();
    startup_measurement_config_.imu_gyroscope_mean_maximum_degps =
      get_parameter(
      "measurement_imu_gyroscope_mean_maximum_degps").as_double();
    startup_measurement_config_.imu_gyroscope_stddev_maximum_degps =
      get_parameter(
      "measurement_imu_gyroscope_stddev_maximum_degps").as_double();
    startup_measurement_config_.stable_plane_frame_count = static_cast<int>(
      get_parameter("measurement_stable_plane_frame_count").as_int());
    startup_measurement_config_.maximum_height_stddev_m =
      get_parameter(
      "measurement_maximum_height_stddev_m").as_double();
    startup_measurement_config_.maximum_plane_normal_rms_deg =
      get_parameter(
      "measurement_maximum_plane_normal_rms_deg").as_double();
    startup_measurement_config_.timeout_sec =
      get_parameter("measurement_timeout_sec").as_double();

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

    lane_reconstruction_enabled_ =
      get_parameter("lane_reconstruction_enabled").as_bool();
    lane_output_topic_ = get_parameter("lane_output_topic").as_string();
    lane_preview_enabled_ =
      get_parameter("lane_preview_enabled").as_bool();
    lane_preview_result_only_enabled_ =
      get_parameter("lane_preview_result_only_enabled").as_bool();
    lane_preview_sliding_windows_enabled_ =
      get_parameter("lane_preview_sliding_windows_enabled").as_bool();
    preview_x_origin_m_ = get_parameter("preview_x_origin_m").as_double();
    lane_reconstructor_config_.output_lateral_margin_m =
      get_parameter("lane_output_lateral_margin_m").as_double();
    lane_preview_overlay_alpha_ =
      get_parameter("lane_preview_overlay_alpha").as_double();
    lane_reconstructor_config_.x_min_m = bev_config_.x_min_m;
    lane_reconstructor_config_.x_max_m = bev_config_.x_max_m;
    lane_reconstructor_config_.y_min_m = bev_config_.y_min_m;
    lane_reconstructor_config_.y_max_m = bev_config_.y_max_m;
    lane_reconstructor_config_.meter_per_pixel =
      bev_config_.meter_per_pixel;
    lane_reconstructor_config_.image_width = bev_config_.output_width;
    lane_reconstructor_config_.image_height = bev_config_.output_height;
    lane_reconstructor_config_.minimum_brightness = static_cast<int>(
      get_parameter("lane_minimum_brightness").as_int());
    lane_reconstructor_config_.far_minimum_brightness = static_cast<int>(
      get_parameter("lane_far_minimum_brightness").as_int());
    lane_reconstructor_config_.maximum_saturation = static_cast<int>(
      get_parameter("lane_maximum_saturation").as_int());
    lane_reconstructor_config_.brightness_blur_kernel = static_cast<int>(
      get_parameter("lane_brightness_blur_kernel").as_int());
    lane_reconstructor_config_.vertical_close_m =
      get_parameter("lane_vertical_close_m").as_double();
    lane_reconstructor_config_.minimum_lane_mark_width_m =
      get_parameter("lane_minimum_mark_width_m").as_double();
    lane_reconstructor_config_.maximum_lane_mark_width_m =
      get_parameter("lane_maximum_mark_width_m").as_double();
    lane_reconstructor_config_.minimum_local_contrast = static_cast<int>(
      get_parameter("lane_minimum_local_contrast").as_int());
    lane_reconstructor_config_.maximum_local_background_brightness =
      static_cast<int>(get_parameter(
        "lane_maximum_local_background_brightness").as_int());
    lane_reconstructor_config_.local_background_band_m =
      get_parameter("lane_local_background_band_m").as_double();
    lane_reconstructor_config_.tracked_lane_mark_width_near_m =
      get_parameter("lane_tracked_mark_width_near_m").as_double();
    lane_reconstructor_config_.tracked_lane_mark_width_far_m =
      get_parameter("lane_tracked_mark_width_far_m").as_double();
    lane_reconstructor_config_.measurement_lateral_gate_near_m =
      get_parameter("lane_measurement_lateral_gate_near_m").as_double();
    lane_reconstructor_config_.measurement_lateral_gate_far_m =
      get_parameter("lane_measurement_lateral_gate_far_m").as_double();
    lane_reconstructor_config_.row_step_px = static_cast<int>(
      get_parameter("lane_row_step_px").as_int());
    lane_reconstructor_config_.observation_minimum_x_m =
      get_parameter("lane_observation_minimum_x_m").as_double();
    lane_reconstructor_config_.observation_maximum_x_m =
      get_parameter("lane_observation_maximum_x_m").as_double();
    lane_reconstructor_config_.reconstruction_minimum_x_m =
      get_parameter("lane_reconstruction_minimum_x_m").as_double();
    lane_reconstructor_config_.reconstruction_maximum_x_m =
      get_parameter("lane_reconstruction_maximum_x_m").as_double();
    lane_reconstructor_config_.maximum_extrapolation_m =
      get_parameter("lane_maximum_extrapolation_m").as_double();
    lane_reconstructor_config_.sliding_window_step_m =
      get_parameter("lane_sliding_window_step_m").as_double();
    lane_reconstructor_config_.sliding_window_length_m =
      get_parameter("lane_sliding_window_length_m").as_double();
    lane_reconstructor_config_.sliding_window_half_width_near_m =
      get_parameter("lane_sliding_window_half_width_near_m").as_double();
    lane_reconstructor_config_.sliding_window_half_width_far_m =
      get_parameter("lane_sliding_window_half_width_far_m").as_double();
    lane_reconstructor_config_.sliding_window_measurement_weight =
      get_parameter("lane_sliding_window_measurement_weight").as_double();
    lane_reconstructor_config_.sliding_window_heading_weight =
      get_parameter("lane_sliding_window_heading_weight").as_double();
    lane_reconstructor_config_.maximum_tracking_arc_length_m =
      get_parameter("lane_maximum_tracking_arc_length_m").as_double();
    lane_reconstructor_config_.maximum_gap_fill_m =
      get_parameter("lane_maximum_gap_fill_m").as_double();
    lane_reconstructor_config_.measured_point_smoothing_weight =
      get_parameter("lane_measured_point_smoothing_weight").as_double();
    lane_reconstructor_config_.minimum_window_pixel_count = static_cast<int>(
      get_parameter("lane_minimum_window_pixel_count").as_int());
    lane_reconstructor_config_.expected_lane_width_m =
      get_parameter("lane_expected_width_m").as_double();
    lane_reconstructor_config_.lane_width_tolerance_m =
      get_parameter("lane_width_tolerance_m").as_double();
    lane_reconstructor_config_.initial_center_tolerance_m =
      get_parameter("lane_initial_center_tolerance_m").as_double();
    lane_reconstructor_config_.single_lane_initial_tolerance_m =
      get_parameter("lane_single_initial_tolerance_m").as_double();
    lane_reconstructor_config_.maximum_tracking_gap_m =
      get_parameter("lane_maximum_tracking_gap_m").as_double();
    lane_reconstructor_config_.minimum_points = static_cast<int>(
      get_parameter("lane_minimum_points").as_int());
    lane_reconstructor_config_.minimum_counterpart_points = static_cast<int>(
      get_parameter("lane_minimum_counterpart_points").as_int());
    lane_reconstructor_config_.allow_single_lane =
      get_parameter("lane_allow_single_lane").as_bool();
    lane_reconstructor_config_.centerline_from_single_boundary_enabled =
      get_parameter(
        "lane_centerline_from_single_boundary_enabled").as_bool();
    lane_reconstructor_config_.centerline_preserve_reference_shape =
      get_parameter("lane_centerline_preserve_reference_shape").as_bool();
    lane_reconstructor_config_.single_boundary_side_lock_enabled =
      get_parameter("lane_single_boundary_side_lock_enabled").as_bool();
    lane_reconstructor_config_.single_boundary_side_pair_confirmation_frames =
      static_cast<int>(get_parameter(
          "lane_single_boundary_side_pair_confirmation_frames").as_int());
    lane_reconstructor_config_.single_boundary_side_lock_reset_frames =
      static_cast<int>(get_parameter(
          "lane_single_boundary_side_lock_reset_frames").as_int());
    lane_reconstructor_config_.centerline_midpoint_smoothing_weight =
      get_parameter(
        "lane_centerline_midpoint_smoothing_weight").as_double();
    lane_reconstructor_config_.centerline_temporal_current_weight =
      get_parameter(
        "lane_centerline_temporal_current_weight").as_double();
    lane_reconstructor_config_.centerline_transition_maximum_correction_m =
      get_parameter(
        "lane_centerline_transition_maximum_correction_m").as_double();
    lane_reconstructor_config_.centerline_transition_correction_decay =
      get_parameter(
        "lane_centerline_transition_correction_decay").as_double();
    lane_reconstructor_config_.centerline_tangent_window_m =
      get_parameter("lane_centerline_tangent_window_m").as_double();
    lane_reconstructor_config_.centerline_maximum_curvature_per_m =
      get_parameter("lane_centerline_maximum_curvature_per_m").as_double();
    lane_reconstructor_config_.centerline_maximum_heading_step_deg =
      get_parameter("lane_centerline_maximum_heading_step_deg").as_double();
    lane_reconstructor_config_.temporal_tracking_enabled =
      get_parameter("lane_temporal_tracking_enabled").as_bool();
    lane_reconstructor_config_.temporal_maximum_lateral_jump_near_m =
      get_parameter(
        "lane_temporal_maximum_lateral_jump_near_m").as_double();
    lane_reconstructor_config_.temporal_maximum_lateral_jump_far_m =
      get_parameter(
        "lane_temporal_maximum_lateral_jump_far_m").as_double();
    lane_reconstructor_config_.temporal_maximum_heading_jump_deg =
      get_parameter("lane_temporal_maximum_heading_jump_deg").as_double();
    lane_reconstructor_config_.temporal_confirmation_frames =
      static_cast<int>(
        get_parameter("lane_temporal_confirmation_frames").as_int());
    lane_reconstructor_config_.temporal_hold_frames = static_cast<int>(
      get_parameter("lane_temporal_hold_frames").as_int());
    lane_reconstructor_config_.output_line_thickness_m =
      get_parameter("lane_output_line_thickness_m").as_double();

    status_log_interval_sec_ =
      get_parameter("status_log_interval_sec").as_double();
    startup_timeout_sec_ = get_parameter("startup_timeout_sec").as_double();
    stabilization_settle_sec_ =
      get_parameter("stabilization_settle_sec").as_double();
  }

  void validateParameters() const
  {
    if (configuration_version_ != 1) {
      throw std::invalid_argument(
              "configuration_version must be 1; check that bev_config.yaml "
              "was loaded for the bev_processor node");
    }
    if (input_topic_.empty()) {
      throw std::invalid_argument("input_topic must not be empty");
    }
    if (
      startup_ground_reference_topic_.empty() ||
      startup_ground_reference_frame_id_.empty())
    {
      throw std::invalid_argument(
              "startup ground reference topic and frame_id must not be empty");
    }
    if (publish_enabled_ && output_topic_.empty()) {
      throw std::invalid_argument(
              "output_topic must not be empty when publishing is enabled");
    }
    if (lane_reconstruction_enabled_ && lane_output_topic_.empty()) {
      throw std::invalid_argument(
              "lane_output_topic must not be empty when lane reconstruction "
              "is enabled");
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
      bev_config_.x_min_m < 0.0 ||
      bev_config_.x_max_m <= bev_config_.x_min_m ||
      bev_config_.y_max_m <= bev_config_.y_min_m ||
      bev_config_.meter_per_pixel <= 0.0 ||
      bev_config_.output_width <= 0 ||
      bev_config_.output_height <= 0)
    {
      throw std::invalid_argument("invalid BEV bounds");
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
      !std::isfinite(input_bottom_fraction_) ||
      input_bottom_fraction_ <= 0.0 ||
      input_bottom_fraction_ > 1.0 ||
      publish_max_fps_ < 0.0 ||
      preview_max_fps_ <= 0.0 ||
      preview_max_width_ <= 0 ||
      preview_max_height_ <= 0 ||
      !std::isfinite(lane_preview_overlay_alpha_) ||
      lane_preview_overlay_alpha_ < 0.0 ||
      lane_preview_overlay_alpha_ > 1.0 ||
      !std::isfinite(preview_x_origin_m_) ||
      preview_x_origin_m_ < bev_config_.x_min_m ||
      preview_x_origin_m_ >= bev_config_.x_max_m ||
      !std::isfinite(lane_reconstructor_config_.output_lateral_margin_m) ||
      lane_reconstructor_config_.output_lateral_margin_m < 0.0 ||
      status_log_interval_sec_ <= 0.0 ||
      startup_timeout_sec_ <= 0.0 ||
      !std::isfinite(stabilization_settle_sec_) ||
      stabilization_settle_sec_ < 0.0)
    {
      throw std::invalid_argument("invalid rate, preview, or status parameter");
    }
  }

  void initializeInputCropGeometry()
  {
    input_crop_width_ = camera_model_.image_width;
    input_crop_height_ = static_cast<int>(2 * std::llround(
        static_cast<double>(camera_model_.image_height) *
        input_bottom_fraction_ / 2.0));
    input_crop_height_ = std::clamp(
      input_crop_height_, 2, camera_model_.image_height);
    input_crop_top_ = camera_model_.image_height - input_crop_height_;
  }

  void publishStartupGroundReference(const std::string & attitude_source)
  {
    const cv::Vec3d up_camera = attitudeUpVector(
      startup_roll_deg_, startup_pitch_down_deg_);
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = startup_ground_reference_frame_id_;
    message.vector.x = up_camera[0];
    message.vector.y = up_camera[1];
    message.vector.z = up_camera[2];
    startup_ground_reference_publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(),
      "Published startup ground reference for camera stabilization: "
      "topic=%s frame=%s source=%s normal=(%.6f, %.6f, %.6f).",
      startup_ground_reference_topic_.c_str(),
      startup_ground_reference_frame_id_.c_str(),
      attitude_source.c_str(),
      up_camera[0], up_camera[1], up_camera[2]);
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
      input_crop_width_,
      input_crop_height_,
      lut.map_x,
      lut.map_y);

    const int valid_pixels = cv::countNonZero(lut.valid_mask);
    const int output_pixels =
      bev_config_.output_width * bev_config_.output_height;
    const double valid_percent =
      100.0 * static_cast<double>(valid_pixels) /
      static_cast<double>(output_pixels);

    bev_lut_ = lut;
    fused_coverage_reported_ = false;
    std::atomic_store_explicit(
      &gpu_processor_, std::move(processor), std::memory_order_release);
    valid_lut_percent_.store(valid_percent, std::memory_order_relaxed);
    applied_roll_deg_.store(roll_deg, std::memory_order_relaxed);
    applied_pitch_down_deg_.store(
      pitch_down_deg, std::memory_order_relaxed);

    if (source != nullptr) {
      RCLCPP_INFO(
        get_logger(),
        "BEV LUT installed from %s: height=%.3fm, roll=%.3f deg, "
        "pitch_down=%.3f deg, fixed yaw=%.3f deg, valid=%.2f%%",
        source, camera_model.position_vehicle_m[2],
        roll_deg, pitch_down_deg, camera_yaw_deg_, valid_percent);
    }
  }

  void onBevInput(camera_driver::msg::BevInput::ConstSharedPtr message)
  {
    received_total_.fetch_add(1U, std::memory_order_relaxed);
    received_interval_.fetch_add(1U, std::memory_order_relaxed);

    const bool dimensions_valid =
      static_cast<int>(message->source_width) == camera_model_.image_width &&
      static_cast<int>(message->source_height) == camera_model_.image_height &&
      static_cast<int>(message->source_crop_top) == input_crop_top_ &&
      static_cast<int>(message->cropped_width) == input_crop_width_ &&
      static_cast<int>(message->cropped_height) == input_crop_height_;
    const std::size_t minimum_step =
      static_cast<std::size_t>(input_crop_width_);
    const std::size_t nv12_rows =
      static_cast<std::size_t>(input_crop_height_) * 3U / 2U;
    const bool memory_valid =
      message->step >= minimum_step &&
      message->nv12.size() >=
      static_cast<std::size_t>(message->step) * nv12_rows;
    if (!dimensions_valid || !memory_valid)
    {
      invalid_total_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Rejected fused BEV input: expected source=%dx%d crop=top %dpx/%dx%d, "
        "got source=%ux%u crop=top %u/%ux%u (step=%u, data=%zu).",
        camera_model_.image_width,
        camera_model_.image_height,
        input_crop_top_,
        input_crop_width_,
        input_crop_height_,
        message->source_width,
        message->source_height,
        message->source_crop_top,
        message->cropped_width,
        message->cropped_height,
        message->step,
        message->nv12.size());
      return;
    }

    const auto received_at = SteadyClock::now();
    if (!first_camera_input_seen_) {
      first_camera_input_seen_ = true;
      first_camera_input_at_ = received_at;
      RCLCPP_INFO(
        get_logger(),
        "First camera frame received; suppressing BEV output for %.1fs "
        "while the unchanged fixed-reference stabilizer calibrates.",
        stabilization_settle_sec_);
    }
    const double camera_elapsed_sec = std::chrono::duration<double>(
      received_at - first_camera_input_at_).count();
    if (camera_elapsed_sec < stabilization_settle_sec_) {
      stabilization_settle_total_.fetch_add(1U, std::memory_order_relaxed);
      return;
    }

    recordPipelineLatency(
      message->header,
      stabilized_latency_samples_interval_,
      stabilized_latency_ns_interval_,
      stabilized_latency_ns_max_interval_);
    accepted_interval_.fetch_add(1U, std::memory_order_relaxed);

    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      latest_input_ = std::move(message);
      latest_input_received_at_ = received_at;
      ++input_generation_;
    }
    accepted_total_.fetch_add(1U, std::memory_order_relaxed);
    input_cv_.notify_one();
  }

  void processingLoop()
  {
    std::uint64_t processed_input_generation = 0U;

    while (!stop_.load(std::memory_order_acquire)) {
      camera_driver::msg::BevInput::ConstSharedPtr input;
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
        cv::Matx33d source_to_stabilized;
        for (int row = 0; row < 3; ++row) {
          for (int column = 0; column < 3; ++column) {
            source_to_stabilized(row, column) =
              input->source_to_stabilized_homography[
              static_cast<std::size_t>(row * 3 + column)];
          }
        }
        const double determinant = cv::determinant(
          cv::Mat(source_to_stabilized));
        if (
          !cv::checkRange(cv::Mat(source_to_stabilized)) ||
          !std::isfinite(determinant) || std::abs(determinant) < 1.0e-9)
        {
          throw std::runtime_error(
                  "camera supplied an invalid stabilization homography");
        }
        const cv::Matx33d stabilized_to_source =
          source_to_stabilized.inv(cv::DECOMP_LU);
        if (!fused_coverage_reported_) {
          const auto coverage = assessFusedRemapCoverage(
            bev_lut_,
            source_to_stabilized,
            static_cast<int>(input->source_width),
            static_cast<int>(input->source_height),
            static_cast<int>(input->source_crop_top));
          const double coverage_percent = 100.0 * coverage.coverage_ratio;
          if (coverage.coverage_ratio < 0.995) {
            RCLCPP_WARN(
              get_logger(),
              "Fused bottom-crop coverage is %.2f%% (%d/%d static LUT "
              "pixels). Increase bev_input_bottom_fraction if the far BEV "
              "edge is black.",
              coverage_percent,
              coverage.covered_pixels,
              coverage.valid_lut_pixels);
          } else {
            RCLCPP_INFO(
              get_logger(),
              "Fused bottom-crop coverage: %.2f%% (%d/%d static LUT pixels).",
              coverage_percent,
              coverage.covered_pixels,
              coverage.valid_lut_pixels);
          }
          fused_coverage_reported_ = true;
        }
        output->image = processor->process(
          input->nv12.data(),
          input->nv12.size(),
          static_cast<std::size_t>(input->step),
          stabilized_to_source,
          static_cast<int>(input->source_crop_top));
        if (lane_reconstructor_) {
          const auto lane_started_at = SteadyClock::now();
          const auto lane = lane_reconstructor_->reconstruct(output->image);
          const auto lane_finished_at = SteadyClock::now();
          const auto lane_process_ns = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              lane_finished_at - lane_started_at).count());
          lane_process_samples_interval_.fetch_add(
            1U, std::memory_order_relaxed);
          lane_process_ns_interval_.fetch_add(
            lane_process_ns, std::memory_order_relaxed);
          updateMaximum(lane_process_ns_max_interval_, lane_process_ns);
          output->lane_mask = lane.reconstructed_mask;
          output->left_lane_mask = lane.left_reconstructed_mask;
          output->right_lane_mask = lane.right_reconstructed_mask;
          output->lane_sliding_windows = lane.sliding_windows;
          latest_lane_points_.store(
            lane.measured_point_count, std::memory_order_relaxed);
          latest_lane_centerline_points_.store(
            lane.centerline_point_count, std::memory_order_relaxed);
          latest_lane_centerline_truncated_.store(
            lane.centerline_normal_offset_truncated,
            std::memory_order_relaxed);
          latest_lane_centerline_single_boundary_.store(
            lane.centerline_from_single_boundary,
            std::memory_order_relaxed);
          latest_lane_temporal_hold_.store(
            lane.temporal_hold_used, std::memory_order_relaxed);
          latest_lane_width_mm_.store(
            static_cast<int>(std::lround(
              1000.0 * lane.measured_lane_width_m)),
            std::memory_order_relaxed);
          latest_lane_reconstructed_maximum_x_mm_.store(
            static_cast<int>(std::lround(
              1000.0 * lane.reconstructed_maximum_x_m)),
            std::memory_order_relaxed);
          if (lane.valid) {
            lane_valid_total_.fetch_add(1U, std::memory_order_relaxed);
            lane_valid_interval_.fetch_add(1U, std::memory_order_relaxed);
          } else {
            lane_invalid_total_.fetch_add(1U, std::memory_order_relaxed);
            lane_invalid_interval_.fetch_add(1U, std::memory_order_relaxed);
          }
        }
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
        if (publish_enabled_) {
          output_publisher_->publish(
            makeBgr8Message(*frame, output_frame_id_));
        }
        if (lane_reconstruction_enabled_) {
          lane_output_publisher_->publish(
            makeMono8Message(*frame, output_frame_id_));
        }
        const auto published_at = SteadyClock::now();
        recordPipelineLatency(
          frame->header,
          bev_ready_latency_samples_interval_,
          bev_ready_latency_ns_interval_,
          bev_ready_latency_ns_max_interval_);
        recordSteadyLatency(
          published_at - frame->input_received_at,
          bev_stage_latency_samples_interval_,
          bev_stage_latency_ns_interval_,
          bev_stage_latency_ns_max_interval_);
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

    const int visible_height = std::clamp(
      static_cast<int>(std::lround(
        (bev_config_.x_max_m - preview_x_origin_m_) /
        bev_config_.meter_per_pixel)),
      1,
      bev_image.rows);
    const double displayed_x_max_m =
      bev_config_.x_max_m - preview_x_origin_m_;
    const double displayed_y_min_m =
      bev_config_.y_min_m -
      lane_reconstructor_config_.output_lateral_margin_m;
    const double displayed_y_max_m =
      bev_config_.y_max_m +
      lane_reconstructor_config_.output_lateral_margin_m;
    cv::Mat preview(
      visible_height + kPreviewTopMargin + kPreviewBottomMargin,
      bev_image.cols + kPreviewLeftMargin + kPreviewRightMargin,
      CV_8UC3,
      margin_color);
    const cv::Rect bev_region(
      kPreviewLeftMargin,
      kPreviewTopMargin,
      bev_image.cols,
      visible_height);
    bev_image(cv::Rect(0, 0, bev_image.cols, visible_height)).copyTo(
      preview(bev_region));
    cv::Mat displayed_bev = preview(bev_region);
    cv::Mat grid_overlay = displayed_bev.clone();

    const double first_x = 0.0;
    for (
      double x_m = first_x;
      x_m <= displayed_x_max_m + epsilon;
      x_m += grid_step_m)
    {
      const int row = std::clamp(
        static_cast<int>(std::lround(
          (displayed_x_max_m - x_m) /
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
    const double first_y =
      std::ceil(displayed_y_min_m / grid_step_m) * grid_step_m;
    for (
      double y_m = first_y;
      y_m <= displayed_y_max_m + epsilon;
      y_m += grid_step_m)
    {
      const int column = std::clamp(
        static_cast<int>(std::lround(
          (displayed_y_max_m - y_m) /
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
        displayed_y_max_m / bev_config_.meter_per_pixel)),
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
      const int lateral_margin_px = static_cast<int>(std::llround(
          lane_reconstructor_config_.output_lateral_margin_m /
          bev_config_.meter_per_pixel));
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::resizeWindow(
        preview_window_name_,
        std::min(
          preview_max_width_,
          bev_config_.output_width + 2 * lateral_margin_px +
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
          cv::Mat displayed_image = makeExpandedBevPreview(
            frame->image, lateral_margin_px);
          if (lane_preview_enabled_ && lane_preview_result_only_enabled_) {
            displayed_image = cv::Mat::zeros(
              displayed_image.size(), displayed_image.type());
          }
          if (
            lane_preview_enabled_ &&
            !frame->left_lane_mask.empty() &&
            !frame->right_lane_mask.empty())
          {
            displayed_image = makeLaneOverlayPreview(
              displayed_image,
              frame->left_lane_mask,
              frame->right_lane_mask,
              frame->lane_mask,
              lane_preview_result_only_enabled_ ?
              1.0 : lane_preview_overlay_alpha_);
          }
          if (
            lane_preview_sliding_windows_enabled_ &&
            !frame->lane_sliding_windows.empty())
          {
            displayed_image = makeSlidingWindowPreview(
              displayed_image, frame->lane_sliding_windows, bev_config_,
              lane_reconstructor_config_.output_lateral_margin_m);
          }
          cv::Mat preview_image = makeCoordinatePreview(displayed_image);
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

  void recordPipelineLatency(
    const std_msgs::msg::Header & header,
    std::atomic<std::uint64_t> & sample_count,
    std::atomic<std::uint64_t> & latency_ns_sum,
    std::atomic<std::uint64_t> & latency_ns_max)
  {
    if (!performance_measurement_enabled_) {
      return;
    }

    const rclcpp::Time frame_stamp(
      header.stamp,
      get_clock()->get_clock_type());
    const std::int64_t latency_ns =
      (get_clock()->now() - frame_stamp).nanoseconds();
    constexpr std::int64_t maximum_valid_latency_ns =
      60LL * 1000LL * 1000LL * 1000LL;
    if (latency_ns < 0 || latency_ns > maximum_valid_latency_ns) {
      return;
    }

    const auto valid_latency_ns = static_cast<std::uint64_t>(latency_ns);
    sample_count.fetch_add(1U, std::memory_order_relaxed);
    latency_ns_sum.fetch_add(valid_latency_ns, std::memory_order_relaxed);
    updateMaximum(latency_ns_max, valid_latency_ns);
  }

  void recordSteadyLatency(
    const SteadyClock::duration latency,
    std::atomic<std::uint64_t> & sample_count,
    std::atomic<std::uint64_t> & latency_ns_sum,
    std::atomic<std::uint64_t> & latency_ns_max)
  {
    if (!performance_measurement_enabled_) {
      return;
    }

    const auto latency_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(latency).count();
    constexpr std::int64_t maximum_valid_latency_ns =
      60LL * 1000LL * 1000LL * 1000LL;
    if (latency_ns < 0 || latency_ns > maximum_valid_latency_ns) {
      return;
    }

    const auto valid_latency_ns = static_cast<std::uint64_t>(latency_ns);
    sample_count.fetch_add(1U, std::memory_order_relaxed);
    latency_ns_sum.fetch_add(valid_latency_ns, std::memory_order_relaxed);
    updateMaximum(latency_ns_max, valid_latency_ns);
  }

  void logStatus()
  {
    const auto now = SteadyClock::now();
    const double elapsed_sec =
      std::chrono::duration<double>(now - status_started_at_).count();
    status_started_at_ = now;

    const auto received =
      received_interval_.exchange(0U, std::memory_order_relaxed);
    const auto accepted =
      accepted_interval_.exchange(0U, std::memory_order_relaxed);
    const auto processed =
      processed_interval_.exchange(0U, std::memory_order_relaxed);
    const auto skipped =
      skipped_interval_.exchange(0U, std::memory_order_relaxed);
    const auto published =
      published_interval_.exchange(0U, std::memory_order_relaxed);
    const auto previewed =
      previewed_interval_.exchange(0U, std::memory_order_relaxed);
    const auto lane_valid =
      lane_valid_interval_.exchange(0U, std::memory_order_relaxed);
    const auto lane_invalid =
      lane_invalid_interval_.exchange(0U, std::memory_order_relaxed);
    const auto lane_process_samples =
      lane_process_samples_interval_.exchange(0U, std::memory_order_relaxed);
    const auto lane_process_ns =
      lane_process_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto lane_process_ns_max =
      lane_process_ns_max_interval_.exchange(0U, std::memory_order_relaxed);
    const auto process_ns =
      process_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto process_ns_max =
      process_ns_max_interval_.exchange(0U, std::memory_order_relaxed);
    const auto stabilized_latency_samples =
      stabilized_latency_samples_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto stabilized_latency_ns =
      stabilized_latency_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto stabilized_latency_ns_max =
      stabilized_latency_ns_max_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto bev_ready_latency_samples =
      bev_ready_latency_samples_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto bev_ready_latency_ns =
      bev_ready_latency_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto bev_ready_latency_ns_max =
      bev_ready_latency_ns_max_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto bev_stage_latency_samples =
      bev_stage_latency_samples_interval_.exchange(
      0U, std::memory_order_relaxed);
    const auto bev_stage_latency_ns =
      bev_stage_latency_ns_interval_.exchange(0U, std::memory_order_relaxed);
    const auto bev_stage_latency_ns_max =
      bev_stage_latency_ns_max_interval_.exchange(
      0U, std::memory_order_relaxed);

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
    const double average_lane_process_ms =
      lane_process_samples > 0U ?
      static_cast<double>(lane_process_ns) /
      static_cast<double>(lane_process_samples) / 1.0e6 :
      0.0;
    const double average_stabilized_latency_ms =
      stabilized_latency_samples > 0U ?
      static_cast<double>(stabilized_latency_ns) /
      static_cast<double>(stabilized_latency_samples) / 1.0e6 :
      0.0;
    const double average_bev_ready_latency_ms =
      bev_ready_latency_samples > 0U ?
      static_cast<double>(bev_ready_latency_ns) /
      static_cast<double>(bev_ready_latency_samples) / 1.0e6 :
      0.0;
    const double average_bev_stage_latency_ms =
      bev_stage_latency_samples > 0U ?
      static_cast<double>(bev_stage_latency_ns) /
      static_cast<double>(bev_stage_latency_samples) / 1.0e6 :
      0.0;
    if (performance_measurement_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "[PERF][PIPELINE] stabilized_fps=%.1f bev_ready_fps=%.1f "
        "processed_fps=%.1f "
        "latency_ms(depthai_to_bev_input_avg/max=%.2f/%.2f,"
        "depthai_to_bev_ready_avg/max=%.2f/%.2f,"
        "bev_input_to_ready_avg/max=%.3f/%.3f) "
        "bev_compute_ms(avg/max)=%.3f/%.3f skipped=%llu "
        "errors(invalid/process/publish)=%llu/%llu/%llu",
        static_cast<double>(accepted) / elapsed_sec,
        static_cast<double>(published) / elapsed_sec,
        static_cast<double>(processed) / elapsed_sec,
        average_stabilized_latency_ms,
        static_cast<double>(stabilized_latency_ns_max) / 1.0e6,
        average_bev_ready_latency_ms,
        static_cast<double>(bev_ready_latency_ns_max) / 1.0e6,
        average_bev_stage_latency_ms,
        static_cast<double>(bev_stage_latency_ns_max) / 1.0e6,
        average_process_ms,
        static_cast<double>(process_ns_max) / 1.0e6,
        static_cast<unsigned long long>(skipped),
        static_cast<unsigned long long>(
          invalid_total_.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
          processing_error_total_.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
          publish_error_total_.load(std::memory_order_relaxed)));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "\nBEV status: input=%.1fHz (%llu total), processed=%.1fHz "
        "(%llu total, skipped=%llu/%llu interval/total), "
        "ROS=%.1fHz, preview=%.1fHz, "
        "compute=%.3f/%.3fms avg/max, latest_age=%.2fms, "
        "extrinsics=startup_measured, fixed_lut=true, "
        "(height=%.3fm,roll=%.2f,pitch_down=%.2fdeg), "
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
        camera_model_.position_vehicle_m[2],
        applied_roll_deg_.load(std::memory_order_relaxed),
        applied_pitch_down_deg_.load(std::memory_order_relaxed),
        static_cast<unsigned long long>(
          invalid_total_.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
          processing_error_total_.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
          publish_error_total_.load(std::memory_order_relaxed)));
    }

    if (lane_reconstruction_enabled_) {
      const int centerline_point_count =
        latest_lane_centerline_points_.load(std::memory_order_relaxed);
      const char * centerline_source = centerline_point_count <= 0 ?
        "none" :
        (latest_lane_centerline_single_boundary_.load(
          std::memory_order_relaxed) ? "single" : "pair");
      RCLCPP_INFO(
        get_logger(),
        "BEV lane: valid/invalid=%.1f/%.1fHz "
        "(%llu/%llu total), measured/centerline=%d/%d, hold=%s, "
        "center_source=%s, normal_offset_trimmed=%s, "
        "width=%.3fm, reconstructed_to=%.2fm, "
        "lane_compute_ms(avg/max)=%.3f/%.3f, output=%s",
        static_cast<double>(lane_valid) / elapsed_sec,
        static_cast<double>(lane_invalid) / elapsed_sec,
        static_cast<unsigned long long>(
          lane_valid_total_.load(std::memory_order_relaxed)),
        static_cast<unsigned long long>(
          lane_invalid_total_.load(std::memory_order_relaxed)),
        latest_lane_points_.load(std::memory_order_relaxed),
        centerline_point_count,
        latest_lane_temporal_hold_.load(std::memory_order_relaxed) ?
        "yes" : "no",
        centerline_source,
        latest_lane_centerline_truncated_.load(std::memory_order_relaxed) ?
        "yes" : "no",
        static_cast<double>(
          latest_lane_width_mm_.load(std::memory_order_relaxed)) / 1000.0,
        static_cast<double>(latest_lane_reconstructed_maximum_x_mm_.load(
          std::memory_order_relaxed)) / 1000.0,
        average_lane_process_ms,
        static_cast<double>(lane_process_ns_max) / 1.0e6,
        lane_output_topic_.c_str());
    }

    if (
      received_total_.load(std::memory_order_relaxed) == 0U &&
      std::chrono::duration<double>(now - node_started_at_).count() >=
      startup_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "No fused camera input received on %s. Check that camera_driver "
        "fused_bev_output_enabled is true and its crop is %dx%d.",
        input_topic_.c_str(),
        input_crop_width_,
        input_crop_height_);
    }
  }

  int configuration_version_{0};
  bool performance_measurement_enabled_{false};
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  std::string startup_ground_reference_topic_;
  std::string startup_ground_reference_frame_id_;
  double expected_input_fps_{80.0};
  double input_bottom_fraction_{0.70};
  int input_crop_width_{1280};
  int input_crop_height_{504};
  int input_crop_top_{216};
  bool publish_enabled_{true};
  double publish_max_fps_{0.0};
  bool preview_enabled_{true};
  double preview_max_fps_{60.0};
  std::string preview_window_name_;
  int preview_max_width_{1280};
  int preview_max_height_{720};
  bool lane_reconstruction_enabled_{true};
  std::string lane_output_topic_{"/camera/image_bev_lane"};
  bool lane_preview_enabled_{true};
  bool lane_preview_result_only_enabled_{false};
  bool lane_preview_sliding_windows_enabled_{false};
  double preview_x_origin_m_{0.02};
  double lane_preview_overlay_alpha_{0.8};
  BevLaneReconstructorConfig lane_reconstructor_config_{};
  double status_log_interval_sec_{5.0};
  double startup_timeout_sec_{12.0};
  double stabilization_settle_sec_{5.5};
  double camera_yaw_deg_{0.0};
  OakStartupMeasurementConfig startup_measurement_config_{};

  RectifiedCameraModel camera_model_{};
  BevConfig bev_config_{};
  RemapLut bev_lut_{};
  bool fused_coverage_reported_{false};
  double startup_roll_deg_{0.0};
  double startup_pitch_down_deg_{14.0};
  std::atomic<double> valid_lut_percent_{0.0};
  std::atomic<double> applied_roll_deg_{0.0};
  std::atomic<double> applied_pitch_down_deg_{14.0};
  std::shared_ptr<CudaBevProcessor> gpu_processor_;
  std::unique_ptr<BevLaneReconstructor> lane_reconstructor_;

  rclcpp::Subscription<camera_driver::msg::BevInput>::SharedPtr
    input_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr output_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lane_output_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    startup_ground_reference_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex input_mutex_;
  std::condition_variable input_cv_;
  camera_driver::msg::BevInput::ConstSharedPtr latest_input_;
  SteadyClock::time_point latest_input_received_at_;
  std::uint64_t input_generation_{0U};
  bool first_camera_input_seen_{false};
  SteadyClock::time_point first_camera_input_at_;

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
  std::atomic<std::uint64_t> stabilization_settle_total_{0U};
  std::atomic<std::uint64_t> processing_error_total_{0U};
  std::atomic<std::uint64_t> publish_error_total_{0U};
  std::atomic<std::uint64_t> lane_valid_total_{0U};
  std::atomic<std::uint64_t> lane_invalid_total_{0U};
  std::atomic<int> latest_lane_points_{0};
  std::atomic<int> latest_lane_centerline_points_{0};
  std::atomic<bool> latest_lane_centerline_truncated_{false};
  std::atomic<bool> latest_lane_centerline_single_boundary_{false};
  std::atomic<bool> latest_lane_temporal_hold_{false};
  std::atomic<int> latest_lane_width_mm_{0};
  std::atomic<int> latest_lane_reconstructed_maximum_x_mm_{0};
  std::atomic<std::uint64_t> received_interval_{0U};
  std::atomic<std::uint64_t> accepted_interval_{0U};
  std::atomic<std::uint64_t> processed_interval_{0U};
  std::atomic<std::uint64_t> skipped_interval_{0U};
  std::atomic<std::uint64_t> published_interval_{0U};
  std::atomic<std::uint64_t> previewed_interval_{0U};
  std::atomic<std::uint64_t> lane_valid_interval_{0U};
  std::atomic<std::uint64_t> lane_invalid_interval_{0U};
  std::atomic<std::uint64_t> lane_process_samples_interval_{0U};
  std::atomic<std::uint64_t> lane_process_ns_interval_{0U};
  std::atomic<std::uint64_t> lane_process_ns_max_interval_{0U};
  std::atomic<std::uint64_t> process_ns_interval_{0U};
  std::atomic<std::uint64_t> process_ns_max_interval_{0U};
  std::atomic<std::uint64_t> stabilized_latency_samples_interval_{0U};
  std::atomic<std::uint64_t> stabilized_latency_ns_interval_{0U};
  std::atomic<std::uint64_t> stabilized_latency_ns_max_interval_{0U};
  std::atomic<std::uint64_t> bev_ready_latency_samples_interval_{0U};
  std::atomic<std::uint64_t> bev_ready_latency_ns_interval_{0U};
  std::atomic<std::uint64_t> bev_ready_latency_ns_max_interval_{0U};
  std::atomic<std::uint64_t> bev_stage_latency_samples_interval_{0U};
  std::atomic<std::uint64_t> bev_stage_latency_ns_interval_{0U};
  std::atomic<std::uint64_t> bev_stage_latency_ns_max_interval_{0U};
};

}  // namespace bev_processor

RCLCPP_COMPONENTS_REGISTER_NODE(bev_processor::BevProcessorNode)
