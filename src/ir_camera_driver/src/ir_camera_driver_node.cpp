#include "ir_camera_driver/cuda_center_reprojector.hpp"
#include "ir_camera_driver/ir_bev_projector.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"
#include "camera_driver/imu_image_stabilizer.hpp"
#include "oak_startup/oak_startup_measurement.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ir_camera_driver
{

using namespace std::chrono_literals;

namespace
{

constexpr std::uint32_t kOv9282FullWidth = 1280U;
constexpr std::uint32_t kOv9282FullHeight = 800U;
constexpr double kOv9282MaximumFullResolutionFps = 129.0;
constexpr double kRvc2MaximumFastStereoFullResolutionFps = 50.0;

std::string uppercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {
      return static_cast<char>(std::toupper(character));
    });
  return value;
}

bool graphicalDisplayAvailable()
{
#if defined(__linux__)
  return std::getenv("DISPLAY") != nullptr ||
         std::getenv("WAYLAND_DISPLAY") != nullptr;
#else
  return true;
#endif
}

double timestampSeconds(const std::chrono::steady_clock::time_point & timestamp)
{
  return std::chrono::duration<double>(timestamp.time_since_epoch()).count();
}

template<typename Matrix>
cv::Matx33d calibrationRotation(
  const Matrix & matrix, const char * name)
{
  if (matrix.size() < 3U || matrix[0].size() < 3U ||
    matrix[1].size() < 3U || matrix[2].size() < 3U)
  {
    throw std::runtime_error(std::string(name) + ": missing rotation");
  }
  cv::Matx33d rotation;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      rotation(row, col) = matrix[row][col];
    }
  }
  if (!cv::checkRange(cv::Mat(rotation)) ||
    std::abs(cv::determinant(cv::Mat(rotation)) - 1.0) > 0.02 ||
    cv::norm(cv::Mat(rotation * rotation.t() - cv::Matx33d::eye())) > 0.02)
  {
    throw std::runtime_error(std::string(name) + ": invalid rotation");
  }
  return rotation;
}

const char * usbSpeedName(const dai::UsbSpeed speed)
{
  switch (speed) {
    case dai::UsbSpeed::LOW:
      return "LOW";
    case dai::UsbSpeed::FULL:
      return "FULL";
    case dai::UsbSpeed::HIGH:
      return "HIGH";
    case dai::UsbSpeed::SUPER:
      return "SUPER";
    case dai::UsbSpeed::SUPER_PLUS:
      return "SUPER_PLUS";
    case dai::UsbSpeed::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

}  // namespace

class IrCameraDriverNode : public rclcpp::Node
{
public:
  IrCameraDriverNode()
  : Node("ir_camera_driver")
  {
    readParameters();
    if (!graphicalDisplayAvailable()) {
      throw std::runtime_error(
              "IR preview requires DISPLAY or WAYLAND_DISPLAY");
    }

    try {
      measureBevMount();
      startOak();
      started_at_ = std::chrono::steady_clock::now();
      last_status_at_ = started_at_;
      status_timer_ = create_wall_timer(
        std::chrono::duration<double>(status_log_interval_sec_),
        std::bind(&IrCameraDriverNode::reportStatus, this));
      if (imu_stabilization_enabled_) {
        imu_thread_ = std::thread(&IrCameraDriverNode::imuLoop, this);
      }
      capture_thread_ = std::thread(&IrCameraDriverNode::captureLoop, this);
      preview_thread_ = std::thread(&IrCameraDriverNode::previewLoop, this);
    } catch (...) {
      stop();
      throw;
    }
  }

  ~IrCameraDriverNode() override
  {
    stop();
  }

private:
  enum class SelectedCamera
  {
    LEFT,
    RIGHT
  };

  enum class EditableField
  {
    NONE = -1,
    FLOOD = 0,
    EXPOSURE = 1,
    ISO = 2
  };

  static constexpr int kControlPanelWidth = 700;
  static constexpr int kControlPanelHeight = 275;

  static const std::array<cv::Rect, 3> & inputRects()
  {
    static const std::array<cv::Rect, 3> rectangles{
      cv::Rect(235, 48, 220, 38),
      cv::Rect(235, 103, 220, 38),
      cv::Rect(235, 158, 220, 38)};
    return rectangles;
  }

  static const cv::Rect & applyRect()
  {
    static const cv::Rect rectangle(485, 48, 175, 38);
    return rectangle;
  }

  static const cv::Rect & autoExposureRect()
  {
    static const cv::Rect rectangle(485, 103, 175, 38);
    return rectangle;
  }

  static const cv::Rect & irToggleRect()
  {
    static const cv::Rect rectangle(485, 158, 175, 38);
    return rectangle;
  }

  struct StereoSnapshot
  {
    std::shared_ptr<dai::ImgFrame> left;
    std::shared_ptr<dai::ImgFrame> right;
    std::shared_ptr<dai::ImgFrame> disparity;
    std::uint64_t generation;
    std::int64_t sequence;
  };

  struct SingleSnapshot
  {
    std::shared_ptr<dai::ImgFrame> frame;
    std::uint64_t generation;
    std::int64_t sequence;
  };

  static SelectedCamera parseSelectedCamera(const std::string & value)
  {
    const auto normalized = uppercase(value);
    if (normalized == "LEFT" || normalized == "CAM_B") {
      return SelectedCamera::LEFT;
    }
    if (normalized == "RIGHT" || normalized == "CAM_C") {
      return SelectedCamera::RIGHT;
    }
    throw std::invalid_argument(
            "selected_camera must be LEFT/CAM_B or RIGHT/CAM_C");
  }

  dai::CameraBoardSocket selectedSocket() const
  {
    return selected_camera_ == SelectedCamera::LEFT ?
           dai::CameraBoardSocket::CAM_B : dai::CameraBoardSocket::CAM_C;
  }

  const char * selectedCameraLabel() const
  {
    return selected_camera_ == SelectedCamera::LEFT ?
           "LEFT/CAM_B" : "RIGHT/CAM_C";
  }

  double requestedFps() const
  {
    return reprojection_enabled_ ? reprojection_fps_ : single_camera_fps_;
  }

  void readParameters()
  {
    reprojection_enabled_ =
      declare_parameter<bool>("reprojection_enabled", true);
    selected_camera_name_ =
      declare_parameter<std::string>("selected_camera", "LEFT");
    virtual_camera_position_ratio_ = declare_parameter<double>(
      "virtual_camera_position_ratio", 0.5);

    width_ = declare_parameter<int>("width", 1280);
    height_ = declare_parameter<int>("height", 800);
    reprojection_fps_ =
      declare_parameter<double>("reprojection_fps", 30.0);
    single_camera_fps_ =
      declare_parameter<double>("single_camera_fps", 30.0);
    undistort_single_camera_ =
      declare_parameter<bool>("undistort_single_camera", true);
    sync_threshold_ms_ =
      declare_parameter<double>("sync_threshold_ms", 2.0);

    ir_enabled_at_start_ = declare_parameter<bool>("ir_enabled", true);
    ir_dot_projector_intensity_ = declare_parameter<double>(
      "ir_dot_projector_intensity", 1.0);
    ir_flood_light_intensity_ = declare_parameter<double>(
      "ir_flood_light_intensity", 0.0);

    manual_exposure_enabled_ =
      declare_parameter<bool>("manual_exposure_enabled", true);
    manual_exposure_us_ =
      declare_parameter<int>("manual_exposure_us", 5000);
    manual_sensitivity_iso_ =
      declare_parameter<int>("manual_sensitivity_iso", 800);

    preview_window_name_ = declare_parameter<std::string>(
      "preview_window_name", "OAK IR center preview");
    bev_preview_window_name_ = declare_parameter<std::string>(
      "bev_preview_window_name", "OAK IR BEV preview");
    controls_window_name_ = declare_parameter<std::string>(
      "controls_window_name", "OAK IR live controls");
    preview_max_fps_ = declare_parameter<double>("preview_max_fps", 30.0);
    preview_max_width_ =
      declare_parameter<int>("preview_max_width", 1280);
    preview_max_height_ =
      declare_parameter<int>("preview_max_height", 800);
    bev_preview_scale_ = declare_parameter<double>("bev_preview_scale", 2.0);
    capture_directory_ =
      declare_parameter<std::string>("capture_directory", ".");
    status_log_interval_sec_ =
      declare_parameter<double>("status_log_interval_sec", 1.0);

    bev_config_.camera_x_m = declare_parameter<double>("bev.camera_x_m", -0.16);
    bev_config_.camera_y_m = declare_parameter<double>("bev.camera_y_m", 0.0);
    bev_config_.camera_height_m = declare_parameter<double>("bev.camera_height_m", 0.20);
    bev_config_.camera_roll_deg = declare_parameter<double>("bev.camera_roll_deg", 0.0);
    bev_config_.camera_pitch_down_deg = declare_parameter<double>(
      "bev.camera_pitch_down_deg", 14.0);
    bev_config_.camera_yaw_deg = declare_parameter<double>("bev.camera_yaw_deg", 0.0);
    bev_config_.x_min_m = declare_parameter<double>("bev.x_min_m", 0.0);
    bev_config_.x_max_m = declare_parameter<double>("bev.x_max_m", 3.0);
    bev_config_.y_min_m = declare_parameter<double>("bev.y_min_m", -0.6);
    bev_config_.y_max_m = declare_parameter<double>("bev.y_max_m", 0.6);
    bev_config_.meter_per_pixel = declare_parameter<double>("bev.meter_per_pixel", 0.01);
    bev_config_.output_width = declare_parameter<int>("bev.output_width", 120);
    bev_config_.output_height = declare_parameter<int>("bev.output_height", 300);

    selected_camera_ = parseSelectedCamera(selected_camera_name_);
    maximum_manual_exposure_us_ = static_cast<int>(std::floor(1.0e6 / requestedFps()));
    if (
      width_ != static_cast<int>(kOv9282FullWidth) ||
      height_ != static_cast<int>(kOv9282FullHeight) ||
      !std::isfinite(reprojection_fps_) || reprojection_fps_ <= 0.0 ||
      reprojection_fps_ > kRvc2MaximumFastStereoFullResolutionFps ||
      !std::isfinite(single_camera_fps_) || single_camera_fps_ <= 0.0 ||
      single_camera_fps_ > kOv9282MaximumFullResolutionFps ||
      !std::isfinite(virtual_camera_position_ratio_) ||
      virtual_camera_position_ratio_ < 0.0 ||
      virtual_camera_position_ratio_ > 1.0 ||
      !std::isfinite(sync_threshold_ms_) || sync_threshold_ms_ <= 0.0 ||
      !std::isfinite(ir_dot_projector_intensity_) ||
      ir_dot_projector_intensity_ < 0.0 ||
      ir_dot_projector_intensity_ > 1.0 ||
      !std::isfinite(ir_flood_light_intensity_) ||
      ir_flood_light_intensity_ < 0.0 ||
      ir_flood_light_intensity_ > 1.0 ||
      (manual_exposure_enabled_ &&
      (manual_exposure_us_ < 10 ||
      manual_exposure_us_ > maximum_manual_exposure_us_ ||
      manual_sensitivity_iso_ < 100 || manual_sensitivity_iso_ > 1600)) ||
      preview_window_name_.empty() || bev_preview_window_name_.empty() ||
      controls_window_name_.empty() ||
      !std::isfinite(preview_max_fps_) || preview_max_fps_ <= 0.0 ||
      preview_max_fps_ > 30.0 ||
      preview_max_width_ < 0 || preview_max_height_ < 0 ||
      !std::isfinite(bev_preview_scale_) || bev_preview_scale_ <= 0.0 ||
      bev_preview_scale_ > 10.0 ||
      capture_directory_.empty() ||
      !std::isfinite(status_log_interval_sec_) ||
      status_log_interval_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "invalid ir_camera_driver parameter configuration");
    }

    if (
      ir_enabled_at_start_ && ir_dot_projector_intensity_ <= 0.0 &&
      ir_flood_light_intensity_ <= 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "IR is enabled but both configured emitter intensities are zero.");
    }

    readMeasurementParameters();
    flood_input_ = formatFlood(ir_flood_light_intensity_);
    exposure_input_ = std::to_string(manual_exposure_us_);
    iso_input_ = std::to_string(manual_sensitivity_iso_);
    manual_exposure_active_.store(manual_exposure_enabled_, std::memory_order_relaxed);
  }

  void readMeasurementParameters()
  {
    device_id_ = declare_parameter<std::string>("device_id", "");
    startup_measurement_enabled_ = declare_parameter<bool>(
      "bev.startup_measurement_enabled", true);
    imu_stabilization_enabled_ = declare_parameter<bool>(
      "imu_stabilization_enabled", true);
    imu_calibration_timeout_sec_ = declare_parameter<double>(
      "imu_calibration_timeout_sec", 30.0);
    auto & measurement = startup_measurement_config_;
    measurement.device_id = device_id_;
    measurement.stereo_fps = declare_parameter<double>(
      "measurement_stereo_fps", measurement.stereo_fps);
    measurement.stereo_width = declare_parameter<int>(
      "measurement_stereo_width", measurement.stereo_width);
    measurement.stereo_height = declare_parameter<int>(
      "measurement_stereo_height", measurement.stereo_height);
    measurement.depth_queue_size = declare_parameter<int>(
      "measurement_depth_queue_size", measurement.depth_queue_size);
    measurement.stereo_subpixel_fractional_bits = declare_parameter<int>(
      "measurement_stereo_subpixel_fractional_bits", measurement.stereo_subpixel_fractional_bits);
    measurement.stereo_left_right_check_threshold = declare_parameter<int>(
      "measurement_stereo_left_right_check_threshold", measurement.stereo_left_right_check_threshold);
    measurement.stereo_confidence_threshold = declare_parameter<int>(
      "measurement_stereo_confidence_threshold", measurement.stereo_confidence_threshold);
    measurement.stereo_disparity_shift = declare_parameter<int>(
      "measurement_stereo_disparity_shift", measurement.stereo_disparity_shift);
    measurement.imu_rate_hz = declare_parameter<double>(
      "measurement_imu_rate_hz", measurement.imu_rate_hz);
    measurement.imu_queue_size = declare_parameter<int>(
      "measurement_imu_queue_size", measurement.imu_queue_size);
    measurement.imu_max_batch_reports = declare_parameter<int>(
      "measurement_imu_max_batch_reports", measurement.imu_max_batch_reports);
    measurement.maximum_imu_pair_skew_sec = declare_parameter<double>(
      "measurement_maximum_imu_pair_skew_sec", measurement.maximum_imu_pair_skew_sec);
    measurement.warmup_sec = declare_parameter<double>(
      "measurement_warmup_sec", measurement.warmup_sec);
    measurement.ir_dot_projector_intensity = declare_parameter<double>(
      "measurement_ir_dot_projector_intensity", measurement.ir_dot_projector_intensity);
    measurement.manual_camera_height_enabled = declare_parameter<bool>(
      "measurement_manual_camera_height_enabled", measurement.manual_camera_height_enabled);
    measurement.manual_camera_height_m = declare_parameter<double>(
      "measurement_manual_camera_height_m", measurement.manual_camera_height_m);
    measurement.roi_width = declare_parameter<int>(
      "measurement_roi_width", measurement.roi_width);
    measurement.roi_height = declare_parameter<int>(
      "measurement_roi_height", measurement.roi_height);
    measurement.roi_vertical_offset_px = declare_parameter<int>(
      "measurement_roi_vertical_offset_px", measurement.roi_vertical_offset_px);
    measurement.roi_preview_enabled = declare_parameter<bool>(
      "measurement_roi_preview_enabled", measurement.roi_preview_enabled);
    measurement.point_sample_step = declare_parameter<int>(
      "measurement_point_sample_step", measurement.point_sample_step);
    measurement.minimum_valid_points = declare_parameter<int>(
      "measurement_minimum_valid_points", measurement.minimum_valid_points);
    measurement.minimum_depth_m = declare_parameter<double>(
      "measurement_minimum_depth_m", measurement.minimum_depth_m);
    measurement.maximum_depth_m = declare_parameter<double>(
      "measurement_maximum_depth_m", measurement.maximum_depth_m);
    measurement.minimum_height_m = declare_parameter<double>(
      "measurement_minimum_height_m", measurement.minimum_height_m);
    measurement.maximum_height_m = declare_parameter<double>(
      "measurement_maximum_height_m", measurement.maximum_height_m);
    measurement.plane_ransac_iterations = declare_parameter<int>(
      "measurement_plane_ransac_iterations", measurement.plane_ransac_iterations);
    measurement.plane_inlier_threshold_m = declare_parameter<double>(
      "measurement_plane_inlier_threshold_m", measurement.plane_inlier_threshold_m);
    measurement.plane_minimum_inliers = declare_parameter<int>(
      "measurement_plane_minimum_inliers", measurement.plane_minimum_inliers);
    measurement.plane_minimum_inlier_ratio = declare_parameter<double>(
      "measurement_plane_minimum_inlier_ratio", measurement.plane_minimum_inlier_ratio);
    measurement.plane_maximum_residual_mad_m = declare_parameter<double>(
      "measurement_plane_maximum_residual_mad_m", measurement.plane_maximum_residual_mad_m);
    measurement.plane_maximum_imu_difference_deg = declare_parameter<double>(
      "measurement_plane_maximum_imu_difference_deg", measurement.plane_maximum_imu_difference_deg);
    measurement.imu_roll_bias_deg = declare_parameter<double>(
      "measurement_imu_roll_bias_deg", measurement.imu_roll_bias_deg);
    measurement.imu_pitch_bias_deg = declare_parameter<double>(
      "measurement_imu_pitch_bias_deg", measurement.imu_pitch_bias_deg);
    measurement.imu_sample_count = declare_parameter<int>(
      "measurement_imu_sample_count", measurement.imu_sample_count);
    measurement.imu_max_direction_rms_deg = declare_parameter<double>(
      "measurement_imu_max_direction_rms_deg", measurement.imu_max_direction_rms_deg);
    measurement.imu_accel_min_mps2 = declare_parameter<double>(
      "measurement_imu_accel_min_mps2", measurement.imu_accel_min_mps2);
    measurement.imu_accel_max_mps2 = declare_parameter<double>(
      "measurement_imu_accel_max_mps2", measurement.imu_accel_max_mps2);
    measurement.imu_gyroscope_mean_maximum_degps = declare_parameter<double>(
      "measurement_imu_gyroscope_mean_maximum_degps", measurement.imu_gyroscope_mean_maximum_degps);
    measurement.imu_gyroscope_stddev_maximum_degps = declare_parameter<double>(
      "measurement_imu_gyroscope_stddev_maximum_degps", measurement.imu_gyroscope_stddev_maximum_degps);
    measurement.stable_plane_frame_count = declare_parameter<int>(
      "measurement_stable_plane_frame_count", measurement.stable_plane_frame_count);
    measurement.maximum_height_stddev_m = declare_parameter<double>(
      "measurement_maximum_height_stddev_m", measurement.maximum_height_stddev_m);
    measurement.maximum_plane_normal_rms_deg = declare_parameter<double>(
      "measurement_maximum_plane_normal_rms_deg", measurement.maximum_plane_normal_rms_deg);
    measurement.timeout_sec = declare_parameter<double>(
      "measurement_timeout_sec", measurement.timeout_sec);
    measurement.attitude_source = oak_startup::parseStartupAttitudeSource(
      declare_parameter<std::string>("measurement_attitude_source", "depth"));
    auto & imu = imu_stabilizer_config_;
    imu.external_reference_required = true;
    imu.startup_discard_duration_sec = declare_parameter<double>(
      "imu_stabilization_startup_discard_duration_sec", imu.startup_discard_duration_sec);
    imu.reference_calibration_duration_sec = declare_parameter<double>(
      "imu_stabilization_reference_calibration_duration_sec", imu.reference_calibration_duration_sec);
    imu.calibration_maximum_angular_speed_degps = declare_parameter<double>(
      "imu_stabilization_calibration_maximum_angular_speed_degps", imu.calibration_maximum_angular_speed_degps);
    imu.maximum_correction_deg = declare_parameter<double>(
      "imu_stabilization_maximum_correction_deg", imu.maximum_correction_deg);
    imu.maximum_frame_imu_wait_sec = declare_parameter<double>(
      "imu_stabilization_maximum_frame_imu_wait_sec", imu.maximum_frame_imu_wait_sec);
    imu.maximum_frame_imu_age_sec = declare_parameter<double>(
      "imu_stabilization_maximum_frame_imu_age_sec", imu.maximum_frame_imu_age_sec);
    imu.maximum_frame_imu_prediction_sec = declare_parameter<double>(
      "imu_stabilization_maximum_frame_imu_prediction_sec", imu.maximum_frame_imu_prediction_sec);
    if (!std::isfinite(imu_calibration_timeout_sec_) ||
      imu_calibration_timeout_sec_ <=
      imu.startup_discard_duration_sec + imu.reference_calibration_duration_sec ||
      !std::isfinite(measurement.imu_rate_hz) || measurement.imu_rate_hz <= 0.0 ||
      measurement.imu_rate_hz > 1000.0 || measurement.imu_queue_size <= 0 ||
      measurement.imu_max_batch_reports <= 1 ||
      measurement.imu_max_batch_reports > measurement.imu_queue_size ||
      !std::isfinite(measurement.maximum_imu_pair_skew_sec) ||
      measurement.maximum_imu_pair_skew_sec <= 0.0)
    {
      throw std::invalid_argument("invalid IR IMU timing/queue configuration");
    }
    if (!reprojection_enabled_ && !undistort_single_camera_) {
      throw std::invalid_argument(
              "metric IR BEV requires undistort_single_camera=true");
    }
    // Validate static geometry even when automatic measurement is requested.
    IrBevProjector geometry_check(bev_config_);
    if (imu_stabilization_enabled_) {
      imu_stabilizer_ =
        std::make_unique<camera_driver::ImuImageStabilizer>(imu);
    }
  }

  void measureBevMount()
  {
    if (startup_measurement_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "IR BEV startup: keep the vehicle stationary on flat ground. "
        "height=%s, attitude=%s, measurement dot=%.2f (runtime dot=%.2f).",
        startup_measurement_config_.manual_camera_height_enabled ? "manual" : "stereo",
        startup_measurement_config_.manual_camera_height_enabled ? "imu" :
        oak_startup::startupAttitudeSourceName(startup_measurement_config_.attitude_source),
        startup_measurement_config_.manual_camera_height_enabled ? 0.0 :
        startup_measurement_config_.ir_dot_projector_intensity,
        ir_dot_projector_intensity_);
      const auto measurement = oak_startup::measureOakStartupExtrinsics(
        startup_measurement_config_, []() {return !rclcpp::ok();});
      device_id_ = measurement.device_id;
      bev_config_.camera_height_m = measurement.height_m;
      bev_config_.camera_roll_deg = measurement.roll_deg;
      bev_config_.camera_pitch_down_deg = measurement.pitch_down_deg;
      RCLCPP_INFO(
        get_logger(),
        "IR_BEV_STARTUP: CAM_A height=%.4fm roll=%.3fdeg pitch_down=%.3fdeg "
        "source=%s/%s, height_stddev=%.4fm plane_RMS=%.3fdeg "
        "inliers=%.1f%% residual_MAD=%.4fm IMU_RMS=%.3fdeg",
        measurement.height_m, measurement.roll_deg, measurement.pitch_down_deg,
        measurement.height_source.c_str(), measurement.attitude_source.c_str(),
        measurement.height_stddev_m, measurement.plane_normal_rms_deg,
        100.0 * measurement.plane_inlier_ratio, measurement.plane_residual_mad_m,
        measurement.imu_direction_rms_deg);
    } else {
      RCLCPP_WARN(get_logger(), "IR BEV uses explicit fixed CAM_A mounting parameters.");
    }
    if (imu_stabilizer_ && !imu_stabilizer_->setExternalReferenceUpCamera(
        oak_startup::attitudeUpVector(
          bev_config_.camera_roll_deg, bev_config_.camera_pitch_down_deg)))
    {
      throw std::runtime_error("could not set the measured IR BEV ground reference");
    }
    bev_projector_ = std::make_unique<IrBevProjector>(bev_config_);
    right_bev_projector_ = std::make_unique<IrBevProjector>(bev_config_);
  }

  void configureExposure(const std::shared_ptr<dai::node::Camera> & camera)
  {
    if (manual_exposure_enabled_) {
      camera->initialControl.setManualExposure(
        static_cast<std::uint32_t>(manual_exposure_us_),
        static_cast<std::uint32_t>(manual_sensitivity_iso_));
    }
  }

  void startOak()
  {
    if (!rclcpp::ok()) {
      throw std::runtime_error("IR camera startup cancelled");
    }
    device_ = device_id_.empty() ?
      std::make_shared<dai::Device>(dai::UsbSpeed::SUPER) :
      std::make_shared<dai::Device>(dai::DeviceInfo(device_id_), dai::UsbSpeed::SUPER);
    pipeline_ = std::make_unique<dai::Pipeline>(device_);
    // DepthAI 3.6 enables startup auto-calibration by default. This preview
    // package must never mutate the device EEPROM as a side effect.
    pipeline_->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
    pipeline_->setXLinkChunkSize(0);

    dai::Node::Output * host_output = nullptr;
    if (reprojection_enabled_) {
      auto left_camera = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_B,
        std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
        static_cast<float>(reprojection_fps_));
      auto right_camera = pipeline_->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_C,
        std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
        static_cast<float>(reprojection_fps_));
      configureExposure(left_camera);
      configureExposure(right_camera);
      camera_control_queues_.push_back(left_camera->inputControl.createInputQueue());
      camera_control_queues_.push_back(right_camera->inputControl.createInputQueue());

      auto * left_output = left_camera->requestOutput(
        std::make_pair(
          static_cast<std::uint32_t>(width_),
          static_cast<std::uint32_t>(height_)),
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        static_cast<float>(reprojection_fps_));
      auto * right_output = right_camera->requestOutput(
        std::make_pair(
          static_cast<std::uint32_t>(width_),
          static_cast<std::uint32_t>(height_)),
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        static_cast<float>(reprojection_fps_));

      auto stereo = pipeline_->create<dai::node::StereoDepth>();
      stereo->build(
        *left_output,
        *right_output,
        dai::node::StereoDepth::PresetMode::FAST_DENSITY);
      stereo->setRectification(true);
      stereo->enableDistortionCorrection(true);
      stereo->setOutputSize(width_, height_);
      stereo->setOutputKeepAspectRatio(true);
      stereo->setDepthAlign(
        dai::StereoDepthConfig::AlgorithmControl::DepthAlign::CENTER);
      stereo->initialConfig->algorithmControl.centerAlignmentShiftFactor =
        static_cast<float>(virtual_camera_position_ratio_);
      // CENTER alignment requires LR-check. Keep subpixel and post filters
      // disabled so the 800P disparity path remains as light as possible.
      stereo->setSubpixel(false);
      stereo->setExtendedDisparity(false);
      stereo->setLeftRightCheck(true);
      stereo->initialConfig->postProcessing.median =
        dai::node::StereoDepth::MedianFilter::MEDIAN_OFF;
      stereo->initialConfig->postProcessing.holeFilling.enable = false;
      stereo->initialConfig->postProcessing.adaptiveMedianFilter.enable =
        false;
      stereo->initialConfig->postProcessing.spatialFilter.enable = false;
      stereo->initialConfig->postProcessing.temporalFilter.enable = false;
      stereo->initialConfig->postProcessing.speckleFilter.enable = false;
      stereo->setRectifyEdgeFillColor(0);
      stereo->setFrameSync(true);

      auto sync = pipeline_->create<dai::node::Sync>();
      sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double, std::milli>(sync_threshold_ms_)));
      sync->setSyncAttempts(-1);
      stereo->rectifiedLeft.link(sync->inputs["left"]);
      stereo->rectifiedRight.link(sync->inputs["right"]);
      stereo->disparity.link(sync->inputs["disparity"]);
      host_output = &sync->out;
      center_reprojector_ = std::make_unique<CudaCenterReprojector>();
    } else {
      auto camera = pipeline_->create<dai::node::Camera>()->build(
        selectedSocket(),
        std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
        static_cast<float>(single_camera_fps_));
      configureExposure(camera);
      camera_control_queues_.push_back(camera->inputControl.createInputQueue());
      host_output = camera->requestOutput(
        std::make_pair(
          static_cast<std::uint32_t>(width_),
          static_cast<std::uint32_t>(height_)),
        dai::ImgFrame::Type::GRAY8,
        dai::ImgResizeMode::CROP,
        static_cast<float>(single_camera_fps_),
        undistort_single_camera_);
    }

    if (imu_stabilization_enabled_) {
      if (device_->getConnectedIMU().empty()) {
        throw std::runtime_error("IR BEV stabilization requires the OAK IMU");
      }
      const auto calibration = device_->getCalibration();
      const auto imu_to_rgb = calibrationRotation(
        calibration.getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_A, false),
        "IMU-to-CAM_A");
      const auto imu_to_output = calibrationRotation(
        calibration.getEepromData().imuExtrinsics.rotationMatrix,
        "calibrated IMU output");
      calibrated_imu_to_rgb_ = imu_to_rgb * imu_to_output.t();
      auto imu = pipeline_->create<dai::node::IMU>();
      imu->enableIMUSensor(
        {dai::IMUSensor::ACCELEROMETER_CALIBRATED,
          dai::IMUSensor::GYROSCOPE_CALIBRATED},
        static_cast<int>(std::lround(startup_measurement_config_.imu_rate_hz)));
      imu->setBatchReportThreshold(1);
      imu->setMaxBatchReports(startup_measurement_config_.imu_max_batch_reports);
      imu_queue_ = imu->out.createOutputQueue(
        static_cast<unsigned int>(startup_measurement_config_.imu_queue_size), false);
      RCLCPP_INFO(
        get_logger(), "IR BEV IMU: keep still for %.1fs discard + %.1fs bias calibration; "
        "BEV waits for synchronized tilt, limit=%.1fdeg.",
        imu_stabilizer_config_.startup_discard_duration_sec,
        imu_stabilizer_config_.reference_calibration_duration_sec,
        imu_stabilizer_config_.maximum_correction_deg);
    }
    output_queue_ = host_output->createOutputQueue(1U, false);
    pipeline_->build();
    const auto xlink_bridge = host_output->getXLinkBridge();
    if (!xlink_bridge || !xlink_bridge->xLinkOut) {
      throw std::runtime_error(
              "DepthAI did not create the IR preview XLink bridge");
    }
    xlink_bridge->xLinkOut->input.setMaxSize(1);
    xlink_bridge->xLinkOut->input.setBlocking(false);

    pipeline_->start();
    setIrState(ir_enabled_at_start_, true);

    RCLCPP_INFO(
      get_logger(),
      "OAK IR preview: mode=%s, %dx%d@%.1f FPS requested, USB=%s, "
      "queue=1/non-blocking, XLink chunks=off, exposure=%s, previews<=%.1f FPS",
      reprojection_enabled_ ? "CUDA stereo-midpoint reprojection" :
      selectedCameraLabel(),
      width_, height_, requestedFps(), usbSpeedName(device_->getUsbSpeed()),
      manual_exposure_enabled_ ? "manual" : "auto",
      preview_max_fps_);
    if (reprojection_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "Virtual camera ratio=%.3f (0=LEFT/CAM_B, 1=RIGHT/CAM_C); "
        "invalid disparity falls back to %s.",
        virtual_camera_position_ratio_, selectedCameraLabel());
    }
    RCLCPP_INFO(
      get_logger(),
      "Keys: I=toggle configured IR emitters, B=save PNG, Q/Esc=quit.");
  }

  void setIrState(const bool enabled, const bool fail_hard)
  {
    if (!device_) {
      return;
    }

    const float dot_intensity = enabled ?
      static_cast<float>(ir_dot_projector_intensity_) : 0.0F;
    const float flood_intensity = enabled ?
      static_cast<float>(ir_flood_light_intensity_) : 0.0F;
    const bool dot_success =
      device_->setIrLaserDotProjectorIntensity(dot_intensity);
    const bool flood_success =
      device_->setIrFloodLightIntensity(flood_intensity);
    const bool dot_required = dot_intensity > 0.0F;
    const bool flood_required = flood_intensity > 0.0F;
    const bool success =
      (!dot_required || dot_success) && (!flood_required || flood_success);

    if (!success) {
      // Avoid leaving one emitter on after a partial update failure.
      device_->setIrLaserDotProjectorIntensity(0.0F);
      device_->setIrFloodLightIntensity(0.0F);
      ir_active_.store(false, std::memory_order_relaxed);
      const std::string message =
        "failed to set the requested OAK IR emitter intensity; a Pro-series "
        "device is required";
      if (fail_hard) {
        throw std::runtime_error(message);
      }
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      return;
    }

    const bool actually_active = enabled && (dot_required || flood_required);
    ir_active_.store(actually_active, std::memory_order_relaxed);
    RCLCPP_INFO(
      get_logger(),
      "IR emitters %s: dot=%.2f, flood=%.2f",
      actually_active ? "ON" : "OFF",
      dot_intensity,
      flood_intensity);
    control_panel_dirty_.store(true, std::memory_order_relaxed);
  }

  static std::string formatFlood(const double value)
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    return stream.str();
  }

  static double parseDouble(const std::string & text)
  {
    std::size_t parsed = 0U;
    const double value = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(value)) {
      throw std::invalid_argument("not a finite number");
    }
    return value;
  }

  static int parseInteger(const std::string & text)
  {
    std::size_t parsed = 0U;
    const long value = std::stol(text, &parsed);
    if (parsed != text.size() || value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max())
    {
      throw std::invalid_argument("not an integer");
    }
    return static_cast<int>(value);
  }

  void sendManualExposure(const int exposure_us, const int sensitivity_iso)
  {
    for (const auto & queue : camera_control_queues_) {
      if (!queue) {
        continue;
      }
      auto control = std::make_shared<dai::CameraControl>();
      control->setManualExposure(
        static_cast<std::uint32_t>(exposure_us),
        static_cast<std::uint32_t>(sensitivity_iso));
      queue->send(control);
    }
    manual_exposure_active_.store(true, std::memory_order_relaxed);
  }

  void sendAutoExposure()
  {
    for (const auto & queue : camera_control_queues_) {
      if (!queue) {
        continue;
      }
      auto control = std::make_shared<dai::CameraControl>();
      control->setAutoExposureEnable();
      queue->send(control);
    }
    manual_exposure_active_.store(false, std::memory_order_relaxed);
    panel_message_ = "Auto exposure enabled";
    panel_message_is_error_ = false;
    control_panel_dirty_.store(true, std::memory_order_relaxed);
    RCLCPP_INFO(get_logger(), "IR camera auto exposure enabled.");
  }

  void applyControlInputs(const EditableField requested_field = EditableField::NONE)
  {
    try {
      const bool apply_flood = requested_field == EditableField::NONE ||
        requested_field == EditableField::FLOOD;
      const bool apply_exposure = requested_field == EditableField::NONE ||
        requested_field == EditableField::EXPOSURE || requested_field == EditableField::ISO;
      double flood = ir_flood_light_intensity_;
      int exposure_us = manual_exposure_us_;
      int sensitivity_iso = manual_sensitivity_iso_;

      if (apply_flood) {
        flood = parseDouble(flood_input_);
        if (flood < 0.0 || flood > 1.0) {
          throw std::invalid_argument("Flood must be in 0.0..1.0");
        }
      }

      if (apply_exposure) {
        exposure_us = parseInteger(exposure_input_);
        sensitivity_iso = parseInteger(iso_input_);
        if (exposure_us < 10 || exposure_us > maximum_manual_exposure_us_) {
          throw std::invalid_argument(
                  "Exposure must be in 10.." +
                  std::to_string(maximum_manual_exposure_us_) + " us");
        }
        if (sensitivity_iso < 100 || sensitivity_iso > 1600) {
          throw std::invalid_argument("ISO must be in 100..1600");
        }
      }

      if (apply_flood) {
        ir_flood_light_intensity_ = flood;
        flood_input_ = formatFlood(flood);
        setIrState(true, false);
      }
      if (apply_exposure) {
        manual_exposure_us_ = exposure_us;
        manual_sensitivity_iso_ = sensitivity_iso;
        exposure_input_ = std::to_string(exposure_us);
        iso_input_ = std::to_string(sensitivity_iso);
        sendManualExposure(exposure_us, sensitivity_iso);
      }

      panel_message_ = "Applied to OAK camera";
      panel_message_is_error_ = false;
      RCLCPP_INFO(
        get_logger(), "IR tuning applied: flood=%.2f exposure=%dus ISO=%d",
        ir_flood_light_intensity_, manual_exposure_us_, manual_sensitivity_iso_);
    } catch (const std::exception & exception) {
      panel_message_ = exception.what();
      panel_message_is_error_ = true;
      RCLCPP_WARN(get_logger(), "IR tuning was not applied: %s", exception.what());
    }
    control_panel_dirty_.store(true, std::memory_order_relaxed);
  }

  static void drawButton(
    cv::Mat & panel, const cv::Rect & rectangle, const std::string & label,
    const cv::Scalar & color)
  {
    cv::rectangle(panel, rectangle, color, cv::FILLED, cv::LINE_AA);
    cv::putText(
      panel, label, cv::Point(rectangle.x + 12, rectangle.y + 25),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
  }

  void drawControlPanel()
  {
    cv::Mat panel(kControlPanelHeight, kControlPanelWidth, CV_8UC3, cv::Scalar(28, 30, 34));
    cv::putText(
      panel, "IR live tuning - click a field, type, press Enter",
      cv::Point(20, 28), cv::FONT_HERSHEY_SIMPLEX, 0.62,
      cv::Scalar(235, 235, 235), 1, cv::LINE_AA);

    const std::array<std::string, 3> labels{
      "Flood intensity [0..1]", "Exposure [us]", "ISO [100..1600]"};
    const std::array<std::string, 3> values{
      flood_input_, exposure_input_, iso_input_};
    for (std::size_t index = 0U; index < inputRects().size(); ++index) {
      const auto field = static_cast<EditableField>(index);
      const auto & rectangle = inputRects()[index];
      cv::putText(
        panel, labels[index], cv::Point(20, rectangle.y + 25),
        cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(205, 205, 205), 1, cv::LINE_AA);
      const bool selected = editable_field_ == field;
      cv::rectangle(
        panel, rectangle, selected ? cv::Scalar(0, 210, 255) : cv::Scalar(130, 135, 145),
        selected ? 2 : 1, cv::LINE_AA);
      cv::putText(
        panel, values[index], cv::Point(rectangle.x + 10, rectangle.y + 26),
        cv::FONT_HERSHEY_SIMPLEX, 0.64, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    }

    drawButton(panel, applyRect(), "APPLY  [Enter]", cv::Scalar(85, 205, 120));
    drawButton(
      panel, autoExposureRect(), "AUTO EXPOSURE [A]", cv::Scalar(215, 180, 80));
    drawButton(
      panel, irToggleRect(),
      ir_active_.load(std::memory_order_relaxed) ? "IR OFF  [I]" : "IR ON   [I]",
      cv::Scalar(190, 150, 235));

    std::ostringstream actual;
    actual << "Actual: " << last_exposure_us_.load(std::memory_order_relaxed) << " us, ISO " <<
      last_sensitivity_iso_.load(std::memory_order_relaxed) << " | mode=" <<
      (manual_exposure_active_.load(std::memory_order_relaxed) ? "manual" : "auto") <<
      " | preview <= " << std::fixed << std::setprecision(1) << preview_max_fps_ << " FPS";
    cv::putText(
      panel, actual.str(), cv::Point(20, 226), cv::FONT_HERSHEY_SIMPLEX, 0.50,
      cv::Scalar(190, 205, 220), 1, cv::LINE_AA);
    cv::putText(
      panel, panel_message_.empty() ? "Tab: next field | B: save original+BEV | Q/Esc: quit" :
      panel_message_, cv::Point(20, 256), cv::FONT_HERSHEY_SIMPLEX, 0.48,
      panel_message_is_error_ ? cv::Scalar(80, 100, 255) : cv::Scalar(150, 225, 160),
      1, cv::LINE_AA);
    cv::imshow(controls_window_name_, panel);
    control_panel_dirty_.store(false, std::memory_order_relaxed);
  }

  void selectField(const EditableField field)
  {
    editable_field_ = field;
    replace_field_on_next_key_ = true;
    panel_message_.clear();
    control_panel_dirty_.store(true, std::memory_order_relaxed);
  }

  static void controlMouseCallback(
    const int event, const int x, const int y, const int, void * context)
  {
    if (event != cv::EVENT_LBUTTONDOWN || context == nullptr) {
      return;
    }
    static_cast<IrCameraDriverNode *>(context)->onControlMouse(x, y);
  }

  void onControlMouse(const int x, const int y)
  {
    const cv::Point point(x, y);
    for (std::size_t index = 0U; index < inputRects().size(); ++index) {
      if (inputRects()[index].contains(point)) {
        selectField(static_cast<EditableField>(index));
        return;
      }
    }
    editable_field_ = EditableField::NONE;
    replace_field_on_next_key_ = false;
    if (applyRect().contains(point)) {
      applyControlInputs();
    } else if (autoExposureRect().contains(point)) {
      sendAutoExposure();
    } else if (irToggleRect().contains(point)) {
      setIrState(!ir_active_.load(std::memory_order_relaxed), false);
    }
    control_panel_dirty_.store(true, std::memory_order_relaxed);
  }

  std::string * activeInput()
  {
    switch (editable_field_) {
      case EditableField::FLOOD:
        return &flood_input_;
      case EditableField::EXPOSURE:
        return &exposure_input_;
      case EditableField::ISO:
        return &iso_input_;
      case EditableField::NONE:
      default:
        return nullptr;
    }
  }

  bool handleEditingKey(const int key)
  {
    if (key == 9) {
      const int next = editable_field_ == EditableField::NONE ? 0 :
        (static_cast<int>(editable_field_) + 1) % 3;
      selectField(static_cast<EditableField>(next));
      return true;
    }
    auto * input = activeInput();
    if (input == nullptr) {
      return false;
    }
    if (key == 10 || key == 13) {
      const EditableField requested_field = editable_field_;
      editable_field_ = EditableField::NONE;
      replace_field_on_next_key_ = false;
      applyControlInputs(requested_field);
      return true;
    }
    if (key == 8 || key == 127) {
      if (replace_field_on_next_key_) {
        input->clear();
        replace_field_on_next_key_ = false;
      } else if (!input->empty()) {
        input->pop_back();
      }
      control_panel_dirty_.store(true, std::memory_order_relaxed);
      return true;
    }
    const bool digit = key >= '0' && key <= '9';
    const bool decimal = key == '.' && editable_field_ == EditableField::FLOOD &&
      input->find('.') == std::string::npos;
    if (digit || decimal) {
      if (replace_field_on_next_key_) {
        input->clear();
        replace_field_on_next_key_ = false;
      }
      if (input->size() < 12U) {
        input->push_back(static_cast<char>(key));
      }
      control_panel_dirty_.store(true, std::memory_order_relaxed);
      return true;
    }
    return true;
  }

  void ensureBevProjector(
    IrBevProjector & projector, dai::ImgFrame & reference_frame,
    const dai::CameraBoardSocket socket)
  {
    if (projector.ready()) {
      return;
    }
    const auto & transformation = reference_frame.getTransformation();
    if (!transformation.isValid()) {
      throw std::runtime_error(
              "IR BEV requires actual output-frame intrinsics; metadata is missing");
    }
    const auto intrinsics = transformation.getIntrinsicMatrix();
    const auto calibration = device_->getCalibration();
    // Frame metadata contains the actual output optical axes, including
    // stereo rectification. Compose T_view->reference with T_reference->CAM_A
    // exactly as DepthAI's PointCloud conversion does; do not apply the
    // EEPROM rectification a second time or assume rectified==physical axes.
    const auto frame_extrinsics = transformation.getExtrinsics();
    const auto reference_socket = frame_extrinsics.toCameraSocket;
    if (reference_socket == dai::CameraBoardSocket::AUTO) {
      throw std::runtime_error("IR BEV output-frame extrinsics have no reference camera");
    }
    const auto view_to_reference = frame_extrinsics.getTransformationMatrix(
      false, dai::LengthUnit::METER);
    const auto rotation_reference_from_view = calibrationRotation(
      view_to_reference, "output-frame-to-reference");
    cv::Matx33d rotation_rgb_from_reference = cv::Matx33d::eye();
    cv::Vec3d reference_center_rgb_m(0.0, 0.0, 0.0);
    if (reference_socket != dai::CameraBoardSocket::CAM_A) {
      const auto reference_to_rgb = calibration.getCameraExtrinsics(
        reference_socket, dai::CameraBoardSocket::CAM_A, false);
      rotation_rgb_from_reference = calibrationRotation(reference_to_rgb, "reference-to-CAM_A");
      if (reference_to_rgb[0].size() < 4U || reference_to_rgb[1].size() < 4U ||
        reference_to_rgb[2].size() < 4U)
      {
        throw std::runtime_error("reference-to-CAM_A translation is missing");
      }
      // CalibrationHandler's default translation unit is centimeters.
      reference_center_rgb_m = cv::Vec3d(
        reference_to_rgb[0][3], reference_to_rgb[1][3], reference_to_rgb[2][3]) * 0.01;
    }
    const auto rotation_rgb_from_view =
      rotation_rgb_from_reference * rotation_reference_from_view;
    const auto center_rgb_m = reference_center_rgb_m + rotation_rgb_from_reference *
      cv::Vec3d(view_to_reference[0][3], view_to_reference[1][3], view_to_reference[2][3]);
    projector.configure(
      intrinsics[0][0], intrinsics[1][1], intrinsics[0][2], intrinsics[1][2],
      width_, height_, rotation_rgb_from_view, center_rgb_m);
    RCLCPP_INFO(
      get_logger(), "IR BEV calibrated: %s K=(%.3f,%.3f,%.3f,%.3f), "
      "IR center in CAM_A=(%.4f,%.4f,%.4f)m, ground coverage=%.1f%%",
      socket == dai::CameraBoardSocket::CAM_B ? "LEFT" : "RIGHT",
      intrinsics[0][0], intrinsics[1][1], intrinsics[0][2], intrinsics[1][2],
      center_rgb_m[0], center_rgb_m[1], center_rgb_m[2],
      100.0 * projector.validPixelRatio());
  }

  void imuLoop()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(imu_calibration_timeout_sec_);
    bool calibration_reported = false;
    while (!stop_requested_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      try {
        if (!pipeline_ || !pipeline_->isRunning()) {
          break;
        }
        if (!imu_stabilizer_->initialized() && std::chrono::steady_clock::now() > deadline) {
          const auto progress = imu_stabilizer_->calibrationProgress();
          RCLCPP_FATAL(
            get_logger(), "IR BEV IMU calibration timed out: %s. Keep the vehicle still.",
            progress.last_rejection_reason.c_str());
          rclcpp::shutdown();
          break;
        }
        auto data = imu_queue_->tryGet<dai::IMUData>();
        if (!data) {
          std::this_thread::sleep_for(200us);
          continue;
        }
        for (const auto & packet : data->packets) {
          const auto & accel = packet.acceleroMeter;
          const auto & gyro = packet.gyroscope;
          const double gyro_time = timestampSeconds(gyro.getTimestamp());
          const double skew = std::abs(timestampSeconds(accel.getTimestamp()) - gyro_time);
          std::optional<cv::Vec3d> acceleration;
          if (std::isfinite(skew) && skew <=
            startup_measurement_config_.maximum_imu_pair_skew_sec)
          {
            acceleration = calibrated_imu_to_rgb_ * cv::Vec3d(accel.x, accel.y, accel.z);
          }
          imu_stabilizer_->update(
            acceleration, calibrated_imu_to_rgb_ * cv::Vec3d(gyro.x, gyro.y, gyro.z), gyro_time);
          imu_samples_total_.fetch_add(1U, std::memory_order_relaxed);
        }
        if (!calibration_reported && imu_stabilizer_->initialized()) {
          calibration_reported = true;
          RCLCPP_INFO(get_logger(), "IR BEV IMU calibration complete; live tilt correction active.");
        }
      } catch (const std::exception & exception) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "IR IMU error: %s", exception.what());
        std::this_thread::sleep_for(1ms);
      }
    }
  }

  std::optional<cv::Matx33d> correctionFor(const dai::ImgFrame & frame)
  {
    if (!imu_stabilization_enabled_) {
      return cv::Matx33d::eye();
    }
    const auto correction = imu_stabilizer_->correctionAt(
      timestampSeconds(frame.getTimestamp(dai::CameraExposureOffset::MIDDLE)));
    if (!correction || !correction->within_correction_limit) {
      bev_imu_rejections_total_.fetch_add(1U, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IR BEV withheld: IMU calibrating, frame/IMU timing invalid, or tilt exceeds limit.");
      return std::nullopt;
    }
    latest_tilt_deg_.store(correction->correction_angle_deg, std::memory_order_relaxed);
    return correction->camera_to_reference_tilt;
  }

  cv::Mat projectSingleBev(dai::ImgFrame & frame)
  {
    ensureBevProjector(*bev_projector_, frame, selectedSocket());
    const auto correction = correctionFor(frame);
    return correction ? bev_projector_->project(frame.getFrame(false), *correction) : cv::Mat();
  }

  cv::Mat projectStereoBev(const StereoSnapshot & snapshot)
  {
    ensureBevProjector(*bev_projector_, *snapshot.left, dai::CameraBoardSocket::CAM_B);
    ensureBevProjector(*right_bev_projector_, *snapshot.right, dai::CameraBoardSocket::CAM_C);
    const auto left_correction = correctionFor(*snapshot.left);
    const auto right_correction = correctionFor(*snapshot.right);
    if (!left_correction || !right_correction) {
      return {};
    }
    // Project each physical lens onto the SAME metric ground grid. The center
    // preview's invalid-disparity fallback belongs to a different optical
    // center and must never be interpreted as metric virtual-camera pixels.
    const auto left = bev_projector_->project(snapshot.left->getFrame(false), *left_correction);
    const auto right = right_bev_projector_->project(
      snapshot.right->getFrame(false), *right_correction);
    cv::Mat both, only_right, output;
    cv::bitwise_and(bev_projector_->validMask(), right_bev_projector_->validMask(), both);
    cv::bitwise_not(bev_projector_->validMask(), only_right);
    cv::bitwise_and(only_right, right_bev_projector_->validMask(), only_right);
    output = left.clone();
    right.copyTo(output, only_right);
    cv::Mat blended;
    cv::addWeighted(left, 0.5, right, 0.5, 0.0, blended);
    blended.copyTo(output, both);
    return output;
  }

  bool validFrameSize(const dai::ImgFrame & frame) const
  {
    return
      static_cast<int>(frame.getWidth()) == width_ &&
      static_cast<int>(frame.getHeight()) == height_;
  }

  void recordSequence(const std::int64_t sequence)
  {
    if (last_sequence_.has_value() && sequence > *last_sequence_ + 1) {
      skipped_sequences_total_.fetch_add(
        static_cast<std::uint64_t>(sequence - *last_sequence_ - 1),
        std::memory_order_relaxed);
    }
    last_sequence_ = sequence;
  }

  void captureLoop()
  {
    while (!stop_requested_.load(std::memory_order_relaxed)) {
      try {
        if (!pipeline_ || !pipeline_->isRunning()) {
          break;
        }

        if (reprojection_enabled_) {
          auto group = output_queue_->tryGet<dai::MessageGroup>();
          if (!group) {
            std::this_thread::sleep_for(100us);
            continue;
          }
          auto left = group->get<dai::ImgFrame>("left");
          auto right = group->get<dai::ImgFrame>("right");
          auto disparity = group->get<dai::ImgFrame>("disparity");
          if (
            !left || !right || !disparity ||
            !validFrameSize(*left) || !validFrameSize(*right) ||
            !validFrameSize(*disparity))
          {
            invalid_frames_total_.fetch_add(1U, std::memory_order_relaxed);
            continue;
          }

          const auto sequence = disparity->getSequenceNum();
          recordSequence(sequence);
          const auto generation =
            captured_total_.fetch_add(1U, std::memory_order_relaxed) + 1U;
          capture_interval_.fetch_add(1U, std::memory_order_relaxed);
          last_exposure_us_.store(
            left->getExposureTime().count(), std::memory_order_relaxed);
          last_sensitivity_iso_.store(
            left->getSensitivity(), std::memory_order_relaxed);
          std::shared_ptr<const StereoSnapshot> snapshot =
            std::make_shared<StereoSnapshot>(
            StereoSnapshot{
              std::move(left), std::move(right), std::move(disparity),
              generation, sequence});
          std::atomic_store_explicit(
            &latest_stereo_, std::move(snapshot), std::memory_order_release);
        } else {
          auto frame = output_queue_->tryGet<dai::ImgFrame>();
          if (!frame) {
            std::this_thread::sleep_for(100us);
            continue;
          }
          if (!validFrameSize(*frame)) {
            invalid_frames_total_.fetch_add(1U, std::memory_order_relaxed);
            continue;
          }

          const auto sequence = frame->getSequenceNum();
          recordSequence(sequence);
          const auto generation =
            captured_total_.fetch_add(1U, std::memory_order_relaxed) + 1U;
          capture_interval_.fetch_add(1U, std::memory_order_relaxed);
          last_exposure_us_.store(
            frame->getExposureTime().count(), std::memory_order_relaxed);
          last_sensitivity_iso_.store(
            frame->getSensitivity(), std::memory_order_relaxed);
          std::shared_ptr<const SingleSnapshot> snapshot =
            std::make_shared<SingleSnapshot>(
            SingleSnapshot{std::move(frame), generation, sequence});
          std::atomic_store_explicit(
            &latest_single_, std::move(snapshot), std::memory_order_release);
        }
        frame_available_.notify_one();
      } catch (const std::exception & exception) {
        capture_errors_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "IR camera capture error: %s", exception.what());
        std::this_thread::sleep_for(1ms);
      }
    }
  }

  void resizePreviewWindow(const cv::Mat & frame)
  {
    if (preview_window_sized_) {
      return;
    }
    double scale = 1.0;
    if (preview_max_width_ > 0) {
      scale = std::min(
        scale,
        static_cast<double>(preview_max_width_) /
        static_cast<double>(frame.cols));
    }
    if (preview_max_height_ > 0) {
      scale = std::min(
        scale,
        static_cast<double>(preview_max_height_) /
        static_cast<double>(frame.rows));
    }
    cv::resizeWindow(
      preview_window_name_,
      std::max(1, static_cast<int>(frame.cols * scale)),
      std::max(1, static_cast<int>(frame.rows * scale)));
    preview_window_sized_ = true;
  }

  void saveFrames(const cv::Mat & original, const cv::Mat & bev)
  {
    if (original.empty() || bev.empty()) {
      RCLCPP_WARN(get_logger(), "Both IR original and BEV frames are required to save.");
      return;
    }
    try {
      const std::filesystem::path directory(capture_directory_);
      std::error_code error;
      std::filesystem::create_directories(directory, error);
      if (error) {
        RCLCPP_ERROR(
          get_logger(), "Could not create '%s': %s",
          directory.string().c_str(), error.message().c_str());
        return;
      }
      const std::string mode = reprojection_enabled_ ?
        "center" : (selected_camera_ == SelectedCamera::LEFT ?
        "left" : "right");
      const std::string ir_state =
        ir_active_.load(std::memory_order_relaxed) ? "ir_on" : "ir_off";
      const std::string stem = "ir_" + mode + "_" + ir_state + "_" +
        std::to_string(get_clock()->now().nanoseconds());
      const auto original_filename = directory / (stem + "_original.png");
      const auto bev_filename = directory / (stem + "_bev.png");
      if (!cv::imwrite(original_filename.string(), original) ||
        !cv::imwrite(bev_filename.string(), bev))
      {
        throw std::runtime_error("OpenCV imwrite returned false");
      }
      const auto absolute_path = std::filesystem::absolute(original_filename, error);
      RCLCPP_INFO(
        get_logger(), "IR original and BEV saved beside: %s",
        (error ? original_filename : absolute_path).string().c_str());
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "Could not save IR preview: %s", exception.what());
    }
  }

  void previewLoop()
  {
    try {
      cv::namedWindow(preview_window_name_, cv::WINDOW_NORMAL);
      cv::namedWindow(bev_preview_window_name_, cv::WINDOW_NORMAL);
      cv::namedWindow(controls_window_name_, cv::WINDOW_AUTOSIZE);
      cv::setMouseCallback(controls_window_name_, controlMouseCallback, this);
      cv::resizeWindow(
        bev_preview_window_name_,
        std::max(1, static_cast<int>(std::lround(
            static_cast<double>(bev_config_.output_width) * bev_preview_scale_))),
        std::max(1, static_cast<int>(std::lround(
            static_cast<double>(bev_config_.output_height) * bev_preview_scale_))));
      drawControlPanel();
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(
        get_logger(), "Could not create IR preview windows: %s",
        exception.what());
      rclcpp::shutdown();
      return;
    }

    std::uint64_t previewed_generation = 0U;
    cv::Mat current_frame;
    cv::Mat current_bev;
    std::shared_ptr<const StereoSnapshot> displayed_stereo;
    std::shared_ptr<const SingleSnapshot> displayed_single;
    bool window_was_visible = false;
    auto next_preview_at = std::chrono::steady_clock::now();
    const auto preview_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / preview_max_fps_));

    while (!stop_requested_.load(std::memory_order_relaxed) && rclcpp::ok()) {
      {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        frame_available_.wait_for(lock, 2ms);
      }
      if (stop_requested_.load(std::memory_order_relaxed)) {
        break;
      }

      try {
        bool frame_updated = false;
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_preview_at) {
          next_preview_at = now + preview_period;
          if (reprojection_enabled_) {
            auto snapshot = std::atomic_load_explicit(
              &latest_stereo_, std::memory_order_acquire);
            if (snapshot && snapshot->generation != previewed_generation) {
              const auto reprojection_started_at =
                std::chrono::steady_clock::now();
              const cv::Mat left = snapshot->left->getFrame(false);
              const cv::Mat right = snapshot->right->getFrame(false);
              const cv::Mat disparity = snapshot->disparity->getFrame(false);
              current_frame = center_reprojector_->process(
                left,
                right,
                disparity,
                virtual_camera_position_ratio_,
                selected_camera_ == SelectedCamera::LEFT);
              const auto elapsed = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - reprojection_started_at);
              latest_reprojection_ms_.store(
                elapsed.count(), std::memory_order_relaxed);
              displayed_stereo = std::move(snapshot);
              displayed_single.reset();
              previewed_generation = displayed_stereo->generation;
              frame_updated = true;
            }
          } else {
            auto snapshot = std::atomic_load_explicit(
              &latest_single_, std::memory_order_acquire);
            if (snapshot && snapshot->generation != previewed_generation) {
              current_frame = snapshot->frame->getFrame(false);
              displayed_single = std::move(snapshot);
              displayed_stereo.reset();
              previewed_generation = displayed_single->generation;
              frame_updated = true;
            }
          }
        }

        if (frame_updated && !current_frame.empty()) {
          const auto bev_started_at = std::chrono::steady_clock::now();
          current_bev.release();
          try {
            current_bev = displayed_stereo ? projectStereoBev(*displayed_stereo) :
              projectSingleBev(*displayed_single->frame);
          } catch (const std::exception & exception) {
            preview_errors_total_.fetch_add(1U, std::memory_order_relaxed);
            RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 1000, "IR BEV withheld: %s", exception.what());
          }
          latest_bev_ms_.store(
            std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - bev_started_at).count(),
            std::memory_order_relaxed);
          resizePreviewWindow(current_frame);
          cv::imshow(preview_window_name_, current_frame);
          if (current_bev.empty()) {
            cv::Mat waiting(300, 500, CV_8UC1, cv::Scalar(0));
            cv::putText(waiting, "BEV unavailable: check calibration / IMU", cv::Point(10, 145),
              cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255), 1, cv::LINE_AA);
            cv::imshow(bev_preview_window_name_, waiting);
          } else {
            cv::imshow(bev_preview_window_name_, current_bev);
            bev_preview_interval_.fetch_add(1U, std::memory_order_relaxed);
          }
          previewed_total_.fetch_add(1U, std::memory_order_relaxed);
          original_preview_interval_.fetch_add(1U, std::memory_order_relaxed);
          control_panel_dirty_.store(true, std::memory_order_relaxed);
        }

        if (control_panel_dirty_.load(std::memory_order_relaxed)) {
          drawControlPanel();
        }

        const int raw_key = cv::waitKeyEx(1);
        const int key = raw_key < 0 ? -1 : raw_key & 0xff;
        if (key == 27 || key == 'q' || key == 'Q') {
          RCLCPP_INFO(get_logger(), "IR preview closed.");
          stop_requested_.store(true, std::memory_order_relaxed);
          frame_available_.notify_all();
          rclcpp::shutdown();
          break;
        }
        const bool editing_key_handled = raw_key >= 0 && handleEditingKey(key);
        if (!editing_key_handled && (key == 'i' || key == 'I')) {
          setIrState(
            !ir_active_.load(std::memory_order_relaxed), false);
        } else if (!editing_key_handled && (key == 'a' || key == 'A')) {
          sendAutoExposure();
        } else if (!editing_key_handled && (key == 'b' || key == 'B')) {
          saveFrames(current_frame, current_bev);
        }

        const double original_visible = cv::getWindowProperty(
          preview_window_name_, cv::WND_PROP_VISIBLE);
        const double bev_visible = cv::getWindowProperty(
          bev_preview_window_name_, cv::WND_PROP_VISIBLE);
        const double controls_visible = cv::getWindowProperty(
          controls_window_name_, cv::WND_PROP_VISIBLE);
        if (original_visible >= 1.0 && bev_visible >= 1.0 && controls_visible >= 1.0) {
          window_was_visible = true;
        }
        if (
          window_was_visible &&
          (original_visible < 1.0 || bev_visible < 1.0 || controls_visible < 1.0))
        {
          RCLCPP_INFO(get_logger(), "IR preview closed.");
          stop_requested_.store(true, std::memory_order_relaxed);
          frame_available_.notify_all();
          rclcpp::shutdown();
          break;
        }
      } catch (const std::exception & exception) {
        preview_errors_total_.fetch_add(1U, std::memory_order_relaxed);
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "IR preview error: %s", exception.what());
      }
    }

    try {
      cv::destroyWindow(preview_window_name_);
      cv::destroyWindow(bev_preview_window_name_);
      cv::destroyWindow(controls_window_name_);
    } catch (...) {
    }
  }

  void reportStatus()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(now - last_status_at_).count();
    last_status_at_ = now;
    if (elapsed <= 0.0) {
      return;
    }

    const auto capture_count =
      capture_interval_.exchange(0U, std::memory_order_relaxed);
    const auto original_preview_count =
      original_preview_interval_.exchange(0U, std::memory_order_relaxed);
    const auto bev_preview_count =
      bev_preview_interval_.exchange(0U, std::memory_order_relaxed);
    RCLCPP_INFO(
      get_logger(),
      "[IR_CAMERA] capture=%.1fHz original=%.1fHz BEV=%.1fHz requested=%.1fHz, "
      "exposure=%ldus ISO=%d, IR=%s, reproject=%.2fms BEV=%.2fms, "
      "skipped=%lu invalid=%lu capture_errors=%lu preview_errors=%lu, "
      "IMU=%s samples=%lu BEV_rejected=%lu tilt=%.3fdeg",
      static_cast<double>(capture_count) / elapsed,
      static_cast<double>(original_preview_count) / elapsed,
      static_cast<double>(bev_preview_count) / elapsed,
      requestedFps(),
      static_cast<long>(last_exposure_us_.load(std::memory_order_relaxed)),
      last_sensitivity_iso_.load(std::memory_order_relaxed),
      ir_active_.load(std::memory_order_relaxed) ? "on" : "off",
      latest_reprojection_ms_.load(std::memory_order_relaxed),
      latest_bev_ms_.load(std::memory_order_relaxed),
      static_cast<unsigned long>(
        skipped_sequences_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        invalid_frames_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        capture_errors_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(
        preview_errors_total_.load(std::memory_order_relaxed)),
      !imu_stabilizer_ ? "off" : (imu_stabilizer_->initialized() ? "ready" : "calibrating"),
      static_cast<unsigned long>(imu_samples_total_.load(std::memory_order_relaxed)),
      static_cast<unsigned long>(bev_imu_rejections_total_.load(std::memory_order_relaxed)),
      latest_tilt_deg_.load(std::memory_order_relaxed));
  }

  void stop()
  {
    if (stopped_.exchange(true, std::memory_order_relaxed)) {
      return;
    }
    stop_requested_.store(true, std::memory_order_relaxed);
    frame_available_.notify_all();

    if (imu_thread_.joinable()) {
      imu_thread_.join();
    }
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (
      preview_thread_.joinable() &&
      preview_thread_.get_id() != std::this_thread::get_id())
    {
      preview_thread_.join();
    }
    if (status_timer_) {
      status_timer_->cancel();
      status_timer_.reset();
    }

    if (device_) {
      try {
        device_->setIrLaserDotProjectorIntensity(0.0F);
        device_->setIrFloodLightIntensity(0.0F);
        ir_active_.store(false, std::memory_order_relaxed);
      } catch (...) {
      }
    }
    imu_queue_.reset();
    output_queue_.reset();
    camera_control_queues_.clear();
    if (pipeline_) {
      try {
        if (pipeline_->isRunning()) {
          pipeline_->stop();
          pipeline_->wait();
        }
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          get_logger(), "OAK shutdown error: %s", exception.what());
      }
      pipeline_.reset();
    }
    device_.reset();
    center_reprojector_.reset();
  }

  std::string device_id_;
  bool startup_measurement_enabled_{true};
  bool imu_stabilization_enabled_{true};
  double imu_calibration_timeout_sec_{30.0};
  oak_startup::OakStartupMeasurementConfig startup_measurement_config_;
  camera_driver::ImuImageStabilizerConfig imu_stabilizer_config_;
  std::unique_ptr<camera_driver::ImuImageStabilizer> imu_stabilizer_;
  std::shared_ptr<dai::MessageQueue> imu_queue_;
  cv::Matx33d calibrated_imu_to_rgb_{cv::Matx33d::eye()};
  std::thread imu_thread_;
  std::atomic<std::uint64_t> imu_samples_total_{0U};
  std::atomic<std::uint64_t> bev_imu_rejections_total_{0U};
  std::atomic<double> latest_tilt_deg_{0.0};
  std::unique_ptr<IrBevProjector> right_bev_projector_;

  bool reprojection_enabled_{true};
  std::string selected_camera_name_{"LEFT"};
  SelectedCamera selected_camera_{SelectedCamera::LEFT};
  double virtual_camera_position_ratio_{0.5};
  int width_{1280};
  int height_{800};
  double reprojection_fps_{30.0};
  double single_camera_fps_{30.0};
  bool undistort_single_camera_{true};
  double sync_threshold_ms_{2.0};

  bool ir_enabled_at_start_{true};
  double ir_dot_projector_intensity_{1.0};
  double ir_flood_light_intensity_{0.0};
  bool manual_exposure_enabled_{true};
  int manual_exposure_us_{5000};
  int manual_sensitivity_iso_{800};
  int maximum_manual_exposure_us_{0};

  std::string preview_window_name_{"OAK IR center preview"};
  std::string bev_preview_window_name_{"OAK IR BEV preview"};
  std::string controls_window_name_{"OAK IR live controls"};
  double preview_max_fps_{30.0};
  int preview_max_width_{1280};
  int preview_max_height_{800};
  double bev_preview_scale_{2.0};
  std::string capture_directory_{"."};
  double status_log_interval_sec_{1.0};
  bool preview_window_sized_{false};

  IrBevConfig bev_config_;
  EditableField editable_field_{EditableField::NONE};
  bool replace_field_on_next_key_{false};
  std::string flood_input_{"0.00"};
  std::string exposure_input_{"5000"};
  std::string iso_input_{"800"};
  std::string panel_message_;
  bool panel_message_is_error_{false};

  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> output_queue_;
  std::vector<std::shared_ptr<dai::InputQueue>> camera_control_queues_;
  std::unique_ptr<CudaCenterReprojector> center_reprojector_;
  std::unique_ptr<IrBevProjector> bev_projector_;

  std::shared_ptr<const StereoSnapshot> latest_stereo_;
  std::shared_ptr<const SingleSnapshot> latest_single_;
  std::mutex wait_mutex_;
  std::condition_variable frame_available_;
  std::thread capture_thread_;
  std::thread preview_thread_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> stopped_{false};
  std::atomic<bool> ir_active_{false};
  std::atomic<bool> manual_exposure_active_{false};
  std::atomic<bool> control_panel_dirty_{true};
  std::atomic<std::uint64_t> captured_total_{0U};
  std::atomic<std::uint64_t> capture_interval_{0U};
  std::atomic<std::uint64_t> previewed_total_{0U};
  std::atomic<std::uint64_t> original_preview_interval_{0U};
  std::atomic<std::uint64_t> bev_preview_interval_{0U};
  std::atomic<std::uint64_t> skipped_sequences_total_{0U};
  std::atomic<std::uint64_t> invalid_frames_total_{0U};
  std::atomic<std::uint64_t> capture_errors_total_{0U};
  std::atomic<std::uint64_t> preview_errors_total_{0U};
  std::atomic<std::int64_t> last_exposure_us_{0};
  std::atomic<int> last_sensitivity_iso_{0};
  std::atomic<double> latest_reprojection_ms_{0.0};
  std::atomic<double> latest_bev_ms_{0.0};
  std::optional<std::int64_t> last_sequence_;
  std::chrono::steady_clock::time_point started_at_{};
  std::chrono::steady_clock::time_point last_status_at_{};
};

}  // namespace ir_camera_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<ir_camera_driver::IrCameraDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ir_camera_driver"),
      "IR camera driver terminated: %s", exception.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
