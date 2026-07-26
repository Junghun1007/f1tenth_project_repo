#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "camera_height_estimator/camera_height_geometry.hpp"
#include "depthai/depthai.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace camera_height_estimator
{

using namespace std::chrono_literals;

namespace
{

constexpr std::uint32_t kOv9282FullWidth = 1280U;
constexpr std::uint32_t kOv9282FullHeight = 800U;
constexpr double kRadiansToDegrees =
  180.0 / 3.141592653589793238462643383279502884;

std::array<double, 3> normalized(
  const std::array<double, 3> & vector)
{
  const double norm = std::sqrt(
    vector[0] * vector[0] +
    vector[1] * vector[1] +
    vector[2] * vector[2]);
  if (!std::isfinite(norm) || norm <= 1.0e-9) {
    throw std::invalid_argument("cannot normalize a zero/non-finite vector");
  }
  return {vector[0] / norm, vector[1] / norm, vector[2] / norm};
}

double clampedAcos(const double value)
{
  return std::acos(std::clamp(value, -1.0, 1.0));
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

class CameraHeightEstimatorNode : public rclcpp::Node
{
public:
  CameraHeightEstimatorNode()
  : Node("camera_height_estimator")
  {
    readParameters();
    height_publisher_ = create_publisher<std_msgs::msg::Float64>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());

    startOak();
    measurement_started_at_ = std::chrono::steady_clock::now();
    poll_timer_ = create_wall_timer(
      1ms, std::bind(&CameraHeightEstimatorNode::pollOak, this));

    RCLCPP_INFO(
      get_logger(),
      "Independent height measurement started: stereo=%dx%d@%.1fHz, "
      "center ROI=%dx%d, IMU=%.1fHz, output=%s",
      stereo_width_, stereo_height_, stereo_fps_,
      roi_width_, roi_height_, imu_rate_hz_, output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "No ROS image/IMU input is used and no RGB image stream is started.");
  }

  ~CameraHeightEstimatorNode() override
  {
    releaseOak();
  }

private:
  enum class MeasurementState
  {
    WAITING_FOR_STABLE_IMU,
    COLLECTING_DEPTH,
    COMPLETE,
    FAILED
  };

  void readParameters()
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/camera/height");
    stereo_fps_ = declare_parameter<double>("stereo_fps", 30.0);
    stereo_width_ = declare_parameter<int>("stereo_width", 640);
    stereo_height_ = declare_parameter<int>("stereo_height", 400);
    depth_queue_size_ = declare_parameter<int>("depth_queue_size", 2);
    imu_rate_hz_ = declare_parameter<double>("imu_rate_hz", 100.0);
    imu_queue_size_ = declare_parameter<int>("imu_queue_size", 20);

    roi_width_ = declare_parameter<int>("roi_width", 10);
    roi_height_ = declare_parameter<int>("roi_height", 10);
    minimum_valid_pixels_ =
      declare_parameter<int>("minimum_valid_pixels", 25);
    minimum_depth_m_ =
      declare_parameter<double>("minimum_depth_m", 0.30);
    maximum_depth_m_ =
      declare_parameter<double>("maximum_depth_m", 3.00);
    minimum_height_m_ =
      declare_parameter<double>("minimum_height_m", 0.10);
    maximum_height_m_ =
      declare_parameter<double>("maximum_height_m", 1.00);
    maximum_height_mad_m_ =
      declare_parameter<double>("maximum_height_mad_m", 0.015);
    minimum_downward_ray_component_ = declare_parameter<double>(
      "minimum_downward_ray_component", 0.05);

    imu_sample_count_ = declare_parameter<int>("imu_sample_count", 10);
    imu_max_direction_rms_deg_ = declare_parameter<double>(
      "imu_max_direction_rms_deg", 0.50);
    imu_accel_min_mps2_ =
      declare_parameter<double>("imu_accel_min_mps2", 7.50);
    imu_accel_max_mps2_ =
      declare_parameter<double>("imu_accel_max_mps2", 12.00);

    stable_depth_frame_count_ =
      declare_parameter<int>("stable_depth_frame_count", 5);
    maximum_height_stddev_m_ =
      declare_parameter<double>("maximum_height_stddev_m", 0.010);
    measurement_timeout_sec_ =
      declare_parameter<double>("measurement_timeout_sec", 10.0);

    if (
      output_topic_.empty() ||
      !std::isfinite(stereo_fps_) || stereo_fps_ <= 0.0 ||
      stereo_width_ <= 0 || stereo_height_ <= 0 ||
      depth_queue_size_ <= 0 ||
      !std::isfinite(imu_rate_hz_) || imu_rate_hz_ <= 0.0 ||
      imu_queue_size_ <= 0 ||
      roi_width_ <= 0 || roi_height_ <= 0 ||
      roi_width_ > stereo_width_ || roi_height_ > stereo_height_ ||
      minimum_valid_pixels_ <= 0 ||
      minimum_valid_pixels_ > roi_width_ * roi_height_ ||
      !std::isfinite(minimum_depth_m_) || minimum_depth_m_ <= 0.0 ||
      !std::isfinite(maximum_depth_m_) ||
      maximum_depth_m_ <= minimum_depth_m_ ||
      !std::isfinite(minimum_height_m_) || minimum_height_m_ <= 0.0 ||
      !std::isfinite(maximum_height_m_) ||
      maximum_height_m_ <= minimum_height_m_ ||
      !std::isfinite(maximum_height_mad_m_) ||
      maximum_height_mad_m_ <= 0.0 ||
      !std::isfinite(minimum_downward_ray_component_) ||
      minimum_downward_ray_component_ <= 0.0 ||
      minimum_downward_ray_component_ >= 1.0 ||
      imu_sample_count_ <= 0 ||
      !std::isfinite(imu_max_direction_rms_deg_) ||
      imu_max_direction_rms_deg_ <= 0.0 ||
      !std::isfinite(imu_accel_min_mps2_) ||
      imu_accel_min_mps2_ <= 0.0 ||
      !std::isfinite(imu_accel_max_mps2_) ||
      imu_accel_max_mps2_ <= imu_accel_min_mps2_ ||
      stable_depth_frame_count_ <= 0 ||
      !std::isfinite(maximum_height_stddev_m_) ||
      maximum_height_stddev_m_ <= 0.0 ||
      !std::isfinite(measurement_timeout_sec_) ||
      measurement_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "invalid independent camera-height estimator parameter");
    }

    geometry_config_ = HeightGeometryConfig{
      static_cast<std::size_t>(minimum_valid_pixels_),
      minimum_depth_m_,
      maximum_depth_m_,
      minimum_height_m_,
      maximum_height_m_,
      maximum_height_mad_m_,
      minimum_downward_ray_component_};
  }

  void startOak()
  {
    device_ = std::make_shared<dai::Device>(dai::UsbSpeed::SUPER);
    pipeline_ = std::make_unique<dai::Pipeline>(device_);
    pipeline_->setXLinkChunkSize(0);

    auto mono_left = pipeline_->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_B,
      std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
      static_cast<float>(stereo_fps_));
    auto mono_right = pipeline_->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_C,
      std::make_pair(kOv9282FullWidth, kOv9282FullHeight),
      static_cast<float>(stereo_fps_));
    auto * mono_left_output = mono_left->requestOutput(
      std::make_pair(
        static_cast<std::uint32_t>(stereo_width_),
        static_cast<std::uint32_t>(stereo_height_)),
      dai::ImgFrame::Type::GRAY8,
      dai::ImgResizeMode::CROP,
      static_cast<float>(stereo_fps_));
    auto * mono_right_output = mono_right->requestOutput(
      std::make_pair(
        static_cast<std::uint32_t>(stereo_width_),
        static_cast<std::uint32_t>(stereo_height_)),
      dai::ImgFrame::Type::GRAY8,
      dai::ImgResizeMode::CROP,
      static_cast<float>(stereo_fps_));

    auto stereo = pipeline_->create<dai::node::StereoDepth>();
    stereo->build(
      *mono_left_output,
      *mono_right_output,
      dai::node::StereoDepth::PresetMode::DENSITY);
    // The RGB sensor itself is not started. StereoDepth only uses factory
    // calibration to express depth in the CAM_A/RGB optical coordinate frame.
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    stereo->setOutputSize(stereo_width_, stereo_height_);
    stereo->setOutputKeepAspectRatio(true);
    stereo->setLeftRightCheck(true);
    stereo->setSubpixel(true);
    depth_queue_ = stereo->depth.createOutputQueue(
      static_cast<unsigned int>(depth_queue_size_), false);

    const auto imu_name = device_->getConnectedIMU();
    if (imu_name.empty()) {
      throw std::runtime_error("the OAK device reported no connected IMU");
    }

    const auto calibration = device_->readCalibration();
    const auto imu_to_rgb = calibration.getImuToCameraExtrinsics(
      dai::CameraBoardSocket::CAM_A, false);
    if (
      imu_to_rgb.size() < 3U ||
      imu_to_rgb[0].size() < 3U ||
      imu_to_rgb[1].size() < 3U ||
      imu_to_rgb[2].size() < 3U)
    {
      throw std::runtime_error(
              "factory calibration has no valid IMU-to-RGB rotation");
    }
    for (std::size_t row = 0; row < 3U; ++row) {
      for (std::size_t column = 0; column < 3U; ++column) {
        imu_to_rgb_rotation_[row][column] =
          static_cast<double>(imu_to_rgb[row][column]);
      }
    }

    auto imu = pipeline_->create<dai::node::IMU>();
    imu->enableIMUSensor(
      dai::IMUSensor::ACCELEROMETER_RAW,
      static_cast<int>(std::lround(imu_rate_hz_)));
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    imu_queue_ = imu->out.createOutputQueue(
      static_cast<unsigned int>(imu_queue_size_), false);

    pipeline_->start();
    RCLCPP_INFO(
      get_logger(),
      "OAK opened by this process only: USB=%s, IMU=%s; "
      "active sensors=CAM_B/C+IMU",
      usbSpeedName(device_->getUsbSpeed()), imu_name.c_str());
  }

  void pollOak()
  {
    if (
      state_ == MeasurementState::COMPLETE ||
      state_ == MeasurementState::FAILED)
    {
      return;
    }

    const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() -
      measurement_started_at_).count();
    if (elapsed_sec > measurement_timeout_sec_) {
      failAndRelease(
        "measurement timed out before stable IMU/depth was obtained");
      return;
    }

    try {
      if (!pipeline_ || !pipeline_->isRunning()) {
        failAndRelease("OAK pipeline stopped before height was measured");
        return;
      }

      if (state_ == MeasurementState::WAITING_FOR_STABLE_IMU) {
        pollImu();
      }
      if (state_ == MeasurementState::COLLECTING_DEPTH) {
        pollDepth();
      }
    } catch (const std::exception & exception) {
      failAndRelease(exception.what());
    }
  }

  void pollImu()
  {
    auto data = imu_queue_->tryGet<dai::IMUData>();
    if (!data) {
      return;
    }

    for (const auto & packet : data->packets) {
      const auto & raw = packet.acceleroMeter;
      const std::array<double, 3> acceleration_imu{
        static_cast<double>(raw.x),
        static_cast<double>(raw.y),
        static_cast<double>(raw.z)};
      std::array<double, 3> acceleration_rgb{};
      for (std::size_t row = 0; row < 3U; ++row) {
        for (std::size_t column = 0; column < 3U; ++column) {
          acceleration_rgb[row] +=
            imu_to_rgb_rotation_[row][column] *
            acceleration_imu[column];
        }
      }

      const double magnitude = std::sqrt(
        acceleration_rgb[0] * acceleration_rgb[0] +
        acceleration_rgb[1] * acceleration_rgb[1] +
        acceleration_rgb[2] * acceleration_rgb[2]);
      if (
        !std::isfinite(magnitude) ||
        magnitude < imu_accel_min_mps2_ ||
        magnitude > imu_accel_max_mps2_)
      {
        imu_direction_samples_.clear();
        continue;
      }

      imu_direction_samples_.push_back(normalized(acceleration_rgb));
      while (
        static_cast<int>(imu_direction_samples_.size()) > imu_sample_count_)
      {
        imu_direction_samples_.pop_front();
      }
    }

    if (
      static_cast<int>(imu_direction_samples_.size()) < imu_sample_count_)
    {
      return;
    }

    std::array<double, 3> mean{0.0, 0.0, 0.0};
    for (const auto & sample : imu_direction_samples_) {
      for (std::size_t axis = 0; axis < 3U; ++axis) {
        mean[axis] += sample[axis];
      }
    }
    mean = normalized(mean);

    double squared_angle_sum = 0.0;
    for (const auto & sample : imu_direction_samples_) {
      const double angle_deg = clampedAcos(
        sample[0] * mean[0] +
        sample[1] * mean[1] +
        sample[2] * mean[2]) * kRadiansToDegrees;
      squared_angle_sum += angle_deg * angle_deg;
    }
    const double direction_rms_deg = std::sqrt(
      squared_angle_sum /
      static_cast<double>(imu_direction_samples_.size()));
    if (direction_rms_deg > imu_max_direction_rms_deg_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for stationary IMU: direction RMS=%.3fdeg, limit=%.3fdeg",
        direction_rms_deg, imu_max_direction_rms_deg_);
      return;
    }

    frozen_specific_force_ = mean;
    state_ = MeasurementState::COLLECTING_DEPTH;

    const double roll_deg =
      std::atan2(-mean[0], -mean[1]) * kRadiansToDegrees;
    const double pitch_down_deg = std::atan2(
      -mean[2], std::hypot(mean[0], mean[1])) * kRadiansToDegrees;
    RCLCPP_INFO(
      get_logger(),
      "IMU gravity fixed: samples=%d, roll=%.3fdeg, "
      "pitch_down=%.3fdeg, direction_RMS=%.3fdeg",
      imu_sample_count_, roll_deg, pitch_down_deg, direction_rms_deg);
  }

  void pollDepth()
  {
    auto packet = depth_queue_->tryGet<dai::ImgFrame>();
    if (!packet) {
      return;
    }
    if (
      packet->getType() != dai::ImgFrame::Type::RAW16 ||
      static_cast<int>(packet->getWidth()) != stereo_width_ ||
      static_cast<int>(packet->getHeight()) != stereo_height_)
    {
      throw std::runtime_error(
              "DepthAI output is not the requested RAW16 depth geometry");
    }

    const auto & transformation = packet->getTransformation();
    if (!transformation.isValid()) {
      throw std::runtime_error(
              "RGB-aligned depth output has no valid transformation");
    }
    const auto intrinsic_matrix = transformation.getIntrinsicMatrix();
    const CameraIntrinsics intrinsics{
      static_cast<double>(intrinsic_matrix[0][0]),
      static_cast<double>(intrinsic_matrix[1][1]),
      static_cast<double>(intrinsic_matrix[0][2]),
      static_cast<double>(intrinsic_matrix[1][2])};

    const auto & bytes = packet->getData();
    const std::size_t minimum_stride =
      static_cast<std::size_t>(stereo_width_) * sizeof(std::uint16_t);
    const std::size_t stride = std::max(
      minimum_stride, static_cast<std::size_t>(packet->getStride()));
    if (
      bytes.size() <
      stride * static_cast<std::size_t>(stereo_height_))
    {
      throw std::runtime_error("DepthAI returned an undersized depth frame");
    }

    const int start_u = (stereo_width_ - roi_width_) / 2;
    const int start_v = (stereo_height_ - roi_height_) / 2;
    std::vector<DepthSample> samples;
    samples.reserve(static_cast<std::size_t>(roi_width_ * roi_height_));
    for (int v = start_v; v < start_v + roi_height_; ++v) {
      const auto row_offset = static_cast<std::size_t>(v) * stride;
      for (int u = start_u; u < start_u + roi_width_; ++u) {
        std::uint16_t depth_mm = 0U;
        std::memcpy(
          &depth_mm,
          bytes.data() + row_offset +
          static_cast<std::size_t>(u) * sizeof(depth_mm),
          sizeof(depth_mm));
        if (depth_mm == 0U) {
          continue;
        }
        samples.push_back(DepthSample{
          static_cast<double>(u),
          static_cast<double>(v),
          static_cast<double>(depth_mm) * 0.001});
      }
    }

    HeightDiagnostics diagnostics;
    const auto estimate = estimateCameraHeight(
      samples, intrinsics, frozen_specific_force_, geometry_config_,
      &diagnostics);
    if (!estimate) {
      stable_height_samples_.clear();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected center ROI: %s; nonzero=%zu, usable=%zu/%d, "
        "candidate_height=%.4fm, depth_median=%.3fm, "
        "height_MAD=%.4fm, downward_component=%.3f",
        heightRejectionName(diagnostics.rejection),
        samples.size(), diagnostics.valid_pixel_count,
        minimum_valid_pixels_, diagnostics.candidate_height_m,
        diagnostics.median_depth_m, diagnostics.height_mad_m,
        diagnostics.downward_ray_component);
      return;
    }

    stable_height_samples_.push_back(estimate->height_m);
    while (
      static_cast<int>(stable_height_samples_.size()) >
      stable_depth_frame_count_)
    {
      stable_height_samples_.pop_front();
    }
    if (
      static_cast<int>(stable_height_samples_.size()) <
      stable_depth_frame_count_)
    {
      return;
    }

    double mean_height_m = 0.0;
    for (const double height_m : stable_height_samples_) {
      mean_height_m += height_m;
    }
    mean_height_m /=
      static_cast<double>(stable_height_samples_.size());

    double squared_error_sum = 0.0;
    for (const double height_m : stable_height_samples_) {
      const double error_m = height_m - mean_height_m;
      squared_error_sum += error_m * error_m;
    }
    const double height_stddev_m = std::sqrt(
      squared_error_sum /
      static_cast<double>(stable_height_samples_.size()));
    if (height_stddev_m > maximum_height_stddev_m_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Waiting for stable height: stddev=%.4fm, limit=%.4fm",
        height_stddev_m, maximum_height_stddev_m_);
      return;
    }

    std_msgs::msg::Float64 output;
    output.data = mean_height_m;
    height_publisher_->publish(output);
    state_ = MeasurementState::COMPLETE;

    RCLCPP_INFO(
      get_logger(),
      "CAMERA_HEIGHT_RESULT_M=%.4f", mean_height_m);
    RCLCPP_INFO(
      get_logger(),
      "Height finalized: center_depth=%.3fm, valid=%zu/%d, "
      "spatial_MAD=%.4fm, temporal_stddev=%.4fm",
      estimate->median_depth_m,
      estimate->valid_pixel_count, roi_width_ * roi_height_,
      estimate->height_mad_m, height_stddev_m);

    releaseOak();
    RCLCPP_INFO(
      get_logger(),
      "OAK pipeline and USB connection released. The node is now idle and "
      "only retains the latched %s result.",
      output_topic_.c_str());
  }

  void failAndRelease(const std::string & reason)
  {
    state_ = MeasurementState::FAILED;
    RCLCPP_ERROR(
      get_logger(), "Camera-height measurement failed: %s", reason.c_str());
    releaseOak();
    RCLCPP_INFO(
      get_logger(), "OAK pipeline and USB connection released after failure.");
  }

  void releaseOak()
  {
    if (poll_timer_) {
      poll_timer_->cancel();
    }
    if (pipeline_) {
      try {
        if (pipeline_->isRunning()) {
          pipeline_->stop();
        }
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          get_logger(), "Error while stopping OAK: %s", exception.what());
      }
    }
    depth_queue_.reset();
    imu_queue_.reset();
    pipeline_.reset();
    device_.reset();
  }

  std::string output_topic_;
  double stereo_fps_{30.0};
  int stereo_width_{640};
  int stereo_height_{400};
  int depth_queue_size_{2};
  double imu_rate_hz_{100.0};
  int imu_queue_size_{20};

  int roi_width_{10};
  int roi_height_{10};
  int minimum_valid_pixels_{25};
  double minimum_depth_m_{0.30};
  double maximum_depth_m_{3.00};
  double minimum_height_m_{0.10};
  double maximum_height_m_{1.00};
  double maximum_height_mad_m_{0.015};
  double minimum_downward_ray_component_{0.05};

  int imu_sample_count_{10};
  double imu_max_direction_rms_deg_{0.50};
  double imu_accel_min_mps2_{7.50};
  double imu_accel_max_mps2_{12.00};

  int stable_depth_frame_count_{5};
  double maximum_height_stddev_m_{0.010};
  double measurement_timeout_sec_{10.0};
  HeightGeometryConfig geometry_config_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_publisher_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  std::shared_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  std::shared_ptr<dai::MessageQueue> depth_queue_;
  std::shared_ptr<dai::MessageQueue> imu_queue_;

  std::array<std::array<double, 3>, 3> imu_to_rgb_rotation_{{
    {{1.0, 0.0, 0.0}},
    {{0.0, 1.0, 0.0}},
    {{0.0, 0.0, 1.0}}}};
  std::deque<std::array<double, 3>> imu_direction_samples_;
  std::array<double, 3> frozen_specific_force_{0.0, -1.0, 0.0};
  std::deque<double> stable_height_samples_;
  MeasurementState state_{MeasurementState::WAITING_FOR_STABLE_IMU};
  std::chrono::steady_clock::time_point measurement_started_at_;
};

}  // namespace camera_height_estimator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(
      std::make_shared<
        camera_height_estimator::CameraHeightEstimatorNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("camera_height_estimator"),
      "Independent height estimator terminated: %s", exception.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
