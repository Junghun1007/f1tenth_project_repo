#include "manual_capture/camera_engine.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

namespace manual_capture
{

using namespace std::chrono_literals;

namespace
{

constexpr std::uint32_t kSensorWidth = 1280U;
constexpr std::uint32_t kSensorHeight = 800U;
constexpr double kMaximumStereoFps = 50.0;
constexpr double kMaximumMonoFps = 120.0;
constexpr double kMaximumRgbFps = 60.0;

double seconds(const std::chrono::steady_clock::time_point & time)
{
  return std::chrono::duration<double>(time.time_since_epoch()).count();
}

template<typename Matrix>
cv::Matx33d rotationFrom(const Matrix & matrix, const char * name)
{
  if (matrix.size() < 3U || matrix[0].size() < 3U || matrix[1].size() < 3U ||
    matrix[2].size() < 3U)
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

// Actual output-frame intrinsics and the lens pose in CAM_A coordinates,
// composed the same way as the IR BEV (frame metadata already contains any
// rectification; never apply the EEPROM rectification twice).
LensCalibration lensFromFrame(
  dai::ImgFrame & frame, const dai::CalibrationHandler & calibration, const bool is_rgb)
{
  const auto & transformation = frame.getTransformation();
  if (!transformation.isValid()) {
    throw std::runtime_error("frame has no intrinsics metadata");
  }
  const auto intrinsics = transformation.getIntrinsicMatrix();
  LensCalibration lens;
  lens.fx = intrinsics[0][0];
  lens.fy = intrinsics[1][1];
  lens.cx = intrinsics[0][2];
  lens.cy = intrinsics[1][2];
  lens.width = static_cast<int>(frame.getWidth());
  lens.height = static_cast<int>(frame.getHeight());

  const auto extrinsics = transformation.getExtrinsics();
  const auto reference = extrinsics.toCameraSocket;
  if (reference == dai::CameraBoardSocket::AUTO) {
    if (is_rgb) {
      return lens;  // CAM_A itself: identity pose
    }
    throw std::runtime_error("lens extrinsics have no reference camera");
  }
  const auto view_to_reference = extrinsics.getTransformationMatrix(false, dai::LengthUnit::METER);
  const auto rotation_reference_from_view = rotationFrom(view_to_reference, "view-to-reference");
  cv::Matx33d rotation_rgb_from_reference = cv::Matx33d::eye();
  cv::Vec3d reference_center_rgb_m(0.0, 0.0, 0.0);
  if (reference != dai::CameraBoardSocket::CAM_A) {
    const auto reference_to_rgb = calibration.getCameraExtrinsics(
      reference, dai::CameraBoardSocket::CAM_A, false);
    rotation_rgb_from_reference = rotationFrom(reference_to_rgb, "reference-to-CAM_A");
    if (reference_to_rgb[0].size() < 4U || reference_to_rgb[1].size() < 4U ||
      reference_to_rgb[2].size() < 4U)
    {
      throw std::runtime_error("reference-to-CAM_A translation is missing");
    }
    // CalibrationHandler translation unit is centimeters.
    reference_center_rgb_m = cv::Vec3d(
      reference_to_rgb[0][3], reference_to_rgb[1][3], reference_to_rgb[2][3]) * 0.01;
  }
  lens.rotation_rgb_from_view = rotation_rgb_from_reference * rotation_reference_from_view;
  lens.view_center_rgb_m = reference_center_rgb_m + rotation_rgb_from_reference *
    cv::Vec3d(view_to_reference[0][3], view_to_reference[1][3], view_to_reference[2][3]);
  return lens;
}

void validate(const CameraConfig & config)
{
  const auto even_size = [](int width, int height) {
      return width > 0 && height > 0 && width <= static_cast<int>(kSensorWidth) &&
             height <= static_cast<int>(kSensorHeight) && width % 2 == 0 && height % 2 == 0;
    };
  const double stereo_limit = config.stereo_mode == StereoMode::LEFT_RIGHT ?
    kMaximumStereoFps : kMaximumMonoFps;
  if (!even_size(config.rgb_width, config.rgb_height) ||
    !even_size(config.stereo_width, config.stereo_height) ||
    !std::isfinite(config.rgb_fps) || config.rgb_fps <= 0.0 || config.rgb_fps > kMaximumRgbFps ||
    !std::isfinite(config.stereo_fps) || config.stereo_fps <= 0.0 ||
    config.stereo_fps > stereo_limit ||
    !std::isfinite(config.virtual_camera_position_ratio) ||
    config.virtual_camera_position_ratio < 0.0 || config.virtual_camera_position_ratio > 1.0 ||
    !std::isfinite(config.sync_threshold_ms) || config.sync_threshold_ms <= 0.0)
  {
    throw std::invalid_argument("invalid camera configuration");
  }
}

std::shared_ptr<dai::CameraControl> exposureControl(const ExposureSetting & setting)
{
  auto control = std::make_shared<dai::CameraControl>();
  if (setting.automatic) {
    control->setAutoExposureEnable();
  } else {
    control->setManualExposure(
      static_cast<std::uint32_t>(std::max(1, setting.exposure_us)),
      static_cast<std::uint32_t>(std::clamp(setting.iso, 100, 1600)));
  }
  return control;
}

void applyInitialExposure(dai::node::Camera & camera, const ExposureSetting & setting)
{
  if (setting.automatic) {
    camera.initialControl.setAutoExposureEnable();
  } else {
    camera.initialControl.setManualExposure(
      static_cast<std::uint32_t>(std::max(1, setting.exposure_us)),
      static_cast<std::uint32_t>(std::clamp(setting.iso, 100, 1600)));
  }
}

ExposureSetting limitStereoExposure(ExposureSetting setting, double fps)
{
  setting.exposure_us = std::clamp(setting.exposure_us, 20, maximumExposureUs(fps));
  setting.iso = std::clamp(setting.iso, 100, 1600);
  return setting;
}

void limitBridge(dai::Node::Output * output, bool metadata_only = false)
{
  const auto bridge = output->getXLinkBridge();
  if (!bridge || !bridge->xLinkOut) {
    throw std::runtime_error("DepthAI did not create an XLink bridge");
  }
  bridge->xLinkOut->input.setMaxSize(1);
  bridge->xLinkOut->input.setBlocking(false);
  if (metadata_only) {
    bridge->xLinkOut->setMetadataOnly(true);
  }
}

class RateMeter
{
public:
  void tick(double now)
  {
    if (last_ > 0.0 && now > last_) {
      const double instant = 1.0 / (now - last_);
      const double previous = fps_.load();
      fps_.store(previous <= 0.0 ? instant : 0.9 * previous + 0.1 * instant);
    }
    last_ = now;
  }
  void reset()
  {
    last_ = 0.0;
    fps_.store(0.0);
  }
  double fps() const {return fps_.load();}

private:
  double last_{0.0};
  std::atomic<double> fps_{0.0};
};

}  // namespace

struct CameraEngine::Impl
{
  CameraConfig config;
  LiveControls controls;
  std::string device_id;
  std::atomic<std::uint64_t> generation{0U};
  std::atomic<bool> running{false};
  std::atomic<bool> stop_requested{false};

  std::shared_ptr<dai::Device> device;
  std::unique_ptr<dai::Pipeline> pipeline;
  std::unique_ptr<dai::CalibrationHandler> calibration;
  std::shared_ptr<dai::MessageQueue> rgb_queue;
  std::shared_ptr<dai::MessageQueue> stereo_queue;
  std::array<std::shared_ptr<dai::MessageQueue>, 2> stereo_metadata_queues;
  std::shared_ptr<dai::InputQueue> rgb_control;
  std::shared_ptr<dai::InputQueue> left_control;
  std::shared_ptr<dai::InputQueue> right_control;
  std::thread rgb_thread;
  std::thread stereo_thread;

  // IMU (pitch/roll correction of the BEV, same estimator as camera_driver).
  std::shared_ptr<ImuTiltTracker> imu_tracker;
  ImuObserver imu_observer;
  std::shared_ptr<dai::MessageQueue> imu_queue;
  std::thread imu_thread;
  cv::Matx33d imu_output_to_camera{cv::Matx33d::eye()};
  std::string imu_error{"IMU not started"};

  mutable std::mutex mutex;  // frames, calibrations, controls, error
  std::shared_ptr<const RgbFrame> latest_rgb;
  std::shared_ptr<const StereoFrame> latest_stereo;
  std::array<StereoExposureReadback, 2> stereo_readback;
  std::optional<LensCalibration> rgb_calibration;
  std::optional<LensCalibration> left_calibration;
  std::string last_error;
  RateMeter rgb_rate;
  RateMeter stereo_rate;

  void setError(const std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    last_error = message;
  }

  void rgbLoop()
  {
    std::uint64_t sequence = 0U;
    while (!stop_requested.load()) {
      try {
        auto frame = rgb_queue->tryGet<dai::ImgFrame>();
        if (!frame) {
          std::this_thread::sleep_for(200us);
          continue;
        }
        auto output = std::make_shared<RgbFrame>();
        output->bgr = frame->getCvFrame();  // NV12/RGB888i -> owned BGR copy
        if (output->bgr.empty() || output->bgr.type() != CV_8UC3) {
          continue;
        }
        output->sequence = ++sequence;
        output->timestamp_sec = seconds(frame->getTimestamp(dai::CameraExposureOffset::MIDDLE));
        output->exposure_us = static_cast<int>(frame->getExposureTime().count());
        output->iso = frame->getSensitivity();
        if (imu_tracker) {output->tilt = imu_tracker->tiltAt(output->timestamp_sec);}
        rgb_rate.tick(output->timestamp_sec);
        std::optional<LensCalibration> lens;
        {
          std::lock_guard<std::mutex> lock(mutex);
          if (!rgb_calibration) {
            lens = lensFromFrame(*frame, *calibration, true);
          }
        }
        std::lock_guard<std::mutex> lock(mutex);
        if (lens) {rgb_calibration = lens;}
        latest_rgb = std::move(output);
      } catch (const std::exception & exception) {
        setError(std::string("RGB: ") + exception.what());
        std::this_thread::sleep_for(5ms);
      }
    }
  }

  void readExposure(std::size_t lens, const dai::ImgFrame & frame)
  {
    const ExposureReading reading{
      static_cast<int>(frame.getExposureTime().count()), frame.getSensitivity(),
      seconds(frame.getTimestamp(dai::CameraExposureOffset::START))};
    std::lock_guard<std::mutex> lock(mutex);
    stereo_readback[lens].latest = reading;
  }

  void stereoLoop()
  {
    std::uint64_t sequence = 0U;
    const bool both = config.stereo_mode == StereoMode::LEFT_RIGHT;
    while (!stop_requested.load()) {
      try {
        auto output = std::make_shared<StereoFrame>();
        std::shared_ptr<dai::ImgFrame> left;
        if (both) {
          for (std::size_t lens = 0; lens < stereo_metadata_queues.size(); ++lens) {
            if (const auto frame = stereo_metadata_queues[lens]->tryGet<dai::ImgFrame>()) {
              readExposure(lens, *frame);
            }
          }
          auto group = stereo_queue->tryGet<dai::MessageGroup>();
          if (!group) {
            std::this_thread::sleep_for(200us);
            continue;
          }
          left = group->get<dai::ImgFrame>("left");
          auto right = group->get<dai::ImgFrame>("right");
          auto disparity = group->get<dai::ImgFrame>("disparity");
          if (!left || !right || !disparity) {
            continue;
          }
          output->right = right->getFrame(false).clone();
          output->disparity = disparity->getFrame(false).clone();
          output->right_exposure_us = static_cast<int>(right->getExposureTime().count());
          output->right_iso = right->getSensitivity();
        } else {
          left = stereo_queue->tryGet<dai::ImgFrame>();
          if (!left) {
            std::this_thread::sleep_for(200us);
            continue;
          }
          readExposure(0, *left);
        }
        output->left = left->getFrame(false).clone();
        if (output->left.empty() || output->left.type() != CV_8UC1) {
          continue;
        }
        output->sequence = ++sequence;
        output->timestamp_sec = seconds(left->getTimestamp(dai::CameraExposureOffset::MIDDLE));
        output->left_exposure_us = static_cast<int>(left->getExposureTime().count());
        output->left_iso = left->getSensitivity();
        if (imu_tracker) {output->tilt = imu_tracker->tiltAt(output->timestamp_sec);}
        stereo_rate.tick(output->timestamp_sec);
        std::optional<LensCalibration> lens;
        if (!both) {
          std::lock_guard<std::mutex> lock(mutex);
          if (!left_calibration) {
            lens = lensFromFrame(*left, *calibration, false);
          }
        }
        std::lock_guard<std::mutex> lock(mutex);
        if (lens) {left_calibration = lens;}
        latest_stereo = std::move(output);
      } catch (const std::exception & exception) {
        setError(std::string("Stereo: ") + exception.what());
        std::this_thread::sleep_for(5ms);
      }
    }
  }

  void imuLoop()
  {
    std::vector<ImuSample> samples;
    while (!stop_requested.load()) {
      try {
        auto data = imu_queue->tryGet<dai::IMUData>();
        if (!data) {
          std::this_thread::sleep_for(200us);
          continue;
        }
        const double received = seconds(std::chrono::steady_clock::now());
        samples.clear();
        for (const auto & packet : data->packets) {
          const auto & a = packet.acceleroMeter;
          const auto & g = packet.gyroscope;
          ImuSample sample;
          sample.acceleration_camera_mps2 = imu_output_to_camera * cv::Vec3d(a.x, a.y, a.z);
          sample.angular_velocity_camera_radps = imu_output_to_camera * cv::Vec3d(g.x, g.y, g.z);
          sample.acceleration_timestamp_sec = seconds(a.getTimestamp());
          sample.gyroscope_timestamp_sec = seconds(g.getTimestamp());
          samples.push_back(sample);
        }
        if (imu_tracker) {
          imu_tracker->processBatch(samples, received);
        }
        if (imu_observer) {
          for (const auto & sample : samples) {
            imu_observer(sample);
          }
        }
      } catch (const std::exception & exception) {
        setError(std::string("IMU: ") + exception.what());
        std::this_thread::sleep_for(5ms);
      }
    }
  }

  // Same frame conversion as camera_driver: CALIBRATED reports already contain the
  // EEPROM IMU output rotation, so apply only the rest of the IMU -> CAM_A rotation.
  void buildImu()
  {
    const bool wanted = (imu_tracker && imu_tracker->config().enabled) || imu_observer;
    if (!wanted) {
      std::lock_guard<std::mutex> lock(mutex);
      imu_error = "IMU stabilization disabled";
      return;
    }
    try {
      if (device->getConnectedIMU().empty()) {
        throw std::runtime_error("the OAK device reported no connected IMU");
      }
      const auto imu_to_camera = rotationFrom(
        calibration->getImuToCameraExtrinsics(dai::CameraBoardSocket::CAM_A, false),
        "IMU-to-CAM_A");
      const auto imu_to_output = rotationFrom(
        calibration->getEepromData().imuExtrinsics.rotationMatrix, "calibrated IMU output");
      imu_output_to_camera = imu_to_camera * imu_to_output.t();
      const auto & imu_config = imu_tracker ? imu_tracker->config() : ImuTiltTrackerConfig{};
      auto imu = pipeline->create<dai::node::IMU>();
      imu->enableIMUSensor(
        {dai::IMUSensor::ACCELEROMETER_CALIBRATED, dai::IMUSensor::GYROSCOPE_CALIBRATED},
        static_cast<int>(std::lround(imu_config.imu_rate_hz)));
      imu->setBatchReportThreshold(1);
      imu->setMaxBatchReports(imu_config.imu_max_batch_reports);
      imu_queue = imu->out.createOutputQueue(
        static_cast<unsigned int>(imu_config.imu_queue_size), false);
      std::lock_guard<std::mutex> lock(mutex);
      imu_error.clear();
    } catch (const std::exception & exception) {
      // Capture keeps working with the fixed mount pose.
      imu_queue.reset();
      std::lock_guard<std::mutex> lock(mutex);
      imu_error = exception.what();
    }
  }

  void build()
  {
    device = config.device_id.empty() ?
      std::make_shared<dai::Device>(dai::UsbSpeed::SUPER) :
      std::make_shared<dai::Device>(dai::DeviceInfo(config.device_id), dai::UsbSpeed::SUPER);
    device_id = device->getDeviceId();
    calibration = std::make_unique<dai::CalibrationHandler>(device->getCalibration());
    pipeline = std::make_unique<dai::Pipeline>(device);
    // Never let a capture tool rewrite the device EEPROM.
    pipeline->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
    pipeline->setXLinkChunkSize(0);

    auto rgb_camera = pipeline->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_A, std::make_pair(kSensorWidth, kSensorHeight),
      static_cast<float>(config.rgb_fps));
    applyInitialExposure(*rgb_camera, controls.rgb);
    auto * rgb_output = rgb_camera->requestOutput(
      std::make_pair(
        static_cast<std::uint32_t>(config.rgb_width),
        static_cast<std::uint32_t>(config.rgb_height)),
      config.rgb_format == RgbFormat::NV12 ? dai::ImgFrame::Type::NV12 :
      dai::ImgFrame::Type::RGB888i,
      dai::ImgResizeMode::CROP, static_cast<float>(config.rgb_fps), config.rgb_undistort);
    rgb_control = rgb_camera->inputControl.createInputQueue();

    const auto stereo_size = std::make_pair(
      static_cast<std::uint32_t>(config.stereo_width),
      static_cast<std::uint32_t>(config.stereo_height));
    const float stereo_fps = static_cast<float>(config.stereo_fps);
    auto left_camera = pipeline->create<dai::node::Camera>()->build(
      dai::CameraBoardSocket::CAM_B, std::make_pair(kSensorWidth, kSensorHeight), stereo_fps);
    applyInitialExposure(*left_camera, controls.left);
    left_control = left_camera->inputControl.createInputQueue(1U, false);

    dai::Node::Output * stereo_output = nullptr;
    std::array<dai::Node::Output *, 2> raw_outputs{};
    if (config.stereo_mode == StereoMode::LEFT_RIGHT) {
      auto right_camera = pipeline->create<dai::node::Camera>()->build(
        dai::CameraBoardSocket::CAM_C, std::make_pair(kSensorWidth, kSensorHeight), stereo_fps);
      applyInitialExposure(*right_camera, controls.right);
      right_control = right_camera->inputControl.createInputQueue(1U, false);
      auto * left_raw = left_camera->requestOutput(
        stereo_size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, stereo_fps);
      auto * right_raw = right_camera->requestOutput(
        stereo_size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, stereo_fps);
      // Independent exposure readback even when a synchronized depth group is missing.
      raw_outputs = {left_raw, right_raw};
      for (std::size_t lens = 0; lens < raw_outputs.size(); ++lens) {
        stereo_metadata_queues[lens] = raw_outputs[lens]->createOutputQueue(1U, false);
      }

      // Same light center-aligned disparity setup as the IR center preview.
      auto stereo = pipeline->create<dai::node::StereoDepth>();
      stereo->build(*left_raw, *right_raw, dai::node::StereoDepth::PresetMode::FAST_DENSITY);
      stereo->setRectification(true);
      stereo->enableDistortionCorrection(true);
      stereo->setOutputSize(config.stereo_width, config.stereo_height);
      stereo->setOutputKeepAspectRatio(true);
      stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::CENTER);
      stereo->initialConfig->algorithmControl.centerAlignmentShiftFactor =
        static_cast<float>(config.virtual_camera_position_ratio);
      stereo->setSubpixel(false);
      stereo->setExtendedDisparity(false);
      stereo->setLeftRightCheck(true);
      stereo->initialConfig->postProcessing.median =
        dai::node::StereoDepth::MedianFilter::MEDIAN_OFF;
      stereo->initialConfig->postProcessing.holeFilling.enable = false;
      stereo->initialConfig->postProcessing.adaptiveMedianFilter.enable = false;
      stereo->initialConfig->postProcessing.spatialFilter.enable = false;
      stereo->initialConfig->postProcessing.temporalFilter.enable = false;
      stereo->initialConfig->postProcessing.speckleFilter.enable = false;
      stereo->setRectifyEdgeFillColor(0);
      stereo->setFrameSync(true);

      auto sync = pipeline->create<dai::node::Sync>();
      sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double, std::milli>(config.sync_threshold_ms)));
      sync->setSyncAttempts(-1);
      stereo->rectifiedLeft.link(sync->inputs["left"]);
      stereo->rectifiedRight.link(sync->inputs["right"]);
      stereo->disparity.link(sync->inputs["disparity"]);
      stereo_output = &sync->out;
    } else {
      // Left lens alone, undistorted on the device for the metric left BEV.
      stereo_output = left_camera->requestOutput(
        stereo_size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, stereo_fps, true);
    }

    rgb_queue = rgb_output->createOutputQueue(1U, false);
    stereo_queue = stereo_output->createOutputQueue(1U, false);
    buildImu();
    pipeline->build();
    limitBridge(rgb_output);
    limitBridge(stereo_output);
    for (auto * raw : raw_outputs) {
      if (raw) {
        limitBridge(raw, true);  // keep the raw images on the device
      }
    }
    pipeline->start();
  }

  void applyIr()
  {
    if (!device) {
      return;
    }
    device->setIrFloodLightIntensity(static_cast<float>(std::clamp(controls.flood, 0.0, 1.0)));
    device->setIrLaserDotProjectorIntensity(
      static_cast<float>(std::clamp(controls.dot, 0.0, 1.0)));
  }

  void shutdown()
  {
    stop_requested.store(true);
    if (rgb_thread.joinable()) {rgb_thread.join();}
    if (stereo_thread.joinable()) {stereo_thread.join();}
    if (imu_thread.joinable()) {imu_thread.join();}
    if (imu_tracker) {imu_tracker->setImuStreaming(false);}
    if (device) {
      try {
        device->setIrLaserDotProjectorIntensity(0.0F);
        device->setIrFloodLightIntensity(0.0F);
      } catch (...) {
      }
    }
    rgb_queue.reset();
    stereo_queue.reset();
    imu_queue.reset();
    stereo_metadata_queues = {};
    rgb_control.reset();
    left_control.reset();
    right_control.reset();
    if (pipeline) {
      try {
        if (pipeline->isRunning()) {
          pipeline->stop();
          pipeline->wait();
        }
      } catch (...) {
      }
      pipeline.reset();
    }
    calibration.reset();
    device.reset();
    running.store(false);
  }
};

CameraEngine::CameraEngine()
: impl_(std::make_unique<Impl>())
{
}

CameraEngine::~CameraEngine()
{
  stop();
}

void CameraEngine::start(const CameraConfig & config, const LiveControls & controls)
{
  validate(config);
  stop();
  auto & impl = *impl_;
  {
    std::lock_guard<std::mutex> lock(impl.mutex);
    impl.config = config;
    impl.controls = controls;
    impl.controls.left = limitStereoExposure(controls.left, config.stereo_fps);
    impl.controls.right = limitStereoExposure(controls.right, config.stereo_fps);
    impl.latest_rgb.reset();
    impl.latest_stereo.reset();
    impl.stereo_readback = {};
    impl.rgb_calibration.reset();
    impl.left_calibration.reset();
    impl.last_error.clear();
  }
  impl.rgb_rate.reset();
  impl.stereo_rate.reset();
  impl.stop_requested.store(false);
  try {
    impl.build();
    impl.applyIr();
  } catch (...) {
    impl.shutdown();
    throw;
  }
  impl.config.device_id = impl.device_id;  // reopen the same device next time
  {
    std::lock_guard<std::mutex> lock(impl.mutex);
    const double now = seconds(std::chrono::steady_clock::now());
    for (auto & readback : impl.stereo_readback) {
      readback.sent_at_sec = now;
    }
  }
  impl.running.store(true);
  impl.generation.fetch_add(1U);
  impl.rgb_thread = std::thread(&Impl::rgbLoop, &impl);
  impl.stereo_thread = std::thread(&Impl::stereoLoop, &impl);
  if (impl.imu_queue) {
    if (impl.imu_tracker) {impl.imu_tracker->setImuStreaming(true);}
    impl.imu_thread = std::thread(&Impl::imuLoop, &impl);
  }
}

void CameraEngine::setImu(std::shared_ptr<ImuTiltTracker> tracker, ImuObserver observer)
{
  if (impl_->running.load()) {
    throw std::logic_error("setImu must be called before start");
  }
  impl_->imu_tracker = std::move(tracker);
  impl_->imu_observer = std::move(observer);
}

std::string CameraEngine::imuError() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->imu_error;
}

void CameraEngine::stop()
{
  impl_->shutdown();
}

bool CameraEngine::running() const {return impl_->running.load();}
const CameraConfig & CameraEngine::config() const {return impl_->config;}
std::string CameraEngine::deviceId() const {return impl_->device_id;}
std::uint64_t CameraEngine::generation() const {return impl_->generation.load();}

std::shared_ptr<const RgbFrame> CameraEngine::latestRgb() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->latest_rgb;
}

std::shared_ptr<const StereoFrame> CameraEngine::latestStereo() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->latest_stereo;
}

std::optional<LensCalibration> CameraEngine::rgbCalibration() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->rgb_calibration;
}

std::optional<LensCalibration> CameraEngine::leftCalibration() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->left_calibration;
}

void CameraEngine::setRgbExposure(const ExposureSetting & setting)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->controls.rgb = setting;
  if (impl_->running.load() && impl_->rgb_control) {
    impl_->rgb_control->send(exposureControl(setting));
  }
}

void CameraEngine::setStereoExposure(const ExposureSetting & left, const ExposureSetting & right)
{
  auto & impl = *impl_;
  std::lock_guard<std::mutex> lock(impl.mutex);
  const auto send = [&](std::size_t lens, const std::shared_ptr<dai::InputQueue> & queue,
      const ExposureSetting & request, ExposureSetting & applied) {
      auto & readback = impl.stereo_readback[lens];
      if (!impl.running.load() || !queue) {
        readback.send_error = "camera control queue unavailable";
        return false;
      }
      try {
        queue->send(exposureControl(request));
      } catch (const std::exception & error) {
        readback.send_error = error.what();
        return false;
      }
      applied = request;
      readback.sent_at_sec = seconds(std::chrono::steady_clock::now());
      readback.send_error.clear();
      return true;
    };
  const auto left_request = limitStereoExposure(left, impl.config.stereo_fps);
  const auto right_request = limitStereoExposure(right, impl.config.stereo_fps);
  const bool left_sent = send(0, impl.left_control, left_request, impl.controls.left);
  if (impl.config.stereo_mode == StereoMode::LEFT_RIGHT) {
    send(1, impl.right_control, right_request, impl.controls.right);
  } else if (left_sent) {
    // Keep the selected mode and linked values for when CAM_C is started again.
    impl.controls.right = right_request;
  }
}

std::array<StereoExposureReadback, 2> CameraEngine::stereoExposureReadback() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->stereo_readback;
}

void CameraEngine::setFlood(const double intensity)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->controls.flood = std::clamp(intensity, 0.0, 1.0);
  if (impl_->running.load()) {impl_->applyIr();}
}

void CameraEngine::setDot(const double intensity)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->controls.dot = std::clamp(intensity, 0.0, 1.0);
  if (impl_->running.load()) {impl_->applyIr();}
}

LiveControls CameraEngine::controls() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->controls;
}

double CameraEngine::measuredRgbFps() const {return impl_->rgb_rate.fps();}
double CameraEngine::measuredStereoFps() const {return impl_->stereo_rate.fps();}

std::string CameraEngine::lastError() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->last_error;
}

}  // namespace manual_capture
