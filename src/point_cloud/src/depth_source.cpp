#include "point_cloud/depth_source.hpp"

#include <cmath>
#include <stdexcept>

namespace point_cloud
{
std::pair<std::uint32_t, std::uint32_t> resolutionSize(const std::string & value)
{
  if (value == "400p") {return {640, 400};}
  if (value == "480p") {return {640, 480};}
  if (value == "720p") {return {1280, 720};}
  if (value == "800p") {return {1280, 800};}
  throw std::invalid_argument("camera.resolution: choose 400p, 480p, 720p or 800p");
}

namespace
{
dai::node::StereoDepth::PresetMode preset(const std::string & value)
{
  using Mode = dai::node::StereoDepth::PresetMode;
  if (value == "default") {return Mode::DEFAULT;}
  if (value == "high_density") {return Mode::FAST_DENSITY;}
  if (value == "high_accuracy") {return Mode::FAST_ACCURACY;}
  throw std::invalid_argument("depth.mode: choose default, high_density or high_accuracy");
}

dai::StereoDepthConfig::MedianFilter median(const std::string & value)
{
  using Filter = dai::StereoDepthConfig::MedianFilter;
  if (value == "off") {return Filter::MEDIAN_OFF;}
  if (value == "3x3") {return Filter::KERNEL_3x3;}
  if (value == "5x5") {return Filter::KERNEL_5x5;}
  if (value == "7x7") {return Filter::KERNEL_7x7;}
  throw std::invalid_argument("depth.median_filter: choose off, 3x3, 5x5 or 7x7");
}
}  // namespace

void validate(const Config & c)
{
  resolutionSize(c.resolution);
  preset(c.mode);
  median(c.median);
  if (!std::isfinite(c.fps) || c.fps < 1 || c.fps > 60) {
    throw std::invalid_argument("camera.fps must be 1..60; achievable FPS depends on the device/mode");
  }
  for (const double intensity : {c.dot_intensity, c.flood_intensity}) {
    if (!std::isfinite(intensity) || intensity < 0 || intensity > 1) {
      throw std::invalid_argument("IR dot/flood intensities must be finite values in 0..1");
    }
  }
  if (c.confidence < 0 || c.confidence > 255 || c.lr_threshold < 0 || c.lr_threshold > 128) {
    throw std::invalid_argument("confidence_threshold must be 0..255; left_right_threshold must be 0..128");
  }
  if (c.subpixel_bits < 3 || c.subpixel_bits > 5) {
    throw std::invalid_argument("depth.subpixel_fractional_bits must be 3, 4 or 5");
  }
  if (c.subpixel && c.extended) {
    throw std::invalid_argument("subpixel and extended_disparity cannot be enabled together");
  }
  if (c.subpixel && c.subpixel_bits > 3 && c.median != "off") {
    throw std::invalid_argument("4/5-bit subpixel requires depth.median_filter: off");
  }
  if (c.projection.pixel_stride < 1 || c.projection.pixel_stride > 16 ||
    !std::isfinite(c.projection.min_depth_m) || !std::isfinite(c.projection.max_depth_m) ||
    c.projection.min_depth_m < 0 || c.projection.max_depth_m < 0 ||
    (c.projection.max_depth_m > 0 && c.projection.max_depth_m <= c.projection.min_depth_m))
  {
    throw std::invalid_argument("points: stride must be 1..16; depth limits must be >=0, with max=0 or max>min");
  }
  if (!std::isfinite(c.max_age_sec) || c.max_age_sec < 0.02 || c.max_age_sec > 5 ||
    !std::isfinite(c.metrics_interval) || c.metrics_interval < 0.1 || c.metrics_interval > 60)
  {
    throw std::invalid_argument("input.max_age_sec must be 0.02..5; metrics.print_interval_sec must be 0.1..60");
  }
}

DepthSource::DepthSource(const Config & c, const std::string & device_id)
{
  validate(c);
  device_ = device_id.empty() ? std::make_shared<dai::Device>(dai::UsbSpeed::SUPER) :
    std::make_shared<dai::Device>(dai::DeviceInfo(device_id), dai::UsbSpeed::SUPER);
  pipeline_ = std::make_unique<dai::Pipeline>(device_);
  pipeline_->setAutoCalibrationMode(dai::Pipeline::AutoCalibrationMode::OFF);
  pipeline_->setXLinkChunkSize(0);
  const auto size = resolutionSize(c.resolution);
  auto left = pipeline_->create<dai::node::Camera>();
  auto right = pipeline_->create<dai::node::Camera>();
  left->build(dai::CameraBoardSocket::CAM_B, size, static_cast<float>(c.fps));
  right->build(dai::CameraBoardSocket::CAM_C, size, static_cast<float>(c.fps));
  auto stereo = pipeline_->create<dai::node::StereoDepth>();
  stereo->build(*left->requestOutput(size), *right->requestOutput(size), preset(c.mode));
  stereo->initialConfig->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
  stereo->setDepthAlign(dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT);
  stereo->initialConfig->setConfidenceThreshold(c.confidence);
  stereo->initialConfig->setLeftRightCheckThreshold(c.lr_threshold);
  stereo->setLeftRightCheck(c.lr_check);
  stereo->setSubpixel(c.subpixel);
  stereo->setSubpixelFractionalBits(c.subpixel_bits);
  stereo->setExtendedDisparity(c.extended);
  stereo->initialConfig->setMedianFilter(median(c.median));
  auto & post = stereo->initialConfig->postProcessing;
  post.decimationFilter.decimationFactor = 1;
  post.temporalFilter.enable = false;  // Each cloud is always one depth frame.
  post.spatialFilter.enable = c.spatial;
  post.speckleFilter.enable = c.speckle;
  post.holeFilling.enable = c.hole_filling;
  post.adaptiveMedianFilter.enable = c.adaptive_median;
  post.bilateralSigmaValue = 0;
  post.thresholdFilter.minRange = 0;
  post.thresholdFilter.maxRange = 65535;
  queue_ = stereo->depth.createOutputQueue(1, false);
  pipeline_->build();
  const auto bridge = stereo->depth.getXLinkBridge();
  if (!bridge || !bridge->xLinkOut) {throw std::runtime_error("Missing depth XLink bridge");}
  bridge->xLinkOut->input.setMaxSize(1);
  bridge->xLinkOut->input.setBlocking(false);
  pipeline_->start();
  if (!device_->setIrLaserDotProjectorIntensity(static_cast<float>(c.dot_intensity)) && c.dot_intensity > 0) {
    throw std::runtime_error("Dot projector unavailable: use a supported OAK Pro or set depth.ir_dot_projector_intensity=0.0");
  }
  if (!device_->setIrFloodLightIntensity(static_cast<float>(c.flood_intensity)) && c.flood_intensity > 0) {
    throw std::runtime_error("IR flood light unavailable: set depth.ir_flood_light_intensity=0.0");
  }
}

DepthSource::~DepthSource()
{
  try {if (pipeline_) {pipeline_->stop();}} catch (...) {}  // Disconnect must not terminate ROS shutdown.
}

std::shared_ptr<dai::ImgFrame> DepthSource::tryGet()
{
  if (!pipeline_->isRunning()) {throw std::runtime_error("Depth pipeline stopped");}
  return queue_->tryGet<dai::ImgFrame>();
}

std::string DepthSource::deviceId() const {return device_->getDeviceId();}
}  // namespace point_cloud
