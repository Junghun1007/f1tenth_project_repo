#include "ir_camera_driver/ir_bev_projector.hpp"

#include <cmath>
#include <stdexcept>

#include "opencv2/imgproc.hpp"

namespace ir_camera_driver
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double radians(const double degrees)
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

cv::Matx33d rotationVehicleFromCamera(const IrBevConfig & config)
{
  // Camera optical coordinates are right/down/forward. Vehicle coordinates
  // are forward/left/up. Positive configured pitch points the camera down.
  const cv::Matx33d optical_to_vehicle(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
  return rotationZ(radians(config.camera_yaw_deg)) *
         rotationY(radians(config.camera_pitch_down_deg)) *
         rotationX(radians(config.camera_roll_deg)) *
         optical_to_vehicle;
}

void validateConfig(const IrBevConfig & config)
{
  const bool finite =
    std::isfinite(config.camera_x_m) &&
    std::isfinite(config.camera_y_m) &&
    std::isfinite(config.camera_height_m) &&
    std::isfinite(config.camera_roll_deg) &&
    std::isfinite(config.camera_pitch_down_deg) &&
    std::isfinite(config.camera_yaw_deg) &&
    std::isfinite(config.x_min_m) &&
    std::isfinite(config.x_max_m) &&
    std::isfinite(config.y_min_m) &&
    std::isfinite(config.y_max_m) &&
    std::isfinite(config.meter_per_pixel);
  if (!finite || config.camera_height_m <= 0.0 ||
    config.x_max_m <= config.x_min_m || config.y_max_m <= config.y_min_m ||
    config.meter_per_pixel <= 0.0 || config.output_width <= 0 ||
    config.output_height <= 0)
  {
    throw std::invalid_argument("invalid IR BEV geometry configuration");
  }

  const int expected_width = static_cast<int>(std::lround(
      (config.y_max_m - config.y_min_m) / config.meter_per_pixel));
  const int expected_height = static_cast<int>(std::lround(
      (config.x_max_m - config.x_min_m) / config.meter_per_pixel));
  if (config.output_width != expected_width || config.output_height != expected_height) {
    throw std::invalid_argument(
            "IR BEV output dimensions do not match metric extents and meter_per_pixel");
  }
}

}  // namespace

IrBevProjector::IrBevProjector(IrBevConfig config)
: config_(config)
{
  validateConfig(config_);
}

void IrBevProjector::configure(
  const double fx, const double fy, const double cx, const double cy,
  const int input_width, const int input_height)
{
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) ||
    !std::isfinite(cy) || fx <= 0.0 || fy <= 0.0 || input_width <= 1 ||
    input_height <= 1)
  {
    throw std::invalid_argument("invalid rectified IR camera intrinsics");
  }

  map_x_ = cv::Mat(
    config_.output_height, config_.output_width, CV_32FC1, cv::Scalar(-1.0F));
  map_y_ = cv::Mat(
    config_.output_height, config_.output_width, CV_32FC1, cv::Scalar(-1.0F));
  valid_mask_ = cv::Mat(
    config_.output_height, config_.output_width, CV_8UC1, cv::Scalar(0));

  const cv::Matx33d rotation_camera_from_vehicle =
    rotationVehicleFromCamera(config_).t();
  const cv::Vec3d camera_position(
    config_.camera_x_m, config_.camera_y_m, config_.camera_height_m);

  for (int v_bev = 0; v_bev < config_.output_height; ++v_bev) {
    const double x_vehicle = config_.x_max_m -
      (static_cast<double>(v_bev) + 0.5) * config_.meter_per_pixel;
    for (int u_bev = 0; u_bev < config_.output_width; ++u_bev) {
      const double y_vehicle = config_.y_max_m -
        (static_cast<double>(u_bev) + 0.5) * config_.meter_per_pixel;
      const cv::Vec3d point_camera = rotation_camera_from_vehicle *
        (cv::Vec3d(x_vehicle, y_vehicle, 0.0) - camera_position);
      if (point_camera[2] <= 1.0e-6) {
        continue;
      }

      const double u_source = fx * point_camera[0] / point_camera[2] + cx;
      const double v_source = fy * point_camera[1] / point_camera[2] + cy;
      if (u_source < 0.0 || v_source < 0.0 ||
        u_source >= static_cast<double>(input_width - 1) ||
        v_source >= static_cast<double>(input_height - 1))
      {
        continue;
      }
      map_x_.at<float>(v_bev, u_bev) = static_cast<float>(u_source);
      map_y_.at<float>(v_bev, u_bev) = static_cast<float>(v_source);
      valid_mask_.at<std::uint8_t>(v_bev, u_bev) = 255U;
    }
  }
}

cv::Mat IrBevProjector::project(const cv::Mat & rectified_image) const
{
  if (!ready()) {
    throw std::runtime_error("IR BEV projector has not received camera intrinsics");
  }
  if (rectified_image.empty() || rectified_image.type() != CV_8UC1) {
    throw std::invalid_argument("IR BEV input must be a non-empty GRAY8 image");
  }
  cv::Mat output;
  cv::remap(
    rectified_image, output, map_x_, map_y_, cv::INTER_LINEAR,
    cv::BORDER_CONSTANT, cv::Scalar(0));
  return output;
}

bool IrBevProjector::ready() const noexcept
{
  return !map_x_.empty() && !map_y_.empty();
}

double IrBevProjector::validPixelRatio() const noexcept
{
  if (valid_mask_.empty()) {
    return 0.0;
  }
  return static_cast<double>(cv::countNonZero(valid_mask_)) /
         static_cast<double>(valid_mask_.total());
}

}  // namespace ir_camera_driver
