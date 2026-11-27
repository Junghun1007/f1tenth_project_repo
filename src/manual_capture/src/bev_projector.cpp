#include "manual_capture/bev_projector.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace manual_capture
{

void validateGrid(const BevGridConfig & grid)
{
  const bool finite =
    std::isfinite(grid.camera_x_m) && std::isfinite(grid.camera_y_m) &&
    std::isfinite(grid.camera_yaw_deg) && std::isfinite(grid.x_min_m) &&
    std::isfinite(grid.x_max_m) && std::isfinite(grid.y_min_m) &&
    std::isfinite(grid.y_max_m) && std::isfinite(grid.meter_per_pixel);
  if (!finite || grid.x_max_m <= grid.x_min_m || grid.y_max_m <= grid.y_min_m ||
    grid.meter_per_pixel <= 0.0 || grid.output_width <= 0 || grid.output_height <= 0)
  {
    throw std::invalid_argument("invalid BEV grid configuration");
  }
  const int expected_width = static_cast<int>(std::lround(
      (grid.y_max_m - grid.y_min_m) / grid.meter_per_pixel));
  const int expected_height = static_cast<int>(std::lround(
      (grid.x_max_m - grid.x_min_m) / grid.meter_per_pixel));
  if (grid.output_width != expected_width || grid.output_height != expected_height) {
    throw std::invalid_argument(
            "BEV output size does not match metric extents and meter_per_pixel");
  }
}

BevProjector::BevProjector(BevGridConfig grid)
: grid_(grid)
{
  validateGrid(grid_);
}

void BevProjector::configure(const LensCalibration & lens, const MountPose & pose)
{
  if (!std::isfinite(lens.fx) || !std::isfinite(lens.fy) || !std::isfinite(lens.cx) ||
    !std::isfinite(lens.cy) || lens.fx <= 0.0 || lens.fy <= 0.0 || lens.width <= 1 ||
    lens.height <= 1 || !cv::checkRange(cv::Mat(lens.rotation_rgb_from_view)) ||
    !cv::checkRange(cv::Mat(lens.view_center_rgb_m)))
  {
    throw std::invalid_argument("invalid lens calibration");
  }
  lens_ = lens;
  has_lens_ = true;
  pose_ = pose;
  rebuild();
}

void BevProjector::setPose(const MountPose & pose)
{
  pose_ = pose;
  if (has_lens_) {
    rebuild();
  }
}

void BevProjector::reset()
{
  has_lens_ = false;
  lut_ = RemapLut{};
}

bool BevProjector::ready() const noexcept
{
  return !lut_.map_x.empty() && !lut_.map_y.empty();
}

void BevProjector::rebuild()
{
  if (!std::isfinite(pose_.height_m) || pose_.height_m <= 0.0 ||
    !std::isfinite(pose_.roll_deg) || !std::isfinite(pose_.pitch_down_deg))
  {
    throw std::invalid_argument("invalid BEV mount pose");
  }
  // Same math as the vehicle BEV: CAM_A pose measured on the ground plane,
  // composed with the calibrated CAM_A->lens transform.
  const cv::Matx33d rotation_vehicle_from_rgb = mountRotationVehicleFromCamera(
    degToRad(pose_.roll_deg), degToRad(pose_.pitch_down_deg), degToRad(grid_.camera_yaw_deg));
  RectifiedCameraModel camera{
    lens_.fx, lens_.fy, lens_.cx, lens_.cy, lens_.width, lens_.height,
    cv::Vec3d(grid_.camera_x_m, grid_.camera_y_m, pose_.height_m) +
    rotation_vehicle_from_rgb * lens_.view_center_rgb_m,
    rotation_vehicle_from_rgb * lens_.rotation_rgb_from_view};
  const BevConfig bev{grid_.x_min_m, grid_.x_max_m, grid_.y_min_m, grid_.y_max_m,
    grid_.meter_per_pixel, grid_.output_width, grid_.output_height};
  lut_ = generateRemap(camera, bev);
}

void BevProjector::checkInput(const cv::Mat & image) const
{
  if (!ready()) {
    throw std::runtime_error("BEV projector is not configured");
  }
  if (image.empty() || (image.type() != CV_8UC1 && image.type() != CV_8UC3) ||
    image.cols != lens_.width || image.rows != lens_.height)
  {
    throw std::invalid_argument("BEV input image does not match the lens calibration");
  }
}

cv::Mat BevProjector::project(const cv::Mat & image) const
{
  checkInput(image);
  return convertToBev(image, lut_);
}

cv::Mat BevProjector::project(
  const cv::Mat & image, const cv::Matx33d & rgb_camera_to_reference_tilt) const
{
  checkInput(image);
  if (!cv::checkRange(cv::Mat(rgb_camera_to_reference_tilt))) {
    throw std::invalid_argument("invalid IMU tilt");
  }
  // The LUT samples the reference (mount pose) view. A ray d in the current
  // CAM_A frame is R*d in the reference frame; for this lens (CAM_A <- view = Q)
  // the same body rotation is Q^T R Q. Source pixel = K (Q^T R Q)^T K^-1 * LUT pixel.
  // The lens offset from CAM_A is a few cm, so the translation part is ignored.
  const cv::Matx33d & q = lens_.rotation_rgb_from_view;
  const cv::Matx33d view_rotation = q.t() * rgb_camera_to_reference_tilt * q;
  const cv::Matx33d k(lens_.fx, 0.0, lens_.cx, 0.0, lens_.fy, lens_.cy, 0.0, 0.0, 1.0);
  const cv::Matx33d reference_to_source = k * view_rotation.t() * k.inv();

  cv::Mat map_x(lut_.map_x.size(), CV_32FC1, cv::Scalar(-1.0F));
  cv::Mat map_y(lut_.map_y.size(), CV_32FC1, cv::Scalar(-1.0F));
  for (int row = 0; row < map_x.rows; ++row) {
    const auto * valid = lut_.valid_mask.ptr<std::uint8_t>(row);
    const auto * x = lut_.map_x.ptr<float>(row);
    const auto * y = lut_.map_y.ptr<float>(row);
    auto * out_x = map_x.ptr<float>(row);
    auto * out_y = map_y.ptr<float>(row);
    for (int column = 0; column < map_x.cols; ++column) {
      if (valid[column] == 0U) {
        continue;
      }
      const cv::Vec3d source = reference_to_source * cv::Vec3d(x[column], y[column], 1.0);
      if (source[2] <= 1.0e-9) {
        continue;
      }
      out_x[column] = static_cast<float>(source[0] / source[2]);
      out_y[column] = static_cast<float>(source[1] / source[2]);
    }
  }
  cv::Mat bev;
  cv::remap(image, bev, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  return bev;
}

}  // namespace manual_capture
