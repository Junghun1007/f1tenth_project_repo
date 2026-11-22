#include "ir_camera_driver/ir_bev_projector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <stdexcept>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"

namespace ir_camera_driver
{
namespace
{

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
  const int input_width, const int input_height,
  const cv::Matx33d & rotation_rgb_from_view,
  const cv::Vec3d & view_center_rgb_m,
  const std::vector<double> & distortion)
{
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) ||
    !std::isfinite(cy) || fx <= 0.0 || fy <= 0.0 || input_width <= 1 ||
    input_height <= 1 || !cv::checkRange(cv::Mat(rotation_rgb_from_view)) ||
    !cv::checkRange(cv::Mat(view_center_rgb_m)))
  {
    throw std::invalid_argument("invalid rectified IR camera calibration");
  }
  camera_ = {fx, fy, cx, cy, input_width, input_height,
    cv::Vec3d(0.0, 0.0, 0.0), cv::Matx33d::eye()};
  grid_ = {config_.x_min_m, config_.x_max_m, config_.y_min_m, config_.y_max_m,
    config_.meter_per_pixel, config_.output_width, config_.output_height};
  rotation_vehicle_from_rgb_ = bev_processor::mountRotationVehicleFromCamera(
    bev_processor::degToRad(config_.camera_roll_deg),
    bev_processor::degToRad(config_.camera_pitch_down_deg),
    bev_processor::degToRad(config_.camera_yaw_deg));
  rotation_rgb_from_view_ = rotation_rgb_from_view;
  view_center_rgb_m_ = view_center_rgb_m;
  distortion_.clear();
  max_normalized_radius_ = 0.0;
  bool any_distortion = false;
  for (const double c : distortion) {any_distortion = any_distortion || c != 0.0;}
  if (any_distortion) {
    const auto n = distortion.size();
    if ((n != 4 && n != 5 && n != 8 && n != 12 && n != 14) ||
      !cv::checkRange(cv::Mat(distortion)))
    {
      throw std::invalid_argument("IR distortion needs 4/5/8/12/14 finite OpenCV coefficients");
    }
    distortion_ = distortion;
    // 원본 영상 가장자리 점들을 왜곡 해제해 시선 반경의 상한을 구한다
    std::vector<cv::Point2d> edge, undistorted;
    const double w = input_width - 1.0, h = input_height - 1.0;
    for (int i = 0; i <= 16; ++i) {
      const double t = i / 16.0;
      edge.emplace_back(t * w, 0.0); edge.emplace_back(t * w, h);
      edge.emplace_back(0.0, t * h); edge.emplace_back(w, t * h);
    }
    const cv::Matx33d k(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    cv::undistortPoints(edge, undistorted, k, distortion_);
    for (const auto & p : undistorted) {
      max_normalized_radius_ = std::max(max_normalized_radius_, std::hypot(p.x, p.y));
    }
    max_normalized_radius_ *= 1.05;
  }
  updateMaps(cv::Matx33d::eye());
}

void IrBevProjector::updateMaps(const cv::Matx33d & current_rgb_to_reference)
{
  // Same vehicle axes, metric pixel centers and inverse projection as
  // bev_processor. The startup measurement is in CAM_A optical coordinates.
  const cv::Matx33d rotation_vehicle_from_current_rgb =
    rotation_vehicle_from_rgb_ * current_rgb_to_reference;
  camera_.rotation_vehicle_from_camera =
    rotation_vehicle_from_current_rgb * rotation_rgb_from_view_;
  camera_.position_vehicle_m =
    cv::Vec3d(config_.camera_x_m, config_.camera_y_m, config_.camera_height_m) +
    rotation_vehicle_from_current_rgb * view_center_rgb_m_;
  // Rotate the calibrated RGB-to-IR lever arm as well as the optical axes.
  // Translation/heave of the whole device is not observable from tilt alone.
  lut_ = distortion_.empty() ? bev_processor::generateRemap(camera_, grid_) : distortedRemap();
  last_correction_ = current_rgb_to_reference;
}

bev_processor::RemapLut IrBevProjector::distortedRemap() const
{
  // bev_processor::generateRemap과 같은 격자·좌표계. 바닥점 → 카메라 시선 → 렌즈 왜곡 → 원본(왜곡) 영상 픽셀
  bev_processor::RemapLut lut{
    cv::Mat(grid_.output_height, grid_.output_width, CV_32FC1, cv::Scalar(-1.0F)),
    cv::Mat(grid_.output_height, grid_.output_width, CV_32FC1, cv::Scalar(-1.0F)),
    cv::Mat(grid_.output_height, grid_.output_width, CV_8UC1, cv::Scalar(0))};
  const cv::Matx33d rotation_camera_from_vehicle = camera_.rotation_vehicle_from_camera.t();
  std::vector<cv::Point3d> rays;
  std::vector<int> index;
  rays.reserve(static_cast<std::size_t>(grid_.output_width * grid_.output_height));
  index.reserve(rays.capacity());
  for (int v_bev = 0; v_bev < grid_.output_height; ++v_bev) {
    const double x_vehicle = grid_.x_max_m - (v_bev + 0.5) * grid_.meter_per_pixel;
    for (int u_bev = 0; u_bev < grid_.output_width; ++u_bev) {
      const double y_vehicle = grid_.y_max_m - (u_bev + 0.5) * grid_.meter_per_pixel;
      const cv::Vec3d p = rotation_camera_from_vehicle *
        (cv::Vec3d(x_vehicle, y_vehicle, 0.0) - camera_.position_vehicle_m);
      if (p[2] <= 1.0e-6) {continue;}
      const double x = p[0] / p[2], y = p[1] / p[2];
      // 유효 영역은 예전(장치 왜곡 보정 영상, 같은 K)과 똑같이: 보정 영상 안쪽 시선만
      const double u_rect = camera_.fx * x + camera_.cx, v_rect = camera_.fy * y + camera_.cy;
      if (u_rect < 0.0 || v_rect < 0.0 || u_rect >= camera_.image_width - 1.0 ||
        v_rect >= camera_.image_height - 1.0 || std::hypot(x, y) > max_normalized_radius_) {continue;}
      rays.emplace_back(x, y, 1.0);
      index.push_back(v_bev * grid_.output_width + u_bev);
    }
  }
  if (rays.empty()) {return lut;}
  std::vector<cv::Point2d> pixels(rays.size());
  double c[14] = {0};
  for (std::size_t i = 0; i < distortion_.size(); ++i) {c[i] = distortion_[i];}
  if (c[12] != 0.0 || c[13] != 0.0) {
    // 센서 기울기(τx, τy) 모델은 OpenCV에 맡긴다
    const cv::Matx33d k(camera_.fx, 0.0, camera_.cx, 0.0, camera_.fy, camera_.cy, 0.0, 0.0, 1.0);
    cv::projectPoints(rays, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), k, distortion_, pixels);
  } else {
    // OpenCV 원근 모델 (k1,k2,p1,p2,k3,k4,k5,k6,s1..s4) — projectPoints와 같은 식, 매 프레임이라 직접 계산
    for (std::size_t i = 0; i < rays.size(); ++i) {
      const double x = rays[i].x, y = rays[i].y, r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
      const double radial = (1.0 + c[0] * r2 + c[1] * r4 + c[4] * r6) / (1.0 + c[5] * r2 + c[6] * r4 + c[7] * r6);
      const double xd = x * radial + 2.0 * c[2] * x * y + c[3] * (r2 + 2.0 * x * x) + c[8] * r2 + c[9] * r4;
      const double yd = y * radial + c[2] * (r2 + 2.0 * y * y) + 2.0 * c[3] * x * y + c[10] * r2 + c[11] * r4;
      pixels[i] = {camera_.fx * xd + camera_.cx, camera_.fy * yd + camera_.cy};
    }
  }
  auto * map_x = lut.map_x.ptr<float>();
  auto * map_y = lut.map_y.ptr<float>();
  auto * valid = lut.valid_mask.ptr<std::uint8_t>();
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    const double u = pixels[i].x, v = pixels[i].y;
    // generateRemap과 같이 오른쪽·아래 가장자리에 보간용 한 픽셀을 남긴다
    if (!(u >= 0.0 && v >= 0.0 && u < camera_.image_width - 1.0 && v < camera_.image_height - 1.0)) {continue;}
    map_x[index[i]] = static_cast<float>(u);
    map_y[index[i]] = static_cast<float>(v);
    valid[index[i]] = 255U;
  }
  return lut;
}

cv::Mat IrBevProjector::project(
  const cv::Mat & rectified_image,
  const cv::Matx33d & current_rgb_to_reference)
{
  if (!ready()) {
    throw std::runtime_error("IR BEV projector has not received camera intrinsics");
  }
  if (rectified_image.empty() || rectified_image.type() != CV_8UC1 ||
    rectified_image.cols != camera_.image_width ||
    rectified_image.rows != camera_.image_height ||
    !cv::checkRange(cv::Mat(current_rgb_to_reference)))
  {
    throw std::invalid_argument("invalid IR BEV image or IMU correction");
  }
  if (cv::norm(cv::Mat(current_rgb_to_reference - last_correction_)) > 1.0e-10) {
    updateMaps(current_rgb_to_reference);
  }
  return bev_processor::convertToBev(rectified_image, lut_);
}

bool IrBevProjector::ready() const noexcept
{
  return !lut_.map_x.empty() && !lut_.map_y.empty();
}

void IrBevProjector::setMountPose(
  const cv::Matx33d & rotation_vehicle_from_rgb, const cv::Vec3d & position)
{
  if (!ready() || !cv::checkRange(cv::Mat(rotation_vehicle_from_rgb)) ||
    !cv::checkRange(cv::Mat(position)) || position[2]<=0.0) {
    throw std::invalid_argument("invalid shared BEV mounting pose");
  }
  rotation_vehicle_from_rgb_=rotation_vehicle_from_rgb;
  config_.camera_x_m=position[0]; config_.camera_y_m=position[1];
  config_.camera_height_m=position[2];
  updateMaps(cv::Matx33d::eye());
}

double IrBevProjector::validPixelRatio() const noexcept
{
  if (lut_.valid_mask.empty()) {
    return 0.0;
  }
  return static_cast<double>(cv::countNonZero(lut_.valid_mask)) /
         static_cast<double>(lut_.valid_mask.total());
}

const cv::Mat & IrBevProjector::validMask() const noexcept
{
  return lut_.valid_mask;
}

}  // namespace ir_camera_driver
