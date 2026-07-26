#include "bev_processor/bev_geometry.hpp"

#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace bev_processor
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

double degToRad(const double degrees)
{
  return degrees * kPi / 180.0;
}

cv::Matx33d rotationX(const double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return cv::Matx33d(
    1.0, 0.0, 0.0,
    0.0, cosine, -sine,
    0.0, sine, cosine);
}

cv::Matx33d rotationY(const double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return cv::Matx33d(
    cosine, 0.0, sine,
    0.0, 1.0, 0.0,
    -sine, 0.0, cosine);
}

cv::Matx33d rotationZ(const double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return cv::Matx33d(
    cosine, -sine, 0.0,
    sine, cosine, 0.0,
    0.0, 0.0, 1.0);
}

cv::Matx33d mountRotationVehicleFromCamera(
  const double roll_rad,
  const double downward_pitch_rad,
  const double yaw_rad)
{
  // At zero angles: camera +X=vehicle -Y, camera +Y=vehicle -Z,
  // camera +Z=vehicle +X. Positive pitch points the camera downward.
  const cv::Matx33d optical_to_vehicle(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
  return rotationZ(yaw_rad) *
         rotationY(downward_pitch_rad) *
         rotationX(roll_rad) *
         optical_to_vehicle;
}

EulerAngles cameraAttitudeFromSpecificForce(
  const cv::Vec3d & acceleration_camera_mps2)
{
  const double magnitude = cv::norm(acceleration_camera_mps2);
  if (!std::isfinite(magnitude) || magnitude <= 1.0e-9) {
    throw std::invalid_argument(
            "camera-frame acceleration must be finite and non-zero");
  }

  // Camera optical coordinates are RDF: +X right, +Y down, +Z forward.
  // At rest an accelerometer measures specific force opposite gravity, so a
  // level camera reads approximately (0, -g, 0). These equations match the
  // roll and positive-downward-pitch convention used by the BEV mount model.
  return EulerAngles{
    std::atan2(
      -acceleration_camera_mps2[0],
      -acceleration_camera_mps2[1]),
    std::atan2(
      -acceleration_camera_mps2[2],
      std::hypot(
        acceleration_camera_mps2[0],
        acceleration_camera_mps2[1])),
    0.0};
}

RemapLut generateRemap(
  const RectifiedCameraModel & camera,
  const BevConfig & bev)
{
  RemapLut lut{
    cv::Mat(
      bev.output_height, bev.output_width,
      CV_32FC1, cv::Scalar(-1.0F)),
    cv::Mat(
      bev.output_height, bev.output_width,
      CV_32FC1, cv::Scalar(-1.0F)),
    cv::Mat(
      bev.output_height, bev.output_width,
      CV_8UC1, cv::Scalar(0))};

  // P_c = R_cv * (P_v - C_v), where R_cv = R_vc^T.
  const cv::Matx33d rotation_camera_from_vehicle =
    camera.rotation_vehicle_from_camera.t();

  for (int v_bev = 0; v_bev < bev.output_height; ++v_bev) {
    const double x_vehicle =
      bev.x_max_m -
      (static_cast<double>(v_bev) + 0.5) * bev.meter_per_pixel;

    for (int u_bev = 0; u_bev < bev.output_width; ++u_bev) {
      const double y_vehicle =
        bev.y_max_m -
        (static_cast<double>(u_bev) + 0.5) * bev.meter_per_pixel;
      const cv::Vec3d point_vehicle(x_vehicle, y_vehicle, 0.0);
      const cv::Vec3d point_camera =
        rotation_camera_from_vehicle *
        (point_vehicle - camera.position_vehicle_m);

      if (point_camera[2] <= 1.0e-6) {
        continue;
      }

      const double u_source =
        camera.fx * point_camera[0] / point_camera[2] + camera.cx;
      const double v_source =
        camera.fy * point_camera[1] / point_camera[2] + camera.cy;

      // Keep one interpolation pixel inside the right and bottom edges.
      if (
        u_source < 0.0 || v_source < 0.0 ||
        u_source >= static_cast<double>(camera.image_width - 1) ||
        v_source >= static_cast<double>(camera.image_height - 1))
      {
        continue;
      }

      lut.map_x.at<float>(v_bev, u_bev) =
        static_cast<float>(u_source);
      lut.map_y.at<float>(v_bev, u_bev) =
        static_cast<float>(v_source);
      lut.valid_mask.at<std::uint8_t>(v_bev, u_bev) = 255U;
    }
  }
  return lut;
}

cv::Mat convertToBev(
  const cv::Mat & rectified_image,
  const RemapLut & lut)
{
  cv::Mat bev_image;
  cv::remap(
    rectified_image,
    bev_image,
    lut.map_x,
    lut.map_y,
    cv::INTER_LINEAR,
    cv::BORDER_CONSTANT,
    cv::Scalar(0, 0, 0));
  return bev_image;
}

}  // namespace bev_processor
