#include "camera_height_estimator/camera_height_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace camera_height_estimator
{

namespace
{

double median(std::vector<double> values)
{
  if (values.empty()) {
    throw std::invalid_argument("median requires at least one value");
  }

  const auto middle = values.begin() +
    static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  if ((values.size() % 2U) != 0U) {
    return *middle;
  }

  const double upper = *middle;
  const auto lower = std::max_element(values.begin(), middle);
  return 0.5 * (*lower + upper);
}

void setRejection(
  HeightDiagnostics * diagnostics,
  const HeightRejection rejection)
{
  if (diagnostics != nullptr) {
    diagnostics->rejection = rejection;
  }
}

}  // namespace

const char * heightRejectionName(const HeightRejection rejection)
{
  switch (rejection) {
    case HeightRejection::NONE:
      return "none";
    case HeightRejection::INVALID_SPECIFIC_FORCE:
      return "invalid IMU gravity";
    case HeightRejection::CENTER_RAY_NOT_DOWNWARD:
      return "camera center ray is not sufficiently downward";
    case HeightRejection::INSUFFICIENT_VALID_PIXELS:
      return "insufficient valid center depth pixels";
    case HeightRejection::HEIGHT_OUT_OF_RANGE:
      return "candidate height is outside the allowed range";
    case HeightRejection::INCONSISTENT_HEIGHTS:
      return "center-pixel heights are spatially inconsistent";
    default:
      return "unknown";
  }
}

std::optional<HeightEstimate> estimateCameraHeight(
  const std::vector<DepthSample> & samples,
  const CameraIntrinsics & intrinsics,
  const std::array<double, 3> & camera_specific_force_mps2,
  const HeightGeometryConfig & config,
  HeightDiagnostics * diagnostics)
{
  if (diagnostics != nullptr) {
    *diagnostics = HeightDiagnostics{};
  }
  if (
    !std::isfinite(intrinsics.fx) ||
    !std::isfinite(intrinsics.fy) ||
    !std::isfinite(intrinsics.cx) ||
    !std::isfinite(intrinsics.cy) ||
    intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0)
  {
    throw std::invalid_argument("camera intrinsics must be finite and positive");
  }
  if (
    config.minimum_valid_pixels == 0U ||
    config.minimum_depth_m <= 0.0 ||
    config.maximum_depth_m <= config.minimum_depth_m ||
    config.minimum_height_m <= 0.0 ||
    config.maximum_height_m <= config.minimum_height_m ||
    config.maximum_height_mad_m <= 0.0 ||
    config.minimum_downward_ray_component <= 0.0 ||
    config.minimum_downward_ray_component >= 1.0)
  {
    throw std::invalid_argument("invalid height geometry configuration");
  }

  const double acceleration_norm = std::sqrt(
    camera_specific_force_mps2[0] * camera_specific_force_mps2[0] +
    camera_specific_force_mps2[1] * camera_specific_force_mps2[1] +
    camera_specific_force_mps2[2] * camera_specific_force_mps2[2]);
  if (!std::isfinite(acceleration_norm) || acceleration_norm <= 1.0e-9) {
    setRejection(diagnostics, HeightRejection::INVALID_SPECIFIC_FORCE);
    return std::nullopt;
  }

  // A stationary accelerometer measures specific force opposite gravity.
  // It therefore points upward. The aligned camera optical frame is RDF:
  // +X right, +Y down, +Z forward.
  const std::array<double, 3> up_camera{
    camera_specific_force_mps2[0] / acceleration_norm,
    camera_specific_force_mps2[1] / acceleration_norm,
    camera_specific_force_mps2[2] / acceleration_norm};
  const double downward_ray_component = -up_camera[2];
  if (diagnostics != nullptr) {
    diagnostics->downward_ray_component = downward_ray_component;
  }
  if (
    !std::isfinite(downward_ray_component) ||
    downward_ray_component < config.minimum_downward_ray_component)
  {
    setRejection(diagnostics, HeightRejection::CENTER_RAY_NOT_DOWNWARD);
    return std::nullopt;
  }

  std::vector<double> heights;
  std::vector<double> depths;
  heights.reserve(samples.size());
  depths.reserve(samples.size());

  for (const auto & sample : samples) {
    if (
      !std::isfinite(sample.u) ||
      !std::isfinite(sample.v) ||
      !std::isfinite(sample.z_m) ||
      sample.z_m < config.minimum_depth_m ||
      sample.z_m > config.maximum_depth_m)
    {
      continue;
    }

    // DepthAI RAW16 stereo depth is camera-Z depth, not Euclidean distance.
    const double x_m =
      (sample.u - intrinsics.cx) * sample.z_m / intrinsics.fx;
    const double y_m =
      (sample.v - intrinsics.cy) * sample.z_m / intrinsics.fy;
    const double height_m = -(
      up_camera[0] * x_m +
      up_camera[1] * y_m +
      up_camera[2] * sample.z_m);
    if (std::isfinite(height_m) && height_m > 0.0) {
      heights.push_back(height_m);
      depths.push_back(sample.z_m);
    }
  }

  if (heights.size() < config.minimum_valid_pixels) {
    if (diagnostics != nullptr) {
      diagnostics->valid_pixel_count = heights.size();
    }
    setRejection(diagnostics, HeightRejection::INSUFFICIENT_VALID_PIXELS);
    return std::nullopt;
  }

  const double height_m = median(heights);
  const double median_depth_m = median(depths);
  std::vector<double> absolute_height_errors;
  absolute_height_errors.reserve(heights.size());
  for (const double value : heights) {
    absolute_height_errors.push_back(std::abs(value - height_m));
  }
  const double height_mad_m = median(std::move(absolute_height_errors));

  if (diagnostics != nullptr) {
    diagnostics->candidate_height_m = height_m;
    diagnostics->median_depth_m = median_depth_m;
    diagnostics->height_mad_m = height_mad_m;
    diagnostics->valid_pixel_count = heights.size();
  }
  if (
    height_m < config.minimum_height_m ||
    height_m > config.maximum_height_m)
  {
    setRejection(diagnostics, HeightRejection::HEIGHT_OUT_OF_RANGE);
    return std::nullopt;
  }
  if (height_mad_m > config.maximum_height_mad_m) {
    setRejection(diagnostics, HeightRejection::INCONSISTENT_HEIGHTS);
    return std::nullopt;
  }

  return HeightEstimate{
    height_m,
    median_depth_m,
    height_mad_m,
    downward_ray_component,
    heights.size()};
}

}  // namespace camera_height_estimator
