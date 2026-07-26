#ifndef CAMERA_HEIGHT_ESTIMATOR__CAMERA_HEIGHT_GEOMETRY_HPP_
#define CAMERA_HEIGHT_ESTIMATOR__CAMERA_HEIGHT_GEOMETRY_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace camera_height_estimator
{

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

struct DepthSample
{
  double u{0.0};
  double v{0.0};
  double z_m{0.0};
};

struct HeightGeometryConfig
{
  std::size_t minimum_valid_pixels{25U};
  double minimum_depth_m{0.30};
  double maximum_depth_m{3.00};
  double minimum_height_m{0.10};
  double maximum_height_m{1.00};
  double maximum_height_mad_m{0.015};
  double minimum_downward_ray_component{0.05};
};

struct HeightEstimate
{
  double height_m{0.0};
  double median_depth_m{0.0};
  double height_mad_m{0.0};
  double downward_ray_component{0.0};
  std::size_t valid_pixel_count{0U};
};

enum class HeightRejection
{
  NONE,
  INVALID_SPECIFIC_FORCE,
  CENTER_RAY_NOT_DOWNWARD,
  INSUFFICIENT_VALID_PIXELS,
  HEIGHT_OUT_OF_RANGE,
  INCONSISTENT_HEIGHTS
};

struct HeightDiagnostics
{
  HeightRejection rejection{HeightRejection::NONE};
  double candidate_height_m{0.0};
  double median_depth_m{0.0};
  double height_mad_m{0.0};
  double downward_ray_component{0.0};
  std::size_t valid_pixel_count{0U};
};

const char * heightRejectionName(HeightRejection rejection);

std::optional<HeightEstimate> estimateCameraHeight(
  const std::vector<DepthSample> & samples,
  const CameraIntrinsics & intrinsics,
  const std::array<double, 3> & camera_specific_force_mps2,
  const HeightGeometryConfig & config,
  HeightDiagnostics * diagnostics = nullptr);

}  // namespace camera_height_estimator

#endif  // CAMERA_HEIGHT_ESTIMATOR__CAMERA_HEIGHT_GEOMETRY_HPP_
