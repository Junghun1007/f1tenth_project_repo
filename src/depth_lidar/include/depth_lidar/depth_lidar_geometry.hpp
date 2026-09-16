#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace depth_lidar
{
struct RoiRect { int x{0}; int y{0}; int width{0}; int height{0}; };

struct CameraGeometry
{
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::string signature;
};

struct ProjectionConfig
{
  double roi_width_ratio{1.0};
  double roi_height_ratio{0.30};
  double roi_bottom_offset_ratio{0.25};
  double min_range_m{0.10};
  double max_range_m{3.0};
  double range_offset_m{0.0};
  int pixel_stride{1};
};

struct ClusterConfig
{
  int min_points{20};
  double neighbor_distance_m{0.08};
  double radius_margin_m{0.04};
  double min_radius_m{0.05};
};

struct FloorConfig
{
  int measure_frames{60};
  double min_valid_ratio{0.5};
  double measure_roi_width_ratio{0.8};
  double measure_roi_height_ratio{0.35};
  double measure_roi_bottom_offset_ratio{0.05};
  int fit_pixel_stride{4};
  double fit_max_depth_m{4.0};
  int ransac_iterations{200};
  double inlier_distance_m{0.02};
  int min_inlier_points{100};
  double min_inlier_ratio{0.60};
  double max_tilt_deg{60.0};
  double min_camera_height_m{0.05};
  double max_camera_height_m{1.0};
  double min_height_m{0.05};
  double max_height_m{1.0};
  double noise_scale{3.0};
};

// Ground-aligned forward/left and height above the calibrated floor.
struct ForegroundPoint
{
  double forward_m{0.0};
  double left_m{0.0};
  double up_m{0.0};
  int u{0};
  int v{0};
};

struct ObstacleCircle
{
  double forward_m{0.0};
  double left_m{0.0};
  double radius_m{0.0};
  std::size_t support_points{0};
};

struct DetectionResult
{
  RoiRect roi;
  std::vector<ForegroundPoint> points;
  std::vector<ObstacleCircle> obstacles;
};

bool validateProjectionConfig(const ProjectionConfig & config, std::string & reason);
bool validateClusterConfig(const ClusterConfig & config, std::string & reason);
bool validateFloorConfig(const FloorConfig & config, std::string & reason);
RoiRect computeRoi(int image_width, int image_height,
  double width_ratio, double height_ratio, double bottom_offset_ratio);

// Explicit, frozen plane in camera forward/left/up coordinates. Unit normal
// points upward, d > 0 is camera height. A new measurement replaces prior data.
class FloorReference
{
public:
  void clear();
  void begin(const CameraGeometry & camera, const FloorConfig & config);
  bool accumulate(const std::uint16_t * depth_mm, std::size_t row_stride_elements);
  bool compatible(const CameraGeometry & camera) const;
  bool ready() const { return !measuring_ && inlier_points_ > 0; }
  bool measuring() const { return measuring_; }
  int frames() const { return frames_; }
  int targetFrames() const { return target_frames_; }
  std::size_t inlierPoints() const { return inlier_points_; }
  double cameraHeight() const { return plane_[3]; }
  double rmse() const { return rmse_m_; }
  const std::string & failureReason() const { return failure_reason_; }
  std::array<double, 3> groundCoordinates(double forward, double left, double up) const;
  bool isObstacleHeight(double height_m, const FloorConfig & config) const;
  void save(const std::string & path) const;
  bool load(const std::string & path, const CameraGeometry & camera);

private:
  CameraGeometry camera_;
  int frames_{0};
  int target_frames_{0};
  int min_samples_{0};
  bool measuring_{false};
  FloorConfig measurement_config_;
  std::array<double, 4> plane_{{0.0, 0.0, 1.0, 0.0}};
  std::array<double, 3> forward_axis_{{1.0, 0.0, 0.0}};
  std::array<double, 3> left_axis_{{0.0, 1.0, 0.0}};
  double rmse_m_{0.0};
  std::size_t sample_points_{0};
  std::size_t inlier_points_{0};
  std::string failure_reason_;
  bool fitPlane();
  void updateGroundAxes();
  std::vector<double> mean_m_;
  std::vector<std::uint32_t> counts_;
};

DetectionResult detectForeground(const std::uint16_t * depth_mm,
  std::size_t row_stride_elements, const CameraGeometry & camera,
  const FloorReference & floor, const FloorConfig & floor_config,
  const ProjectionConfig & projection, const ClusterConfig & cluster);
} // namespace depth_lidar
