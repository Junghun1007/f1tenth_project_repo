#pragma once

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
  double min_delta_m{0.05};
  double noise_scale{3.0};
  double release_ratio{0.6};
};

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
  double observation_age_sec{0.0};
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

// Explicit, frozen per-pixel background. Learning always replaces all prior data.
class FloorReference
{
public:
  void clear();
  void begin(const CameraGeometry & camera, const FloorConfig & config);
  bool accumulate(const std::uint16_t * depth_mm, std::size_t row_stride_elements);
  bool compatible(const CameraGeometry & camera) const;
  bool ready() const { return !measuring_ && valid_pixels_ > 0; }
  bool measuring() const { return measuring_; }
  int frames() const { return frames_; }
  int targetFrames() const { return target_frames_; }
  std::size_t validPixels() const { return valid_pixels_; }
  bool isForeground(std::size_t pixel, double depth_m, const FloorConfig & config,
    bool was_foreground = false) const;
  void save(const std::string & path) const;
  bool load(const std::string & path, const CameraGeometry & camera);

private:
  CameraGeometry camera_;
  int frames_{0};
  int target_frames_{0};
  int min_samples_{0};
  bool measuring_{false};
  std::size_t valid_pixels_{0};
  std::vector<double> mean_m_;
  std::vector<double> m2_;
  std::vector<std::uint32_t> counts_;
};

DetectionResult detectForeground(const std::uint16_t * depth_mm,
  std::size_t row_stride_elements, const CameraGeometry & camera,
  const FloorReference & floor, const FloorConfig & floor_config,
  const ProjectionConfig & projection, const ClusterConfig & cluster,
  std::vector<std::uint8_t> * foreground_mask = nullptr);
} // namespace depth_lidar
