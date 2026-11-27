#ifndef MANUAL_CAPTURE__BEV_PROJECTOR_HPP_
#define MANUAL_CAPTURE__BEV_PROJECTOR_HPP_

#include "manual_capture/bev_geometry.hpp"
#include "opencv2/core.hpp"

namespace manual_capture
{

// Fixed (refit-measured) CAM_A mount pose plus the shared vehicle BEV grid.
// The vehicle frame origin/axes are the same for every lens, so the color BEV
// and the left-lens BEV are pixel aligned and centered on the vehicle (y=0).
struct BevGridConfig
{
  double camera_x_m{-0.09};  // same as bev_processor camera_x_m
  double camera_y_m{0.0};
  double camera_yaw_deg{0.0};
  double x_min_m{0.0};
  double x_max_m{3.0};
  double y_min_m{-0.6};
  double y_max_m{0.6};
  double meter_per_pixel{0.01};
  int output_width{120};
  int output_height{300};
};

struct MountPose
{
  double height_m{0.20};
  double roll_deg{0.0};
  double pitch_down_deg{14.0};
};

// Output-frame calibration of one lens relative to CAM_A.
struct LensCalibration
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};
  cv::Matx33d rotation_rgb_from_view{cv::Matx33d::eye()};
  cv::Vec3d view_center_rgb_m{0.0, 0.0, 0.0};
};

void validateGrid(const BevGridConfig & grid);

class BevProjector
{
public:
  explicit BevProjector(BevGridConfig grid);

  void configure(const LensCalibration & lens, const MountPose & pose);
  void setPose(const MountPose & pose);
  void reset();
  bool ready() const noexcept;

  // GRAY8 or BGR8 image of exactly the configured lens size.
  cv::Mat project(const cv::Mat & image) const;
  // Same, for a frame whose CAM_A attitude differs from the mount pose by the
  // IMU tilt (camera_to_reference rotation in CAM_A optical coordinates, as
  // camera_driver's stabilizer reports it). The fixed LUT is pulled back
  // through the per-frame rotation homography of this lens.
  cv::Mat project(const cv::Mat & image, const cv::Matx33d & rgb_camera_to_reference_tilt) const;

private:
  void rebuild();
  void checkInput(const cv::Mat & image) const;

  BevGridConfig grid_;
  LensCalibration lens_{};
  MountPose pose_{};
  bool has_lens_{false};
  RemapLut lut_;
};

}  // namespace manual_capture

#endif  // MANUAL_CAPTURE__BEV_PROJECTOR_HPP_
