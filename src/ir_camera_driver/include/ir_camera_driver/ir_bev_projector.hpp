#ifndef IR_CAMERA_DRIVER__IR_BEV_PROJECTOR_HPP_
#define IR_CAMERA_DRIVER__IR_BEV_PROJECTOR_HPP_

#include "opencv2/core.hpp"

namespace ir_camera_driver
{

struct IrBevConfig
{
  double camera_x_m{-0.16};
  double camera_y_m{0.0};
  double camera_height_m{0.20};
  double camera_roll_deg{0.0};
  double camera_pitch_down_deg{14.0};
  double camera_yaw_deg{0.0};
  double x_min_m{0.0};
  double x_max_m{3.0};
  double y_min_m{-0.6};
  double y_max_m{0.6};
  double meter_per_pixel{0.01};
  int output_width{120};
  int output_height{300};
};

class IrBevProjector
{
public:
  explicit IrBevProjector(IrBevConfig config);

  void configure(
    double fx, double fy, double cx, double cy,
    int input_width, int input_height);

  cv::Mat project(const cv::Mat & rectified_image) const;
  bool ready() const noexcept;
  double validPixelRatio() const noexcept;

private:
  IrBevConfig config_;
  cv::Mat map_x_;
  cv::Mat map_y_;
  cv::Mat valid_mask_;
};

}  // namespace ir_camera_driver

#endif  // IR_CAMERA_DRIVER__IR_BEV_PROJECTOR_HPP_
