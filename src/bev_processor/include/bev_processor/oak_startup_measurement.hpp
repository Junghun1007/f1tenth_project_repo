#ifndef BEV_PROCESSOR__OAK_STARTUP_MEASUREMENT_HPP_
#define BEV_PROCESSOR__OAK_STARTUP_MEASUREMENT_HPP_

#include <cstddef>

namespace bev_processor
{

struct OakStartupMeasurementConfig
{
  double stereo_fps{30.0};
  int stereo_width{640};
  int stereo_height{400};
  int depth_queue_size{2};
  double imu_rate_hz{100.0};
  int imu_queue_size{50};
  double warmup_sec{1.0};

  int roi_width{80};
  int roi_height{40};
  int minimum_valid_pixels{800};
  double minimum_depth_m{0.30};
  double maximum_depth_m{3.00};
  double minimum_height_m{0.10};
  double maximum_height_m{1.00};
  double maximum_height_mad_m{0.008};
  double minimum_downward_ray_component{0.05};

  int imu_sample_count{200};
  double imu_max_direction_rms_deg{0.25};
  double imu_accel_min_mps2{7.50};
  double imu_accel_max_mps2{12.00};

  int stable_depth_frame_count{30};
  double maximum_height_stddev_m{0.004};
  double timeout_sec{20.0};
};

struct OakStartupMeasurement
{
  double height_m{0.0};
  double roll_deg{0.0};
  double pitch_down_deg{0.0};
  double imu_direction_rms_deg{0.0};
  double height_stddev_m{0.0};
  double median_depth_m{0.0};
  double height_mad_m{0.0};
  std::size_t valid_pixel_count{0U};
};

OakStartupMeasurement measureOakStartupExtrinsics(
  const OakStartupMeasurementConfig & config);

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__OAK_STARTUP_MEASUREMENT_HPP_
