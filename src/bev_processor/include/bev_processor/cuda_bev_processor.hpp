#ifndef BEV_PROCESSOR__CUDA_BEV_PROCESSOR_HPP_
#define BEV_PROCESSOR__CUDA_BEV_PROCESSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace bev_processor
{

enum class BevInterpolation
{
  Bilinear,
  Bicubic,
  Adaptive,
};

struct EdgeAdaptiveConfig
{
  double start_x_m{1.20};
  double full_x_m{2.00};
  double strength{0.75};
  double gradient_low{8.0};
  double gradient_high{32.0};
  double coherence_minimum{0.30};
  double maximum_anisotropy{3.0};
  double bev_x_max_m{3.0};
  double meter_per_pixel{0.01};
};

struct CudaLanePreprocessConfig
{
  bool enabled{true};
  int gray_mode{0};

  double near_ratio{0.45};
  double middle_ratio{0.35};
  double far_ratio{0.20};

  double near_gain{1.5};
  int near_noise_floor{17};
  int near_kernel_width{7};
  int near_kernel_height{7};

  double middle_gain{1.6};
  int middle_noise_floor{13};
  int middle_kernel_width{17};
  int middle_kernel_height{17};

  double far_gain{1.65};
  int far_noise_floor{11};
  int far_kernel_width{27};
  int far_kernel_height{27};

  // Shape: 0=rectangle, 1=ellipse, 2=cross.
  int top_hat_kernel_shape{1};
  int top_hat_iterations{1};
  // Border: 0=constant, 1=replicate, 2=reflect, 3=reflect101.
  int top_hat_border_type{0};
};

struct CudaBevResult
{
  cv::Mat bgr;
  cv::Mat gray;
  cv::Mat enhanced_top_hat;
};

class CudaBevProcessor
{
public:
  CudaBevProcessor(
    int input_width,
    int input_height,
    const cv::Mat & map_x,
    const cv::Mat & map_y,
    BevInterpolation interpolation,
    const EdgeAdaptiveConfig & edge_adaptive_config,
    const CudaLanePreprocessConfig & lane_preprocess_config);
  ~CudaBevProcessor();

  CudaBevProcessor(const CudaBevProcessor &) = delete;
  CudaBevProcessor & operator=(const CudaBevProcessor &) = delete;

  CudaBevResult process(
    const std::uint8_t * nv12,
    std::size_t data_size,
    std::size_t input_stride,
    const cv::Matx33d & stabilized_to_source_homography,
    int source_crop_top);

  const std::string & deviceName() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bev_processor

#endif  // BEV_PROCESSOR__CUDA_BEV_PROCESSOR_HPP_
