// Copied from src/multiclass_yolo/src/cuda_yolox_preprocessor.hpp (origin/1113ML)
#ifndef CONE_DRIVE_TEST_COPIED__CUDA_YOLOX_PREPROCESSOR_HPP_
#define CONE_DRIVE_TEST_COPIED__CUDA_YOLOX_PREPROCESSOR_HPP_

#include <cstddef>
#include <cstdint>

#include <cuda_runtime_api.h>

namespace cone_drive_test
{

cudaError_t launch_nv12_roi_to_bgr_nchw(
  const std::uint8_t * device_nv12,
  std::size_t source_stride,
  int source_height,
  int roi_left,
  int roi_top,
  int roi_width,
  int roi_height,
  float * device_output,
  int output_width,
  int output_height,
  cudaStream_t stream) noexcept;

cudaError_t launch_nv12_roi_to_rgb_nchw(
  const std::uint8_t * device_nv12, std::size_t source_stride, int source_height,
  int roi_left, int roi_top, int roi_width, int roi_height,
  float * device_output, int output_width, int output_height,
  int content_left, int content_top, int content_width, int content_height, cudaStream_t stream) noexcept;

}  // namespace cone_drive_test

#endif  // CONE_DRIVE_TEST_COPIED__CUDA_YOLOX_PREPROCESSOR_HPP_
