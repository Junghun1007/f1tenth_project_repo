// Copied from src/multiclass_yolo/src/cuda_yolox_preprocessor.hpp (origin/1113ML)
#ifndef CONE_DRIVE_TEST_COPIED__CUDA_YOLOX_PREPROCESSOR_HPP_
#define CONE_DRIVE_TEST_COPIED__CUDA_YOLOX_PREPROCESSOR_HPP_

#include <cstddef>
#include <cstdint>

#include <cuda_runtime_api.h>

namespace auto_control::drive {

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

}  // namespace auto_control::drive

#endif  // CONE_DRIVE_TEST_COPIED__CUDA_YOLOX_PREPROCESSOR_HPP_
