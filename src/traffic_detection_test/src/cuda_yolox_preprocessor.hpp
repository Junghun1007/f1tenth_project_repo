#ifndef TRAFFIC_DETECTION_TEST__CUDA_YOLOX_PREPROCESSOR_HPP_
#define TRAFFIC_DETECTION_TEST__CUDA_YOLOX_PREPROCESSOR_HPP_

#include <cstddef>
#include <cstdint>

#include <cuda_runtime_api.h>

namespace traffic_detection_test
{

cudaError_t launch_nv12_to_bgr_nchw(
  const std::uint8_t * device_nv12,
  std::size_t source_stride,
  int source_width,
  int source_height,
  float * device_output,
  int output_width,
  int output_height,
  cudaStream_t stream) noexcept;

}  // namespace traffic_detection_test

#endif  // TRAFFIC_DETECTION_TEST__CUDA_YOLOX_PREPROCESSOR_HPP_
