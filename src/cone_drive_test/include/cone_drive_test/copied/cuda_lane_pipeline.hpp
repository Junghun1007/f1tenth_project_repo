// Copied from src/line_detactor/src/cuda_lane_pipeline.hpp (origin/1113ML)
#ifndef CONE_DRIVE_TEST_COPIED__CUDA_LANE_PIPELINE_HPP_
#define CONE_DRIVE_TEST_COPIED__CUDA_LANE_PIPELINE_HPP_

#include <cstdint>

#include <cuda_runtime_api.h>

namespace cone_drive_test
{

cudaError_t launch_bgr_to_rgb_nchw(
  const std::uint8_t * device_bgr,
  float * device_rgb,
  int width,
  int height,
  cudaStream_t stream) noexcept;

cudaError_t launch_lane_labels(
  const float * device_logits,
  // Two uint8 planes: legacy left/right labels, then independent 0/255 stop mask.
  std::uint8_t * device_labels,
  int width,
  int height,
  float mask_threshold,
  float stop_line_mask_threshold,
  cudaStream_t stream) noexcept;

cudaError_t launch_lane_overlay(
  const std::uint8_t * device_bgr,
  const float * device_logits,
  std::uint8_t * device_preview_bgr,
  int width,
  int height,
  float mask_threshold,
  float stop_line_mask_threshold,
  float overlay_alpha,
  cudaStream_t stream) noexcept;

}  // namespace cone_drive_test

#endif  // CONE_DRIVE_TEST_COPIED__CUDA_LANE_PIPELINE_HPP_
