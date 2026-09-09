#include "cuda_lane_pipeline.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace line_detactor
{
namespace
{

__device__ __forceinline__ std::uint8_t blended_channel(
  const std::uint8_t source,
  const float target,
  const float alpha)
{
  const float value =
    static_cast<float>(source) * (1.0F - alpha) + target * alpha;
  return static_cast<std::uint8_t>(fminf(fmaxf(value, 0.0F), 255.0F));
}

__global__ void bgr_to_rgb_nchw_kernel(
  const std::uint8_t * bgr,
  float * rgb,
  const int width,
  const int height)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) {
    return;
  }

  const std::size_t pixel =
    static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
    static_cast<std::size_t>(x);
  const std::size_t bgr_index = pixel * 3U;
  const std::size_t plane =
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  constexpr float scale = 1.0F / 255.0F;
  rgb[pixel] = static_cast<float>(bgr[bgr_index + 2U]) * scale;
  rgb[plane + pixel] = static_cast<float>(bgr[bgr_index + 1U]) * scale;
  rgb[2U * plane + pixel] = static_cast<float>(bgr[bgr_index]) * scale;
}

__global__ void lane_overlay_kernel(
  const std::uint8_t * bgr,
  const float * logits,
  std::uint8_t * preview,
  const int width,
  const int height,
  const float logit_threshold,
  const float alpha)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= width || y >= height) {
    return;
  }

  const std::size_t pixel =
    static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
    static_cast<std::size_t>(x);
  const std::size_t plane =
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  const std::size_t bgr_index = pixel * 3U;
  const float left_logit = logits[pixel];
  const float right_logit = logits[plane + pixel];
  const bool left = left_logit >= logit_threshold;
  const bool right = right_logit >= logit_threshold;

  float blue = static_cast<float>(bgr[bgr_index]);
  float green = static_cast<float>(bgr[bgr_index + 1U]);
  float red = static_cast<float>(bgr[bgr_index + 2U]);
  if (left && (!right || left_logit >= right_logit)) {
    blue = 255.0F;
    green = 0.0F;
    red = 0.0F;
  } else if (right) {
    blue = 0.0F;
    green = 0.0F;
    red = 255.0F;
  } else {
    preview[bgr_index] = bgr[bgr_index];
    preview[bgr_index + 1U] = bgr[bgr_index + 1U];
    preview[bgr_index + 2U] = bgr[bgr_index + 2U];
    return;
  }

  preview[bgr_index] = blended_channel(bgr[bgr_index], blue, alpha);
  preview[bgr_index + 1U] = blended_channel(
    bgr[bgr_index + 1U], green, alpha);
  preview[bgr_index + 2U] = blended_channel(
    bgr[bgr_index + 2U], red, alpha);
}

dim3 grid_for(const int width, const int height)
{
  constexpr unsigned int block_width = 16U;
  constexpr unsigned int block_height = 16U;
  return dim3(
    static_cast<unsigned int>(
      (width + static_cast<int>(block_width) - 1) /
      static_cast<int>(block_width)),
    static_cast<unsigned int>(
      (height + static_cast<int>(block_height) - 1) /
      static_cast<int>(block_height)));
}

}  // namespace

cudaError_t launch_bgr_to_rgb_nchw(
  const std::uint8_t * device_bgr,
  float * device_rgb,
  const int width,
  const int height,
  const cudaStream_t stream) noexcept
{
  const dim3 block(16U, 16U);
  bgr_to_rgb_nchw_kernel<<<grid_for(width, height), block, 0U, stream>>>(
    device_bgr, device_rgb, width, height);
  return cudaGetLastError();
}

cudaError_t launch_lane_overlay(
  const std::uint8_t * device_bgr,
  const float * device_logits,
  std::uint8_t * device_preview_bgr,
  const int width,
  const int height,
  const float mask_threshold,
  const float overlay_alpha,
  const cudaStream_t stream) noexcept
{
  float logit_threshold = 0.0F;
  if (mask_threshold <= 0.0F) {
    logit_threshold = -std::numeric_limits<float>::infinity();
  } else if (mask_threshold >= 1.0F) {
    logit_threshold = std::numeric_limits<float>::infinity();
  } else {
    logit_threshold = logf(mask_threshold / (1.0F - mask_threshold));
  }
  const dim3 block(16U, 16U);
  lane_overlay_kernel<<<grid_for(width, height), block, 0U, stream>>>(
    device_bgr, device_logits, device_preview_bgr, width, height,
    logit_threshold, overlay_alpha);
  return cudaGetLastError();
}

}  // namespace line_detactor
