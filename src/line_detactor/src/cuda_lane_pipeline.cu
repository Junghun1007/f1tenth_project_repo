#include "cuda_lane_pipeline.hpp"

#include <math_constants.h>

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

__device__ float owned_logit(
  const float * logits, const int x, const int y, const int lane,
  const int width, const int height, const float threshold)
{
  if (x < 0 || x >= width) {
    return -CUDART_INF_F;
  }
  const int pixel = y * width + x;
  const float left = logits[pixel];
  const float right = logits[width * height + pixel];
  const bool owned = lane == 0 ?
    (left >= threshold && (!(right >= threshold) || left >= right)) :
    (right >= threshold && (!(left >= threshold) || right > left));
  return owned ? (lane == 0 ? left : right) : -CUDART_INF_F;
}

__global__ void lane_rows_kernel(
  const float * logits, LaneRow * rows, const int width, const int height,
  const float threshold)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= 2 * height) {
    return;
  }
  const int lane = index / height;
  const int y = index % height;
  int runs = 0;
  int count = 0;
  bool previous = false;
  bool boundary = false;
  float mass = 0.0F;
  float moment = 0.0F;
  for (int x = 0; x < width; ++x) {
    const float logit = owned_logit(logits, x, y, lane, width, height, threshold);
    const bool valid = logit != -CUDART_INF_F;
    if (valid) {
      runs += previous ? 0 : 1;
      boundary = boundary || x == 0 || x == width - 1;
      const float probability = 1.0F / (1.0F + expf(-logit));
      mass += probability;
      moment += probability * static_cast<float>(x);
      ++count;
    }
    previous = valid;
  }
  LaneRow row{0.0F, 0.0F, 0.0F};
  if (runs == 1 && !boundary && mass > 0.0F) {
    row.center = moment / mass;
    row.weight = mass / static_cast<float>(count);
  }
  rows[index] = row;
}

// Fractional horizontal mask translation: keep the run width and make subpixel
// corrections visible without rounding beyond the configured displacement cap.
__device__ float2 shifted_lane_sample(
  const float * logits, const float source_x, const int y, const int lane,
  const int width, const int height, const float threshold)
{
  // Avoid converting an out-of-image float to an integer (also for large shifts).
  if (!(source_x > -1.0F && source_x < static_cast<float>(width))) {
    return make_float2(0.0F, 0.0F);
  }
  const int first = static_cast<int>(floorf(source_x));
  const float fraction = source_x - static_cast<float>(first);
  float2 result = make_float2(0.0F, 0.0F);
  for (int offset = 0; offset < 2; ++offset) {
    const float weight = offset == 0 ? 1.0F - fraction : fraction;
    const float logit = owned_logit(logits, first + offset, y, lane, width, height, threshold);
    if (weight > 0.0F && logit != -CUDART_INF_F) {
      result.x += weight;
      result.y += weight / (1.0F + expf(-logit));
    }
  }
  return result;
}

__global__ void lane_overlay_kernel(
  const std::uint8_t * bgr,
  const float * logits,
  std::uint8_t * preview,
  const int width,
  const int height,
  const float logit_threshold,
  const float alpha,
  const LaneRow * rows)
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
  if (rows && (rows[y].shift != 0.0F || rows[height + y].shift != 0.0F)) {
    const float2 left = shifted_lane_sample(
      logits, static_cast<float>(x) - rows[y].shift, y, 0, width, height, logit_threshold);
    const float2 right = shifted_lane_sample(
      logits, static_cast<float>(x) - rows[height + y].shift, y, 1, width, height,
      logit_threshold);
    const bool use_left = left.x > 0.0F && (right.x <= 0.0F || left.y >= right.y);
    const float coverage = use_left ? left.x : right.x;
    if (coverage > 0.0F) {
      preview[bgr_index] = blended_channel(
        bgr[bgr_index], use_left ? 255.0F : 0.0F, alpha * coverage);
      preview[bgr_index + 1U] = blended_channel(bgr[bgr_index + 1U], 0.0F, alpha * coverage);
      preview[bgr_index + 2U] = blended_channel(
        bgr[bgr_index + 2U], use_left ? 0.0F : 255.0F, alpha * coverage);
    } else {
      preview[bgr_index] = bgr[bgr_index];
      preview[bgr_index + 1U] = bgr[bgr_index + 1U];
      preview[bgr_index + 2U] = bgr[bgr_index + 2U];
    }
    return;
  }
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

namespace
{
float threshold_logit(const float threshold)
{
  if (threshold <= 0.0F) {
    return -std::numeric_limits<float>::infinity();
  }
  if (threshold >= 1.0F) {
    return std::numeric_limits<float>::infinity();
  }
  return logf(threshold / (1.0F - threshold));
}
}  // namespace

cudaError_t launch_lane_rows(
  const float * device_logits, LaneRow * device_rows, const int width,
  const int height, const float mask_threshold, const cudaStream_t stream) noexcept
{
  lane_rows_kernel<<<(2 * height + 127) / 128, 128, 0U, stream>>>(
    device_logits, device_rows, width, height, threshold_logit(mask_threshold));
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
  const LaneRow * device_rows,
  const cudaStream_t stream) noexcept
{
  const float logit_threshold = threshold_logit(mask_threshold);
  const dim3 block(16U, 16U);
  lane_overlay_kernel<<<grid_for(width, height), block, 0U, stream>>>(
    device_bgr, device_logits, device_preview_bgr, width, height,
    logit_threshold, overlay_alpha, device_rows);
  return cudaGetLastError();
}

}  // namespace line_detactor
