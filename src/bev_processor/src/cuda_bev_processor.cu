#include "bev_processor/cuda_bev_processor.hpp"

#include <cuda_runtime.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace bev_processor
{

namespace
{

void checkCuda(const cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(
            std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

__device__ float clampFloat(
  const float value,
  const float minimum,
  const float maximum)
{
  return fminf(maximum, fmaxf(minimum, value));
}

__device__ int clampInt(
  const int value,
  const int minimum,
  const int maximum)
{
  return value < minimum ? minimum : value > maximum ? maximum : value;
}

__device__ int borderIndex(const int index, const int length, const int border)
{
  if (index >= 0 && index < length) {
    return index;
  }
  if (border == 0) {
    return -1;
  }
  if (border == 1) {
    return clampInt(index, 0, length - 1);
  }
  int reflected = index;
  while (reflected < 0 || reflected >= length) {
    if (reflected < 0) {
      reflected = border == 2 ? -reflected - 1 : -reflected;
    } else {
      reflected = border == 2 ?
        2 * length - reflected - 1 : 2 * length - reflected - 2;
    }
  }
  return reflected;
}

__device__ std::uint8_t grayFromBgr(
  const std::uint8_t blue,
  const std::uint8_t green,
  const std::uint8_t red,
  const int mode)
{
  if (mode == 1) {
    return static_cast<std::uint8_t>(
      (static_cast<int>(blue) + static_cast<int>(green) +
      static_cast<int>(red) + 1) / 3);
  }
  if (mode == 2) {
    const std::uint8_t blue_green = blue > green ? blue : green;
    return blue_green > red ? blue_green : red;
  }
  constexpr int shift = 15;
  constexpr int blue_weight = 3735;
  constexpr int green_weight = 19235;
  constexpr int red_weight = 9798;
  return static_cast<std::uint8_t>(
    (blue_weight * static_cast<int>(blue) +
    green_weight * static_cast<int>(green) +
    red_weight * static_cast<int>(red) + (1 << (shift - 1))) >> shift);
}

__device__ float readPlane(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const int x,
  const int y)
{
  return static_cast<float>(plane[
      clampInt(y, 0, height - 1) * stride +
      clampInt(x, 0, width - 1)]);
}

__device__ float samplePlane(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const float x,
  const float y)
{
  const float clamped_x = clampFloat(x, 0.0F, width - 1.0F);
  const float clamped_y = clampFloat(y, 0.0F, height - 1.0F);
  const int x0 = static_cast<int>(floorf(clamped_x));
  const int y0 = static_cast<int>(floorf(clamped_y));
  const int x1 = x0 + 1 < width ? x0 + 1 : width - 1;
  const int y1 = y0 + 1 < height ? y0 + 1 : height - 1;
  const float dx = clamped_x - x0;
  const float dy = clamped_y - y0;

  const float top =
    plane[y0 * stride + x0] * (1.0F - dx) +
    plane[y0 * stride + x1] * dx;
  const float bottom =
    plane[y1 * stride + x0] * (1.0F - dx) +
    plane[y1 * stride + x1] * dx;
  return top * (1.0F - dy) + bottom * dy;
}

__device__ float cubicKernel(const float distance)
{
  constexpr float a = -0.5F;
  const float absolute = fabsf(distance);
  if (absolute <= 1.0F) {
    return
      (a + 2.0F) * absolute * absolute * absolute -
      (a + 3.0F) * absolute * absolute + 1.0F;
  }
  if (absolute < 2.0F) {
    return
      a * absolute * absolute * absolute -
      5.0F * a * absolute * absolute +
      8.0F * a * absolute - 4.0F * a;
  }
  return 0.0F;
}

__device__ void localRange2x2(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const float x,
  const float y,
  float & local_minimum,
  float & local_maximum)
{
  const int x0 = clampInt(static_cast<int>(floorf(x)), 0, width - 1);
  const int y0 = clampInt(static_cast<int>(floorf(y)), 0, height - 1);
  const int x1 = x0 + 1 < width ? x0 + 1 : width - 1;
  const int y1 = y0 + 1 < height ? y0 + 1 : height - 1;
  const float value00 = readPlane(plane, stride, width, height, x0, y0);
  const float value10 = readPlane(plane, stride, width, height, x1, y0);
  const float value01 = readPlane(plane, stride, width, height, x0, y1);
  const float value11 = readPlane(plane, stride, width, height, x1, y1);
  local_minimum = fminf(fminf(value00, value10), fminf(value01, value11));
  local_maximum = fmaxf(fmaxf(value00, value10), fmaxf(value01, value11));
}

__device__ float samplePlaneBicubic(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const float x,
  const float y)
{
  const int base_x = static_cast<int>(floorf(x));
  const int base_y = static_cast<int>(floorf(y));
  float weighted_sum = 0.0F;
  float total_weight = 0.0F;
#pragma unroll
  for (int row_offset = -1; row_offset <= 2; ++row_offset) {
    const float weight_y = cubicKernel(y - (base_y + row_offset));
#pragma unroll
    for (int column_offset = -1; column_offset <= 2; ++column_offset) {
      const float weight_x = cubicKernel(x - (base_x + column_offset));
      const float weight = weight_x * weight_y;
      weighted_sum += weight * readPlane(
        plane, stride, width, height,
        base_x + column_offset, base_y + row_offset);
      total_weight += weight;
    }
  }
  const float result = fabsf(total_weight) > 1.0e-6F ?
    weighted_sum / total_weight :
    samplePlane(plane, stride, width, height, x, y);
  float local_minimum = 0.0F;
  float local_maximum = 0.0F;
  localRange2x2(
    plane, stride, width, height, x, y, local_minimum, local_maximum);
  return clampFloat(result, local_minimum, local_maximum);
}

__device__ float smoothstep(
  const float edge0,
  const float edge1,
  const float value)
{
  if (edge1 <= edge0) {
    return value >= edge1 ? 1.0F : 0.0F;
  }
  const float amount = clampFloat((value - edge0) / (edge1 - edge0), 0.0F, 1.0F);
  return amount * amount * (3.0F - 2.0F * amount);
}

__device__ void sobelGradient(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const int x,
  const int y,
  float & gradient_x,
  float & gradient_y)
{
  const float top_left = readPlane(plane, stride, width, height, x - 1, y - 1);
  const float top = readPlane(plane, stride, width, height, x, y - 1);
  const float top_right = readPlane(plane, stride, width, height, x + 1, y - 1);
  const float left = readPlane(plane, stride, width, height, x - 1, y);
  const float right = readPlane(plane, stride, width, height, x + 1, y);
  const float bottom_left = readPlane(plane, stride, width, height, x - 1, y + 1);
  const float bottom = readPlane(plane, stride, width, height, x, y + 1);
  const float bottom_right = readPlane(plane, stride, width, height, x + 1, y + 1);
  gradient_x = 0.25F * (
    top_right + 2.0F * right + bottom_right -
    top_left - 2.0F * left - bottom_left);
  gradient_y = 0.25F * (
    bottom_left + 2.0F * bottom + bottom_right -
    top_left - 2.0F * top - top_right);
}

__device__ float samplePlaneAdaptive(
  const std::uint8_t * plane,
  const int stride,
  const int width,
  const int height,
  const float source_x,
  const float source_y,
  const float x_vehicle_m,
  const float start_x_m,
  const float full_x_m,
  const float strength,
  const float gradient_low,
  const float gradient_high,
  const float coherence_minimum,
  const float maximum_anisotropy)
{
  const float standard = samplePlaneBicubic(
    plane, stride, width, height, source_x, source_y);
  const int center_x = static_cast<int>(floorf(source_x + 0.5F));
  const int center_y = static_cast<int>(floorf(source_y + 0.5F));
  constexpr float gaussian_edge = 0.238994F;
  constexpr float gaussian_center = 0.522012F;
  float tensor_xx = 0.0F;
  float tensor_xy = 0.0F;
  float tensor_yy = 0.0F;
#pragma unroll
  for (int row_offset = -1; row_offset <= 1; ++row_offset) {
    const float weight_y = row_offset == 0 ? gaussian_center : gaussian_edge;
#pragma unroll
    for (int column_offset = -1; column_offset <= 1; ++column_offset) {
      const float weight_x = column_offset == 0 ? gaussian_center : gaussian_edge;
      float gradient_x = 0.0F;
      float gradient_y = 0.0F;
      sobelGradient(
        plane, stride, width, height,
        center_x + column_offset, center_y + row_offset,
        gradient_x, gradient_y);
      const float weight = weight_x * weight_y;
      tensor_xx += weight * gradient_x * gradient_x;
      tensor_xy += weight * gradient_x * gradient_y;
      tensor_yy += weight * gradient_y * gradient_y;
    }
  }

  const float trace = fmaxf(0.0F, tensor_xx + tensor_yy);
  const float eigen_gap = sqrtf(fmaxf(
      0.0F,
      (tensor_xx - tensor_yy) * (tensor_xx - tensor_yy) +
      4.0F * tensor_xy * tensor_xy));
  const float largest_eigenvalue = 0.5F * (trace + eigen_gap);
  const float gradient = fminf(255.0F, sqrtf(fmaxf(0.0F, largest_eigenvalue)));
  const float coherence = eigen_gap / (trace + 1.0e-9F);
  const float blend = clampFloat(
    strength *
    smoothstep(gradient_low, gradient_high, gradient) *
    smoothstep(coherence_minimum, 1.0F, coherence) *
    smoothstep(start_x_m, full_x_m, x_vehicle_m),
    0.0F,
    1.0F);
  if (blend <= 1.0e-6F) {
    return standard;
  }

  const float normal_angle = 0.5F * atan2f(
    2.0F * tensor_xy, tensor_xx - tensor_yy);
  const float normal_x = cosf(normal_angle);
  const float normal_y = sinf(normal_angle);
  const float anisotropy = 1.0F + (maximum_anisotropy - 1.0F) * blend;
  const int base_x = static_cast<int>(floorf(source_x));
  const int base_y = static_cast<int>(floorf(source_y));
  float weighted_sum = 0.0F;
  float total_weight = 0.0F;
#pragma unroll
  for (int row_offset = -1; row_offset <= 2; ++row_offset) {
    const float delta_y = base_y + row_offset - source_y;
    const float weight_y = cubicKernel(-delta_y);
#pragma unroll
    for (int column_offset = -1; column_offset <= 2; ++column_offset) {
      const float delta_x = base_x + column_offset - source_x;
      const float weight_x = cubicKernel(-delta_x);
      const float normal_distance = delta_x * normal_x + delta_y * normal_y;
      const float directional_weight = expf(
        -0.5F * (anisotropy - 1.0F) * normal_distance * normal_distance);
      const float weight = weight_x * weight_y * directional_weight;
      weighted_sum += weight * readPlane(
        plane, stride, width, height,
        base_x + column_offset, base_y + row_offset);
      total_weight += weight;
    }
  }
  if (fabsf(total_weight) <= 1.0e-6F) {
    return standard;
  }
  float local_minimum = 0.0F;
  float local_maximum = 0.0F;
  localRange2x2(
    plane, stride, width, height, source_x, source_y,
    local_minimum, local_maximum);
  const float adaptive = clampFloat(
    weighted_sum / total_weight, local_minimum, local_maximum);
  return (1.0F - blend) * standard + blend * adaptive;
}

__device__ float sampleChroma(
  const std::uint8_t * uv_plane,
  const int stride,
  const int width,
  const int height,
  const float source_x,
  const float source_y,
  const int channel)
{
  const int chroma_width = width / 2;
  const int chroma_height = height / 2;
  const float x = clampFloat(
    source_x * 0.5F, 0.0F, chroma_width - 1.0F);
  const float y = clampFloat(
    source_y * 0.5F, 0.0F, chroma_height - 1.0F);
  const int x0 = static_cast<int>(floorf(x));
  const int y0 = static_cast<int>(floorf(y));
  const int x1 = x0 + 1 < chroma_width ? x0 + 1 : chroma_width - 1;
  const int y1 = y0 + 1 < chroma_height ? y0 + 1 : chroma_height - 1;
  const float dx = x - x0;
  const float dy = y - y0;

  const float top =
    uv_plane[y0 * stride + x0 * 2 + channel] * (1.0F - dx) +
    uv_plane[y0 * stride + x1 * 2 + channel] * dx;
  const float bottom =
    uv_plane[y1 * stride + x0 * 2 + channel] * (1.0F - dx) +
    uv_plane[y1 * stride + x1 * 2 + channel] * dx;
  return top * (1.0F - dy) + bottom * dy;
}

__global__ void nv12ToBevKernel(
  const std::uint8_t * nv12,
  const int input_width,
  const int input_height,
  const float * map_x,
  const float * map_y,
  const float * stabilized_to_source_homography,
  const int source_crop_top,
  const int output_width,
  const int output_height,
  const int interpolation,
  const float adaptive_start_x_m,
  const float adaptive_full_x_m,
  const float adaptive_strength,
  const float adaptive_gradient_low,
  const float adaptive_gradient_high,
  const float adaptive_coherence_minimum,
  const float adaptive_maximum_anisotropy,
  const float bev_x_max_m,
  const float meter_per_pixel,
  const int gray_mode,
  std::uint8_t * output_bgr,
  std::uint8_t * output_gray)
{
  const int output_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int output_y = blockIdx.y * blockDim.y + threadIdx.y;
  if (output_x >= output_width || output_y >= output_height) {
    return;
  }

  const int output_index = output_y * output_width + output_x;
  const float stabilized_x = map_x[output_index];
  const float stabilized_y = map_y[output_index];
  std::uint8_t * destination = output_bgr + output_index * 3;
  if (
    stabilized_x < 0.0F || stabilized_y < 0.0F)
  {
    destination[0] = 0U;
    destination[1] = 0U;
    destination[2] = 0U;
    output_gray[output_index] = 0U;
    return;
  }

  const float denominator =
    stabilized_to_source_homography[6] * stabilized_x +
    stabilized_to_source_homography[7] * stabilized_y +
    stabilized_to_source_homography[8];
  if (!isfinite(denominator) || fabsf(denominator) < 1.0e-6F) {
    destination[0] = 0U;
    destination[1] = 0U;
    destination[2] = 0U;
    output_gray[output_index] = 0U;
    return;
  }
  const float source_x =
    (stabilized_to_source_homography[0] * stabilized_x +
    stabilized_to_source_homography[1] * stabilized_y +
    stabilized_to_source_homography[2]) / denominator;
  const float source_y_full =
    (stabilized_to_source_homography[3] * stabilized_x +
    stabilized_to_source_homography[4] * stabilized_y +
    stabilized_to_source_homography[5]) / denominator;
  const float source_y = source_y_full - source_crop_top;
  if (
    !isfinite(source_x) || !isfinite(source_y) ||
    source_x < 0.0F || source_y < 0.0F ||
    source_x >= input_width - 1.0F ||
    source_y >= input_height - 1.0F)
  {
    destination[0] = 0U;
    destination[1] = 0U;
    destination[2] = 0U;
    output_gray[output_index] = 0U;
    return;
  }

  const std::uint8_t * y_plane = nv12;
  const std::uint8_t * uv_plane =
    nv12 + input_width * input_height;
  float y = 0.0F;
  if (interpolation == static_cast<int>(BevInterpolation::Adaptive)) {
    const float x_vehicle_m =
      bev_x_max_m - (output_y + 0.5F) * meter_per_pixel;
    y = samplePlaneAdaptive(
      y_plane,
      input_width,
      input_width,
      input_height,
      source_x,
      source_y,
      x_vehicle_m,
      adaptive_start_x_m,
      adaptive_full_x_m,
      adaptive_strength,
      adaptive_gradient_low,
      adaptive_gradient_high,
      adaptive_coherence_minimum,
      adaptive_maximum_anisotropy);
  } else if (interpolation == static_cast<int>(BevInterpolation::Bicubic)) {
    y = samplePlaneBicubic(
      y_plane,
      input_width,
      input_width,
      input_height,
      source_x,
      source_y);
  } else {
    y = samplePlane(
      y_plane,
      input_width,
      input_width,
      input_height,
      source_x,
      source_y);
  }
  const float u = sampleChroma(
    uv_plane,
    input_width,
    input_width,
    input_height,
    source_x,
    source_y,
    0);
  const float v = sampleChroma(
    uv_plane,
    input_width,
    input_width,
    input_height,
    source_x,
    source_y,
    1);

  const float c = fmaxf(0.0F, y - 16.0F);
  const float d = u - 128.0F;
  const float e = v - 128.0F;
  const float red = 1.164F * c + 1.596F * e;
  const float green = 1.164F * c - 0.392F * d - 0.813F * e;
  const float blue = 1.164F * c + 2.017F * d;

  const std::uint8_t output_blue = static_cast<std::uint8_t>(
    clampFloat(blue, 0.0F, 255.0F));
  const std::uint8_t output_green = static_cast<std::uint8_t>(
    clampFloat(green, 0.0F, 255.0F));
  const std::uint8_t output_red = static_cast<std::uint8_t>(
    clampFloat(red, 0.0F, 255.0F));
  destination[0] = output_blue;
  destination[1] = output_green;
  destination[2] = output_red;
  output_gray[output_index] = grayFromBgr(
    output_blue, output_green, output_red, gray_mode);
}

__global__ void morphologyPassKernel(
  const std::uint8_t * input,
  std::uint8_t * output,
  const int width,
  const int height,
  const std::uint8_t * kernel_mask,
  const int kernel_width,
  const int kernel_height,
  const int border_type,
  const bool erosion)
{
  const int column = blockIdx.x * blockDim.x + threadIdx.x;
  const int row = blockIdx.y * blockDim.y + threadIdx.y;
  if (column >= width || row >= height) {
    return;
  }
  const int anchor_x = kernel_width / 2;
  const int anchor_y = kernel_height / 2;
  std::uint8_t result = erosion ? 255U : 0U;
  for (int kernel_y = 0; kernel_y < kernel_height; ++kernel_y) {
    for (int kernel_x = 0; kernel_x < kernel_width; ++kernel_x) {
      if (kernel_mask[kernel_y * kernel_width + kernel_x] == 0U) {
        continue;
      }
      const int source_x = borderIndex(
        column + kernel_x - anchor_x, width, border_type);
      const int source_y = borderIndex(
        row + kernel_y - anchor_y, height, border_type);
      const std::uint8_t value = source_x < 0 || source_y < 0 ?
        (erosion ? 255U : 0U) : input[source_y * width + source_x];
      result = erosion ?
        (result < value ? result : value) :
        (result > value ? result : value);
    }
  }
  output[row * width + column] = result;
}

__global__ void enhanceTopHatBandKernel(
  const std::uint8_t * gray,
  const std::uint8_t * opened,
  std::uint8_t * enhanced,
  const int width,
  const int row_begin,
  const int row_end,
  const int noise_floor,
  const float gain)
{
  const int column = blockIdx.x * blockDim.x + threadIdx.x;
  const int row = row_begin + blockIdx.y * blockDim.y + threadIdx.y;
  if (column >= width || row >= row_end) {
    return;
  }
  const int index = row * width + column;
  const int difference =
    static_cast<int>(gray[index]) - static_cast<int>(opened[index]);
  const int top_hat = difference > 0 ? difference : 0;
  const int above_floor = top_hat - noise_floor;
  const float amplified = (above_floor > 0 ? above_floor : 0) * gain;
  enhanced[index] = static_cast<std::uint8_t>(
    amplified < 254.5F ? amplified + 0.5F : 255.0F);
}

bool isOddPositive(const int value)
{
  return value > 0 && value % 2 == 1;
}

void validateLanePreprocess(const CudaLanePreprocessConfig & config)
{
  const auto validGain = [](const double value) {
      return std::isfinite(value) && value > 0.0;
    };
  const auto validNoiseFloor = [](const int value) {
      return value >= 0 && value <= 255;
    };
  if (config.gray_mode < 0 || config.gray_mode > 2) {
    throw std::invalid_argument("lane gray mode must be 0, 1, or 2");
  }
  const double ratio_sum =
    config.near_ratio + config.middle_ratio + config.far_ratio;
  if (
    !std::isfinite(config.near_ratio) || config.near_ratio < 0.0 ||
    !std::isfinite(config.middle_ratio) || config.middle_ratio < 0.0 ||
    !std::isfinite(config.far_ratio) || config.far_ratio < 0.0 ||
    std::abs(ratio_sum - 1.0) > 1.0e-6)
  {
    throw std::invalid_argument(
            "lane near/middle/far ratios must be non-negative and sum to 1");
  }
  if (
    !validGain(config.near_gain) || !validGain(config.middle_gain) ||
    !validGain(config.far_gain) ||
    !validNoiseFloor(config.near_noise_floor) ||
    !validNoiseFloor(config.middle_noise_floor) ||
    !validNoiseFloor(config.far_noise_floor))
  {
    throw std::invalid_argument("lane Top-hat gain or noise floor is invalid");
  }
  if (
    !isOddPositive(config.near_kernel_width) ||
    !isOddPositive(config.near_kernel_height) ||
    !isOddPositive(config.middle_kernel_width) ||
    !isOddPositive(config.middle_kernel_height) ||
    !isOddPositive(config.far_kernel_width) ||
    !isOddPositive(config.far_kernel_height))
  {
    throw std::invalid_argument(
            "lane Top-hat kernel dimensions must be positive odd numbers");
  }
  if (
    config.top_hat_kernel_shape < 0 || config.top_hat_kernel_shape > 2 ||
    config.top_hat_iterations < 1 || config.top_hat_iterations > 10 ||
    config.top_hat_border_type < 0 || config.top_hat_border_type > 3)
  {
    throw std::invalid_argument("lane Top-hat shared settings are invalid");
  }
}

std::vector<std::uint8_t> makeKernelMask(
  const int width,
  const int height,
  const int shape)
{
  const int opencv_shape = shape == 0 ? cv::MORPH_RECT :
    shape == 1 ? cv::MORPH_ELLIPSE : cv::MORPH_CROSS;
  const cv::Mat mask = cv::getStructuringElement(
    opencv_shape, cv::Size(width, height));
  return std::vector<std::uint8_t>(mask.datastart, mask.dataend);
}

}  // namespace

class CudaBevProcessor::Impl
{
public:
  Impl(
    const int input_width,
    const int input_height,
    const cv::Mat & map_x,
    const cv::Mat & map_y,
    const BevInterpolation interpolation,
    const EdgeAdaptiveConfig & edge_adaptive_config,
    const CudaLanePreprocessConfig & lane_preprocess_config)
  : input_width_(input_width),
    input_height_(input_height),
    output_width_(map_x.cols),
    output_height_(map_x.rows),
    interpolation_(interpolation),
    edge_adaptive_config_(edge_adaptive_config),
    lane_preprocess_config_(lane_preprocess_config)
  {
    if (
      input_width_ <= 0 || input_height_ <= 0 ||
      input_width_ % 2 != 0 || input_height_ % 2 != 0)
    {
      throw std::invalid_argument(
              "CUDA NV12 input dimensions must be positive and even");
    }
    if (
      map_x.empty() || map_y.empty() ||
      map_x.size() != map_y.size() ||
      map_x.type() != CV_32FC1 ||
      map_y.type() != CV_32FC1)
    {
      throw std::invalid_argument(
              "CUDA BEV maps must be equal-sized CV_32FC1 matrices");
    }
    validateLanePreprocess(lane_preprocess_config_);

    try {
      int device = 0;
      checkCuda(cudaGetDevice(&device), "cudaGetDevice");
      cudaDeviceProp properties{};
      checkCuda(
        cudaGetDeviceProperties(&properties, device),
        "cudaGetDeviceProperties");
      device_name_ = properties.name;

      checkCuda(
        cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking),
        "cudaStreamCreateWithFlags");

      const std::size_t nv12_bytes =
        static_cast<std::size_t>(input_width_) *
        static_cast<std::size_t>(input_height_) * 3U / 2U;
      const std::size_t map_bytes =
        static_cast<std::size_t>(output_width_) *
        static_cast<std::size_t>(output_height_) * sizeof(float);
      const std::size_t output_bytes =
        static_cast<std::size_t>(output_width_) *
        static_cast<std::size_t>(output_height_) * 3U;
      const std::size_t mono_bytes =
        static_cast<std::size_t>(output_width_) *
        static_cast<std::size_t>(output_height_);

      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_nv12_),
          nv12_bytes),
        "cudaMalloc NV12");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_map_x_),
          map_bytes),
        "cudaMalloc map_x");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_map_y_),
          map_bytes),
        "cudaMalloc map_y");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_stabilized_to_source_),
          9U * sizeof(float)),
        "cudaMalloc stabilized-to-source homography");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_output_),
          output_bytes),
        "cudaMalloc BEV output");
      checkCuda(
        cudaMalloc(
          reinterpret_cast<void **>(&device_gray_),
          mono_bytes),
        "cudaMalloc BEV gray output");
      if (lane_preprocess_config_.enabled) {
        checkCuda(
          cudaMalloc(
            reinterpret_cast<void **>(&device_morphology_a_),
            mono_bytes),
          "cudaMalloc lane morphology A");
        checkCuda(
          cudaMalloc(
            reinterpret_cast<void **>(&device_morphology_b_),
            mono_bytes),
          "cudaMalloc lane morphology B");
        checkCuda(
          cudaMalloc(
            reinterpret_cast<void **>(&device_enhanced_top_hat_),
            mono_bytes),
          "cudaMalloc lane enhanced Top-hat");

        const std::array<std::vector<std::uint8_t>, 3> masks{
          makeKernelMask(
            lane_preprocess_config_.near_kernel_width,
            lane_preprocess_config_.near_kernel_height,
            lane_preprocess_config_.top_hat_kernel_shape),
          makeKernelMask(
            lane_preprocess_config_.middle_kernel_width,
            lane_preprocess_config_.middle_kernel_height,
            lane_preprocess_config_.top_hat_kernel_shape),
          makeKernelMask(
            lane_preprocess_config_.far_kernel_width,
            lane_preprocess_config_.far_kernel_height,
            lane_preprocess_config_.top_hat_kernel_shape)};
        for (std::size_t index = 0; index < masks.size(); ++index) {
          checkCuda(
            cudaMalloc(
              reinterpret_cast<void **>(&device_kernel_masks_[index]),
              masks[index].size()),
            "cudaMalloc lane morphology kernel mask");
          checkCuda(
            cudaMemcpyAsync(
              device_kernel_masks_[index], masks[index].data(),
              masks[index].size(), cudaMemcpyHostToDevice, stream_),
            "upload lane morphology kernel mask");
        }
      }

      const cv::Mat continuous_map_x =
        map_x.isContinuous() ? map_x : map_x.clone();
      const cv::Mat continuous_map_y =
        map_y.isContinuous() ? map_y : map_y.clone();
      checkCuda(
        cudaMemcpyAsync(
          device_map_x_,
          continuous_map_x.ptr<float>(),
          map_bytes,
          cudaMemcpyHostToDevice,
          stream_),
        "cudaMemcpyAsync map_x");
      checkCuda(
        cudaMemcpyAsync(
          device_map_y_,
          continuous_map_y.ptr<float>(),
          map_bytes,
          cudaMemcpyHostToDevice,
          stream_),
        "cudaMemcpyAsync map_y");
      checkCuda(cudaStreamSynchronize(stream_), "upload BEV maps");
    } catch (...) {
      release();
      throw;
    }
  }

  ~Impl()
  {
    release();
  }

  CudaBevResult process(
    const std::uint8_t * nv12,
    const std::size_t data_size,
    const std::size_t input_stride,
    const cv::Matx33d & stabilized_to_source_homography,
    const int source_crop_top)
  {
    if (nv12 == nullptr || input_stride <
      static_cast<std::size_t>(input_width_))
    {
      throw std::invalid_argument("invalid host NV12 buffer or stride");
    }
    const std::size_t input_rows =
      static_cast<std::size_t>(input_height_) * 3U / 2U;
    if (data_size < input_stride * input_rows) {
      throw std::invalid_argument("host NV12 buffer is smaller than expected");
    }
    if (
      source_crop_top < 0 || source_crop_top % 2 != 0 ||
      !cv::checkRange(cv::Mat(stabilized_to_source_homography)))
    {
      throw std::invalid_argument(
              "invalid source crop or stabilized-to-source homography");
    }

    std::array<float, 9> homography{};
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        homography[static_cast<std::size_t>(row * 3 + column)] =
          static_cast<float>(
          stabilized_to_source_homography(row, column));
      }
    }

    std::lock_guard<std::mutex> lock(stream_mutex_);
    checkCuda(
      cudaMemcpy2DAsync(
        device_nv12_,
        static_cast<std::size_t>(input_width_),
        nv12,
        input_stride,
        static_cast<std::size_t>(input_width_),
        input_rows,
        cudaMemcpyHostToDevice,
      stream_),
      "upload NV12 frame");
    checkCuda(
      cudaMemcpyAsync(
        device_stabilized_to_source_,
        homography.data(),
        homography.size() * sizeof(float),
        cudaMemcpyHostToDevice,
        stream_),
      "upload stabilized-to-source homography");

    const dim3 block(16U, 16U);
    const dim3 grid(
      static_cast<unsigned int>((output_width_ + 15) / 16),
      static_cast<unsigned int>((output_height_ + 15) / 16));
    nv12ToBevKernel<<<grid, block, 0, stream_>>>(
      device_nv12_,
      input_width_,
      input_height_,
      device_map_x_,
      device_map_y_,
      device_stabilized_to_source_,
      source_crop_top,
      output_width_,
      output_height_,
      static_cast<int>(interpolation_),
      static_cast<float>(edge_adaptive_config_.start_x_m),
      static_cast<float>(edge_adaptive_config_.full_x_m),
      static_cast<float>(edge_adaptive_config_.strength),
      static_cast<float>(edge_adaptive_config_.gradient_low),
      static_cast<float>(edge_adaptive_config_.gradient_high),
      static_cast<float>(edge_adaptive_config_.coherence_minimum),
      static_cast<float>(edge_adaptive_config_.maximum_anisotropy),
      static_cast<float>(edge_adaptive_config_.bev_x_max_m),
      static_cast<float>(edge_adaptive_config_.meter_per_pixel),
      lane_preprocess_config_.gray_mode,
      device_output_,
      device_gray_);
    checkCuda(cudaGetLastError(), "launch NV12-to-BEV kernel");

    if (lane_preprocess_config_.enabled) {
      const auto processBand = [this, &block, &grid](
          const std::size_t kernel_index,
          const int kernel_width,
          const int kernel_height,
          const int row_begin,
          const int row_end,
          const int noise_floor,
          const double gain) {
          if (row_end <= row_begin) {
            return;
          }
          const std::uint8_t * current = device_gray_;
          std::uint8_t * destination = device_morphology_a_;
          for (int pass = 0;
            pass < lane_preprocess_config_.top_hat_iterations; ++pass)
          {
            morphologyPassKernel<<<grid, block, 0, stream_>>>(
              current, destination, output_width_, output_height_,
              device_kernel_masks_[kernel_index], kernel_width, kernel_height,
              lane_preprocess_config_.top_hat_border_type, true);
            checkCuda(cudaGetLastError(), "launch lane erosion kernel");
            current = destination;
            destination = destination == device_morphology_a_ ?
              device_morphology_b_ : device_morphology_a_;
          }
          for (int pass = 0;
            pass < lane_preprocess_config_.top_hat_iterations; ++pass)
          {
            morphologyPassKernel<<<grid, block, 0, stream_>>>(
              current, destination, output_width_, output_height_,
              device_kernel_masks_[kernel_index], kernel_width, kernel_height,
              lane_preprocess_config_.top_hat_border_type, false);
            checkCuda(cudaGetLastError(), "launch lane dilation kernel");
            current = destination;
            destination = destination == device_morphology_a_ ?
              device_morphology_b_ : device_morphology_a_;
          }
          const dim3 band_grid(
            static_cast<unsigned int>((output_width_ + 15) / 16),
            static_cast<unsigned int>((row_end - row_begin + 15) / 16));
          enhanceTopHatBandKernel<<<band_grid, block, 0, stream_>>>(
            device_gray_, current, device_enhanced_top_hat_, output_width_,
            row_begin, row_end, noise_floor, static_cast<float>(gain));
          checkCuda(cudaGetLastError(), "launch lane Top-hat enhancement kernel");
        };

      const int far_end = std::clamp(
        static_cast<int>(std::lround(
          output_height_ * lane_preprocess_config_.far_ratio)),
        0, output_height_);
      const int middle_end = std::clamp(
        static_cast<int>(std::lround(
          output_height_ * (
            lane_preprocess_config_.far_ratio +
            lane_preprocess_config_.middle_ratio))),
        far_end, output_height_);
      processBand(
        2U,
        lane_preprocess_config_.far_kernel_width,
        lane_preprocess_config_.far_kernel_height,
        0, far_end,
        lane_preprocess_config_.far_noise_floor,
        lane_preprocess_config_.far_gain);
      processBand(
        1U,
        lane_preprocess_config_.middle_kernel_width,
        lane_preprocess_config_.middle_kernel_height,
        far_end, middle_end,
        lane_preprocess_config_.middle_noise_floor,
        lane_preprocess_config_.middle_gain);
      processBand(
        0U,
        lane_preprocess_config_.near_kernel_width,
        lane_preprocess_config_.near_kernel_height,
        middle_end, output_height_,
        lane_preprocess_config_.near_noise_floor,
        lane_preprocess_config_.near_gain);
    }

    CudaBevResult output;
    output.bgr.create(output_height_, output_width_, CV_8UC3);
    const std::size_t output_bytes =
      static_cast<std::size_t>(output_width_) *
      static_cast<std::size_t>(output_height_) * 3U;
    checkCuda(
      cudaMemcpyAsync(
        output.bgr.data,
        device_output_,
        output_bytes,
        cudaMemcpyDeviceToHost,
        stream_),
      "download BEV output");
    if (lane_preprocess_config_.enabled) {
      output.gray.create(output_height_, output_width_, CV_8UC1);
      output.enhanced_top_hat.create(
        output_height_, output_width_, CV_8UC1);
      const std::size_t mono_bytes =
        static_cast<std::size_t>(output_width_) *
        static_cast<std::size_t>(output_height_);
      checkCuda(
        cudaMemcpyAsync(
          output.gray.data, device_gray_, mono_bytes,
          cudaMemcpyDeviceToHost, stream_),
        "download BEV gray output");
      checkCuda(
        cudaMemcpyAsync(
          output.enhanced_top_hat.data, device_enhanced_top_hat_, mono_bytes,
          cudaMemcpyDeviceToHost, stream_),
        "download BEV enhanced Top-hat output");
    }
    checkCuda(cudaStreamSynchronize(stream_), "process NV12 BEV frame");
    return output;
  }

  const std::string & deviceName() const
  {
    return device_name_;
  }

private:
  void release() noexcept
  {
    for (std::uint8_t *& mask : device_kernel_masks_) {
      if (mask != nullptr) {
        cudaFree(mask);
        mask = nullptr;
      }
    }
    if (device_enhanced_top_hat_ != nullptr) {
      cudaFree(device_enhanced_top_hat_);
      device_enhanced_top_hat_ = nullptr;
    }
    if (device_morphology_b_ != nullptr) {
      cudaFree(device_morphology_b_);
      device_morphology_b_ = nullptr;
    }
    if (device_morphology_a_ != nullptr) {
      cudaFree(device_morphology_a_);
      device_morphology_a_ = nullptr;
    }
    if (device_gray_ != nullptr) {
      cudaFree(device_gray_);
      device_gray_ = nullptr;
    }
    if (device_output_ != nullptr) {
      cudaFree(device_output_);
      device_output_ = nullptr;
    }
    if (device_map_y_ != nullptr) {
      cudaFree(device_map_y_);
      device_map_y_ = nullptr;
    }
    if (device_stabilized_to_source_ != nullptr) {
      cudaFree(device_stabilized_to_source_);
      device_stabilized_to_source_ = nullptr;
    }
    if (device_map_x_ != nullptr) {
      cudaFree(device_map_x_);
      device_map_x_ = nullptr;
    }
    if (device_nv12_ != nullptr) {
      cudaFree(device_nv12_);
      device_nv12_ = nullptr;
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }
  }

  int input_width_;
  int input_height_;
  int output_width_;
  int output_height_;
  BevInterpolation interpolation_;
  EdgeAdaptiveConfig edge_adaptive_config_;
  CudaLanePreprocessConfig lane_preprocess_config_;
  std::string device_name_;
  cudaStream_t stream_{nullptr};
  std::uint8_t * device_nv12_{nullptr};
  float * device_map_x_{nullptr};
  float * device_map_y_{nullptr};
  float * device_stabilized_to_source_{nullptr};
  std::uint8_t * device_output_{nullptr};
  std::uint8_t * device_gray_{nullptr};
  std::uint8_t * device_morphology_a_{nullptr};
  std::uint8_t * device_morphology_b_{nullptr};
  std::uint8_t * device_enhanced_top_hat_{nullptr};
  std::array<std::uint8_t *, 3> device_kernel_masks_{};
  std::mutex stream_mutex_;
};

CudaBevProcessor::CudaBevProcessor(
  const int input_width,
  const int input_height,
  const cv::Mat & map_x,
  const cv::Mat & map_y,
  const BevInterpolation interpolation,
  const EdgeAdaptiveConfig & edge_adaptive_config,
  const CudaLanePreprocessConfig & lane_preprocess_config)
: impl_(std::make_unique<Impl>(
    input_width, input_height, map_x, map_y, interpolation,
    edge_adaptive_config, lane_preprocess_config))
{
}

CudaBevProcessor::~CudaBevProcessor() = default;

CudaBevResult CudaBevProcessor::process(
  const std::uint8_t * nv12,
  const std::size_t data_size,
  const std::size_t input_stride,
  const cv::Matx33d & stabilized_to_source_homography,
  const int source_crop_top)
{
  return impl_->process(
    nv12,
    data_size,
    input_stride,
    stabilized_to_source_homography,
    source_crop_top);
}

const std::string & CudaBevProcessor::deviceName() const
{
  return impl_->deviceName();
}

}  // namespace bev_processor
