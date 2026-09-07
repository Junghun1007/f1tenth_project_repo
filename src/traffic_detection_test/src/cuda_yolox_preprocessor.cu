#include "cuda_yolox_preprocessor.hpp"

#include <cstddef>
#include <cstdint>

namespace traffic_detection_test
{

namespace
{

__device__ __forceinline__ int clamp_u8(const int value)
{
  return value < 0 ? 0 : value > 255 ? 255 : value;
}

__global__ void nv12_to_bgr_nchw_kernel(
  const std::uint8_t * nv12,
  const std::size_t source_stride,
  const int source_width,
  const int source_height,
  float * output,
  const int output_width,
  const int output_height)
{
  const int x = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const int y = static_cast<int>(blockIdx.y * blockDim.y + threadIdx.y);
  if (x >= output_width || y >= output_height) {
    return;
  }

  const std::size_t plane_size =
    static_cast<std::size_t>(output_width) *
    static_cast<std::size_t>(output_height);
  const std::size_t output_index =
    static_cast<std::size_t>(y) * static_cast<std::size_t>(output_width) +
    static_cast<std::size_t>(x);

  float blue = 114.0F;
  float green = 114.0F;
  float red = 114.0F;
  if (x < source_width && y < source_height) {
    const int y_value = static_cast<int>(
      nv12[static_cast<std::size_t>(y) * source_stride +
      static_cast<std::size_t>(x)]);
    const std::size_t uv_offset =
      source_stride * static_cast<std::size_t>(source_height) +
      static_cast<std::size_t>(y / 2) * source_stride +
      static_cast<std::size_t>((x / 2) * 2);
    const int u_value = static_cast<int>(nv12[uv_offset]) - 128;
    const int v_value = static_cast<int>(nv12[uv_offset + 1U]) - 128;

    // Match OpenCV's limited-range BT.601 NV12 conversion before converting
    // the uint8 BGR result to the model's unnormalized FP32 tensor.
    constexpr int shift = 20;
    constexpr int rounding = 1 << (shift - 1);
    const int luminance = (y_value > 16 ? y_value - 16 : 0) * 1220542;
    const int blue_u8 = clamp_u8(
      (luminance + 2116026 * u_value + rounding) >> shift);
    const int green_u8 = clamp_u8(
      (luminance - 409993 * u_value - 852492 * v_value + rounding) >>
      shift);
    const int red_u8 = clamp_u8(
      (luminance + 1673527 * v_value + rounding) >> shift);
    blue = static_cast<float>(blue_u8);
    green = static_cast<float>(green_u8);
    red = static_cast<float>(red_u8);
  }

  output[output_index] = blue;
  output[plane_size + output_index] = green;
  output[2U * plane_size + output_index] = red;
}

}  // namespace

cudaError_t launch_nv12_to_bgr_nchw(
  const std::uint8_t * device_nv12,
  const std::size_t source_stride,
  const int source_width,
  const int source_height,
  float * device_output,
  const int output_width,
  const int output_height,
  const cudaStream_t stream) noexcept
{
  constexpr unsigned int block_width = 16U;
  constexpr unsigned int block_height = 16U;
  const dim3 block(block_width, block_height);
  const dim3 grid(
    static_cast<unsigned int>(
      (output_width + static_cast<int>(block_width) - 1) /
      static_cast<int>(block_width)),
    static_cast<unsigned int>(
      (output_height + static_cast<int>(block_height) - 1) /
      static_cast<int>(block_height)));
  nv12_to_bgr_nchw_kernel<<<grid, block, 0U, stream>>>(
    device_nv12, source_stride, source_width, source_height, device_output,
    output_width, output_height);
  return cudaGetLastError();
}

}  // namespace traffic_detection_test
