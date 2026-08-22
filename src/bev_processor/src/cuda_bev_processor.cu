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

__device__ float sampleChannelBilinear(
  const std::uint8_t * data,
  const int row_stride,
  const int width,
  const int height,
  const int pixel_stride,
  const int channel,
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
    data[y0 * row_stride + x0 * pixel_stride + channel] * (1.0F - dx) +
    data[y0 * row_stride + x1 * pixel_stride + channel] * dx;
  const float bottom =
    data[y1 * row_stride + x0 * pixel_stride + channel] * (1.0F - dx) +
    data[y1 * row_stride + x1 * pixel_stride + channel] * dx;
  return top * (1.0F - dy) + bottom * dy;
}

__device__ float cubicCatmullRom(
  const float p0,
  const float p1,
  const float p2,
  const float p3,
  const float amount)
{
  return p1 + 0.5F * amount * (
    p2 - p0 + amount * (
      2.0F * p0 - 5.0F * p1 + 4.0F * p2 - p3 +
      amount * (3.0F * (p1 - p2) + p3 - p0)));
}

__device__ float sampleChannelBicubic(
  const std::uint8_t * data,
  const int row_stride,
  const int width,
  const int height,
  const int pixel_stride,
  const int channel,
  const float x,
  const float y)
{
  const float clamped_x = clampFloat(x, 0.0F, width - 1.0F);
  const float clamped_y = clampFloat(y, 0.0F, height - 1.0F);
  const int base_x = static_cast<int>(floorf(clamped_x));
  const int base_y = static_cast<int>(floorf(clamped_y));
  const float dx = clamped_x - base_x;
  const float dy = clamped_y - base_y;
  float rows[4];

#pragma unroll
  for (int row_offset = -1; row_offset <= 2; ++row_offset) {
    const int sample_y = clampInt(base_y + row_offset, 0, height - 1);
    float samples[4];
#pragma unroll
    for (int column_offset = -1; column_offset <= 2; ++column_offset) {
      const int sample_x = clampInt(base_x + column_offset, 0, width - 1);
      samples[column_offset + 1] = static_cast<float>(
        data[sample_y * row_stride + sample_x * pixel_stride + channel]);
    }
    rows[row_offset + 1] = cubicCatmullRom(
      samples[0], samples[1], samples[2], samples[3], dx);
  }

  // Catmull-Rom can overshoot sharp image edges. Restrict the result to the
  // four pixels surrounding the sample position to avoid visible halos.
  const int next_x = clampInt(base_x + 1, 0, width - 1);
  const int next_y = clampInt(base_y + 1, 0, height - 1);
  const float center00 = static_cast<float>(
    data[base_y * row_stride + base_x * pixel_stride + channel]);
  const float center10 = static_cast<float>(
    data[base_y * row_stride + next_x * pixel_stride + channel]);
  const float center01 = static_cast<float>(
    data[next_y * row_stride + base_x * pixel_stride + channel]);
  const float center11 = static_cast<float>(
    data[next_y * row_stride + next_x * pixel_stride + channel]);
  const float local_minimum = fminf(
    fminf(center00, center10), fminf(center01, center11));
  const float local_maximum = fmaxf(
    fmaxf(center00, center10), fmaxf(center01, center11));
  return clampFloat(
    cubicCatmullRom(rows[0], rows[1], rows[2], rows[3], dy),
    local_minimum,
    local_maximum);
}

__device__ float sampleChannel(
  const std::uint8_t * data,
  const int row_stride,
  const int width,
  const int height,
  const int pixel_stride,
  const int channel,
  const float x,
  const float y,
  const bool bicubic)
{
  if (bicubic) {
    return sampleChannelBicubic(
      data, row_stride, width, height, pixel_stride, channel, x, y);
  }
  return sampleChannelBilinear(
    data, row_stride, width, height, pixel_stride, channel, x, y);
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
  const bool bicubic,
  std::uint8_t * output_bgr)
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
    return;
  }

  const std::uint8_t * y_plane = nv12;
  const std::uint8_t * uv_plane =
    nv12 + input_width * input_height;
  const float y = sampleChannel(
    y_plane,
    input_width,
    input_width,
    input_height,
    1,
    0,
    source_x,
    source_y,
    bicubic);
  const int chroma_width = input_width / 2;
  const int chroma_height = input_height / 2;
  const float chroma_x = source_x * 0.5F;
  const float chroma_y = source_y * 0.5F;
  const float u = sampleChannel(
    uv_plane,
    input_width,
    chroma_width,
    chroma_height,
    2,
    0,
    chroma_x,
    chroma_y,
    bicubic);
  const float v = sampleChannel(
    uv_plane,
    input_width,
    chroma_width,
    chroma_height,
    2,
    1,
    chroma_x,
    chroma_y,
    bicubic);

  const float c = fmaxf(0.0F, y - 16.0F);
  const float d = u - 128.0F;
  const float e = v - 128.0F;
  const float red = 1.164F * c + 1.596F * e;
  const float green = 1.164F * c - 0.392F * d - 0.813F * e;
  const float blue = 1.164F * c + 2.017F * d;

  destination[0] = static_cast<std::uint8_t>(
    clampFloat(blue, 0.0F, 255.0F));
  destination[1] = static_cast<std::uint8_t>(
    clampFloat(green, 0.0F, 255.0F));
  destination[2] = static_cast<std::uint8_t>(
    clampFloat(red, 0.0F, 255.0F));
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
    const BevInterpolation interpolation)
  : input_width_(input_width),
    input_height_(input_height),
    output_width_(map_x.cols),
    output_height_(map_x.rows),
    bicubic_(interpolation == BevInterpolation::Bicubic)
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

  cv::Mat process(
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
      bicubic_,
      device_output_);
    checkCuda(cudaGetLastError(), "launch NV12-to-BEV kernel");

    cv::Mat output(output_height_, output_width_, CV_8UC3);
    const std::size_t output_bytes =
      static_cast<std::size_t>(output_width_) *
      static_cast<std::size_t>(output_height_) * 3U;
    checkCuda(
      cudaMemcpyAsync(
        output.data,
        device_output_,
        output_bytes,
        cudaMemcpyDeviceToHost,
        stream_),
      "download BEV output");
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
  bool bicubic_;
  std::string device_name_;
  cudaStream_t stream_{nullptr};
  std::uint8_t * device_nv12_{nullptr};
  float * device_map_x_{nullptr};
  float * device_map_y_{nullptr};
  float * device_stabilized_to_source_{nullptr};
  std::uint8_t * device_output_{nullptr};
  std::mutex stream_mutex_;
};

CudaBevProcessor::CudaBevProcessor(
  const int input_width,
  const int input_height,
  const cv::Mat & map_x,
  const cv::Mat & map_y,
  const BevInterpolation interpolation)
: impl_(std::make_unique<Impl>(
    input_width, input_height, map_x, map_y, interpolation))
{
}

CudaBevProcessor::~CudaBevProcessor() = default;

cv::Mat CudaBevProcessor::process(
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
