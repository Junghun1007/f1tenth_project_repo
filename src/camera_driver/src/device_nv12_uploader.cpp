#include "camera_driver/device_nv12_uploader.hpp"
#include <cuda_runtime_api.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace camera_driver {

namespace {
void check(cudaError_t e, const char * what) {
  if (e != cudaSuccess) throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(e));
}
}  // namespace

// 버퍼 풀: 받는 쪽이 owner를 놓으면 빈 목록으로 돌아간다. 업로더가 먼저 없어져도 마지막 owner가 놓일 때 해제
struct DeviceNv12Uploader::Pool {
  std::mutex mutex;
  std::vector<std::uint8_t *> buffers, free_list;
  cudaStream_t stream{nullptr};
  ~Pool() {
    for (auto * b : buffers) cudaFree(b);
    if (stream) cudaStreamDestroy(stream);
  }
};

DeviceNv12Uploader::DeviceNv12Uploader(int width, int height, int buffer_count)
: width_(width), height_(height), pool_(std::make_shared<Pool>()) {
  if (width <= 0 || height <= 0 || width % 2 || height % 2 || buffer_count < 2) {
    throw std::invalid_argument("DeviceNv12Uploader: invalid NV12 size or buffer count");
  }
  check(cudaStreamCreateWithFlags(&pool_->stream, cudaStreamNonBlocking), "cudaStreamCreate (NV12 upload)");
  const std::size_t bytes = std::size_t(width) * height * 3 / 2;
  for (int i = 0; i < buffer_count; ++i) {
    void * p = nullptr;
    check(cudaMalloc(&p, bytes), "cudaMalloc (shared NV12)");
    pool_->buffers.push_back(static_cast<std::uint8_t *>(p));
    pool_->free_list.push_back(static_cast<std::uint8_t *>(p));
  }
}

DeviceNv12Uploader::~DeviceNv12Uploader() = default;

std::shared_ptr<const bev_handoff::DeviceNv12Frame> DeviceNv12Uploader::upload(
  const std::uint8_t * nv12, std::size_t stride, std::uint64_t id, std::chrono::steady_clock::time_point captured_at)
{
  if (nv12 == nullptr || stride < std::size_t(width_)) throw std::invalid_argument("DeviceNv12Uploader: invalid host frame");
  std::uint8_t * buffer = nullptr;
  {
    std::lock_guard<std::mutex> lock(pool_->mutex);
    if (pool_->free_list.empty()) return nullptr;
    buffer = pool_->free_list.back();
    pool_->free_list.pop_back();
  }
  auto pool = pool_;
  std::shared_ptr<const void> owner(buffer, [pool](const void * p) {
    std::lock_guard<std::mutex> lock(pool->mutex);
    pool->free_list.push_back(static_cast<std::uint8_t *>(const_cast<void *>(p)));
  });
  // Y와 UV는 원본에서 연속 (UV가 height×stride 위치) → 한 번에 height×1.5 줄 복사
  check(cudaMemcpy2DAsync(buffer, std::size_t(width_), nv12, stride, std::size_t(width_), std::size_t(height_) * 3 / 2,
                          cudaMemcpyHostToDevice, pool_->stream), "cudaMemcpy2DAsync (shared NV12)");
  check(cudaStreamSynchronize(pool_->stream), "cudaStreamSynchronize (shared NV12)");
  auto frame = std::make_shared<bev_handoff::DeviceNv12Frame>();
  frame->device = buffer;
  frame->pitch = std::size_t(width_);
  frame->width = width_;
  frame->height = height_;
  frame->id = id;
  frame->captured_at = captured_at;
  frame->owner = std::move(owner);
  return frame;
}

}  // namespace camera_driver
