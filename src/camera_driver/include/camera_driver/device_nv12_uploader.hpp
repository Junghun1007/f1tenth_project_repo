// 카메라 원본 NV12를 GPU 버퍼 풀에 한 번 올린다 (bev_handoff::DeviceNv12Frame으로 BEV·YOLO가 함께 씀).
// 스트림 동기화까지 끝낸 뒤 돌려주므로 받는 쪽은 다른 스트림에서 바로 읽어도 된다.
// 받는 쪽이 버퍼를 다 잡고 있으면(풀 소진) nullptr → 부른 쪽은 기존 호스트 복사 경로로 처리
#pragma once
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include "bev_handoff/device_frame_handoff.hpp"

namespace camera_driver {

class DeviceNv12Uploader {
public:
  DeviceNv12Uploader(int width, int height, int buffer_count = 4);
  ~DeviceNv12Uploader();
  DeviceNv12Uploader(const DeviceNv12Uploader &) = delete;
  DeviceNv12Uploader & operator=(const DeviceNv12Uploader &) = delete;
  std::shared_ptr<const bev_handoff::DeviceNv12Frame> upload(
    const std::uint8_t * nv12, std::size_t stride, std::uint64_t id, std::chrono::steady_clock::time_point captured_at);
  int width() const { return width_; }
  int height() const { return height_; }

private:
  struct Pool;
  int width_, height_;
  std::shared_ptr<Pool> pool_;
};

}  // namespace camera_driver
