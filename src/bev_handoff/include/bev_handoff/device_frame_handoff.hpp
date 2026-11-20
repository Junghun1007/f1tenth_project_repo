#pragma once
// 카메라 원본 NV12 한 장을 GPU에 한 번만 올려 같은 프로세스의 BEV(bev_processor)와 YOLO(auto_control)가 함께 쓴다.
// camera_driver가 올리고 스트림 동기화까지 끝낸 뒤 발행하므로, 받는 쪽은 다른 CUDA 스트림에서 바로 읽어도 된다.
// owner를 잡고 있는 동안 버퍼는 재사용되지 않는다 (받는 쪽은 자기 스트림 작업이 끝날 때까지 owner를 유지).
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace bev_handoff
{
struct DeviceNv12Frame
{
  const std::uint8_t * device{nullptr};   // Y 평면 시작, 그 뒤 height×pitch 위치에 UV 평면
  std::size_t pitch{0};                   // 한 줄 바이트 (Y·UV 같음)
  int width{0}, height{0};                // 원본 전체 크기
  std::uint64_t id{0};                    // camera_driver 프레임 번호 (BevInput.device_frame_id와 같음, 0 = 없음)
  std::chrono::steady_clock::time_point captured_at;
  std::shared_ptr<const void> owner;
};
using DeviceNv12Callback = std::function<void(std::shared_ptr<const DeviceNv12Frame>)>;

// 발행: 최근 몇 장을 id로 찾을 수 있게 남기고(BEV), 등록된 소비자(YOLO)에게 넘긴다 (소비자는 최신 것만 바꿔 끼울 것)
void publishDeviceNv12Frame(std::shared_ptr<const DeviceNv12Frame> frame);
std::shared_ptr<const DeviceNv12Frame> findDeviceNv12Frame(std::uint64_t id);   // 없으면 nullptr
std::uint64_t registerDeviceNv12Consumer(DeviceNv12Callback callback);
void unregisterDeviceNv12Consumer(std::uint64_t consumer_id);
bool hasDeviceNv12Consumer();
}  // namespace bev_handoff
