#include "bev_handoff/device_frame_handoff.hpp"
#include <array>
#include <mutex>

namespace bev_handoff
{
namespace
{
struct State
{
  std::mutex mutex;
  std::array<std::shared_ptr<const DeviceNv12Frame>, 2> recent;   // BEV가 BevInput을 받을 때까지 (보통 바로 다음)
  std::size_t next{0};
  DeviceNv12Callback consumer;
  std::uint64_t consumer_id{0}, next_consumer_id{1};
};
State & state()
{
  static State s;
  return s;
}
}  // namespace

void publishDeviceNv12Frame(std::shared_ptr<const DeviceNv12Frame> frame)
{
  if (!frame) {return;}
  auto & s = state();
  DeviceNv12Callback consumer;
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    s.recent[s.next] = frame;
    s.next = (s.next + 1) % s.recent.size();
    consumer = s.consumer;
  }
  if (consumer) {consumer(std::move(frame));}
}

std::shared_ptr<const DeviceNv12Frame> findDeviceNv12Frame(const std::uint64_t id)
{
  auto & s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  for (const auto & f : s.recent) {
    if (f && f->id == id) {return f;}
  }
  return nullptr;
}

std::uint64_t registerDeviceNv12Consumer(DeviceNv12Callback callback)
{
  auto & s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  s.consumer = std::move(callback);
  s.consumer_id = s.next_consumer_id++;
  return s.consumer_id;
}

void unregisterDeviceNv12Consumer(const std::uint64_t consumer_id)
{
  auto & s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  if (s.consumer_id == consumer_id) {s.consumer = {}; s.consumer_id = 0;}
}

bool hasDeviceNv12Consumer()
{
  auto & s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  return static_cast<bool>(s.consumer);
}
}  // namespace bev_handoff
