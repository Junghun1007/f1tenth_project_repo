#include "bev_handoff/ir_camera_settings.hpp"
#include <atomic>

namespace bev_handoff
{
namespace
{
std::shared_ptr<const TunnelRuntimeSettings> runtime_settings;

std::shared_ptr<const IrCameraSettings> & slot()
{
  static std::shared_ptr<const IrCameraSettings> settings;
  return settings;
}
std::atomic<std::uint64_t> & revision()
{
  static std::atomic<std::uint64_t> value{0};
  return value;
}
}  // namespace

void publishIrCameraSettings(std::shared_ptr<const IrCameraSettings> settings)
{
  std::atomic_store_explicit(&slot(), std::move(settings), std::memory_order_release);
  revision().fetch_add(1, std::memory_order_acq_rel);
}

std::shared_ptr<const IrCameraSettings> irCameraSettings()
{
  return std::atomic_load_explicit(&slot(), std::memory_order_acquire);
}

std::uint64_t irCameraSettingsRevision()
{
  return revision().load(std::memory_order_acquire);
}
void publishTunnelRuntimeSettings(std::shared_ptr<const TunnelRuntimeSettings> settings)
{
  std::atomic_store(&runtime_settings, std::move(settings));
}

std::shared_ptr<const TunnelRuntimeSettings> tunnelRuntimeSettings()
{
  return std::atomic_load(&runtime_settings);
}
}  // namespace bev_handoff
