#include "bev_handoff/lane_control_hints.hpp"
#include <atomic>

namespace bev_handoff
{
namespace
{
std::shared_ptr<const LaneControlHints> & slot()
{
  static std::shared_ptr<const LaneControlHints> hints;
  return hints;
}
}  // namespace

void publishLaneControlHints(std::shared_ptr<const LaneControlHints> hints)
{
  std::atomic_store_explicit(&slot(), std::move(hints), std::memory_order_release);
}

std::shared_ptr<const LaneControlHints> laneControlHints()
{
  return std::atomic_load_explicit(&slot(), std::memory_order_acquire);
}
}  // namespace bev_handoff
