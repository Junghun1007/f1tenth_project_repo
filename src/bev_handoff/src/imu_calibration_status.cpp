#include "bev_handoff/imu_calibration_status.hpp"
#include <atomic>

namespace bev_handoff
{
namespace
{
std::atomic<int> & slot()
{
  static std::atomic<int> value{static_cast<int>(ImuCalibrationState::unknown)};
  return value;
}
}  // namespace

void publishImuCalibrationState(ImuCalibrationState state)
{
  slot().store(static_cast<int>(state), std::memory_order_release);
}

ImuCalibrationState imuCalibrationState()
{
  return static_cast<ImuCalibrationState>(slot().load(std::memory_order_acquire));
}
}  // namespace bev_handoff
