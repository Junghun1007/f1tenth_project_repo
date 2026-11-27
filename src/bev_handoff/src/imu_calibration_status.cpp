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
std::shared_ptr<const ImuCalibrationProgress> & progress_slot()
{
  static std::shared_ptr<const ImuCalibrationProgress> value;
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

void publishImuCalibrationProgress(std::shared_ptr<const ImuCalibrationProgress> progress)
{
  std::atomic_store_explicit(&progress_slot(), std::move(progress), std::memory_order_release);
}

std::shared_ptr<const ImuCalibrationProgress> imuCalibrationProgress()
{
  return std::atomic_load_explicit(&progress_slot(), std::memory_order_acquire);
}
}  // namespace bev_handoff
