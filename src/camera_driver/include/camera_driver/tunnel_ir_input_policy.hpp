#pragma once
#include <algorithm>
#include <chrono>
#include <cstdlib>

namespace camera_driver
{
enum class TunnelIrRejection {none, time, exposure};

inline TunnelIrRejection tunnelIrRejection(
  bool validation, bool tunnel_profile,
  std::chrono::steady_clock::time_point captured,
  std::chrono::steady_clock::time_point last,
  std::chrono::steady_clock::time_point now,
  std::chrono::steady_clock::time_point changed_at,
  long exposure_us, int iso, long expected_exposure_us = 30000, int expected_iso = 800)
{
  // Like the standalone capture loop: consume each new frame once. In
  // validation mode the command is applied but its metadata is diagnostic.
  if (captured <= last) {return TunnelIrRejection::time;}
  if (validation) {return TunnelIrRejection::none;}
  using namespace std::chrono_literals;
  if (captured > now + 50ms || now - captured > 250ms || captured < changed_at + 150ms) {
    return TunnelIrRejection::time;
  }
  // 장치가 노출을 라인 단위로 맞추므로 ±500 µs·ISO ±5%까지 같은 설정으로 본다
  if (tunnel_profile && (std::labs(exposure_us - expected_exposure_us) > 500 ||
    std::abs(iso - expected_iso) > std::max(1, expected_iso / 20)))
  {
    return TunnelIrRejection::exposure;
  }
  return TunnelIrRejection::none;
}
}
