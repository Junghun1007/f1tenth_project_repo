#include "check.hpp"
#include "cone_drive_test/core/stabilization.hpp"
#include "cone_drive_test/copied/bev_geometry.hpp"

using namespace cone_drive_test;

static ImageStabilizationCorrection corr(double roll_deg, double pitch_deg) {
  ImageStabilizationCorrection c;
  c.camera_to_reference_tilt = rotationZ(degToRad(roll_deg)) * rotationX(degToRad(pitch_deg));   // 광학 z=시선(roll), x=가로(pitch)
  c.roll_error_deg = roll_deg;
  c.pitch_error_deg = pitch_deg;
  c.correction_angle_deg = std::hypot(roll_deg, pitch_deg);
  c.within_correction_limit = c.correction_angle_deg <= 3.0;
  return c;
}

TEST(full_frame_coverage_check_rejects_even_tiny_correction) {   // 버그 재현: 확대 없이 전체 화면 검사는 0.3°도 거부
  const StabilizationCamera cam{561.40, 561.14, 643.03, 392.62, 1280, 800};
  const auto H = makeImageStabilizationHomography(cam.fx, cam.fy, cam.cx, cam.cy, corr(0.3, 0.0));
  CHECK(!outputIsCoveredBySource(H, {1280, 800}, {1280, 800}, 2.0));
}

TEST(small_correction_is_applied) {
  const StabilizationCamera cam{561.40, 561.14, 643.03, 392.62, 1280, 800};
  const auto d = decide_stabilization(cam, corr(0.8, -0.5));
  CHECK(d.apply);
  CHECK(d.reason == StabilizationReason::Applied);
}

TEST(over_limit_correction_is_rejected) {
  const StabilizationCamera cam{561.40, 561.14, 643.03, 392.62, 1280, 800};
  const auto d = decide_stabilization(cam, corr(4.0, 0.0));
  CHECK(!d.apply);
  CHECK(d.reason == StabilizationReason::OverLimit);
}

TEST(stats_summary_counts_applied_ratio) {
  StabilizationStats s;
  for (int k = 0; k < 9; ++k) s.count(StabilizationReason::Applied, 0.5);
  s.count(StabilizationReason::NoImuMatch, 0.0);
  const auto t = s.summary();
  CHECK(t.find("90%") != std::string::npos);
}
