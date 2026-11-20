#include "check.hpp"
#include "auto_control/drive/core/rate_gate.hpp"

using namespace auto_control::drive;

static int accepted(double camera_fps, double target_fps, double seconds) {
  RateGate g(target_fps);
  int n = 0;
  for (int k = 0; k < int(camera_fps * seconds); ++k) n += g.accept(k / camera_fps);
  return n;
}

TEST(seventy_fps_camera_gives_forty_detections) {   // 단순 간격 비교면 70→35가 된다
  CHECK(std::abs(accepted(70.0, 40.0, 10.0) - 400) <= 2);
}

TEST(slower_camera_passes_every_frame) {
  CHECK(accepted(30.0, 40.0, 10.0) == 300);
}

TEST(rate_change_applies) {
  RateGate g(40.0);
  g.set_rate(20.0);
  int n = 0;
  for (int k = 0; k < 700; ++k) n += g.accept(k / 70.0);
  CHECK(std::abs(n - 200) <= 2);
}

TEST(long_pause_does_not_burst) {   // 처리가 멈췄다 돌아와도 밀린 만큼 몰아서 돌리지 않음
  RateGate g(40.0);
  CHECK(g.accept(0.0));
  CHECK(g.accept(5.0));
  CHECK(!g.accept(5.0 + 1.0 / 70.0));
}
