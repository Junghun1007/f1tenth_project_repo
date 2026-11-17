#include <cmath>
#include <cstdlib>
#include <iostream>

#include "auto_control/control_core.hpp"

namespace
{
bool expect_near(double actual, double expected, double tolerance, const char * message)
{
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
    std::cerr << "FAILED: " << message << ": " << actual << " != " << expected << '\n';
    return false;
  }
  return true;
}

bool expect_true(bool value, const char * message)
{
  if (!value) {std::cerr << "FAILED: " << message << '\n';}
  return value;
}
}  // namespace

int main()
{
  using auto_control::duty_feedforward;
  using auto_control::duty_only_coast_release;
  using auto_control::duty_only_stop_speed_limit;
  bool passed = true;

  passed &= expect_near(duty_feedforward(1.0, 0.01, 0.05, 0.10, 0.12), 0.06, 1e-12,
    "feedforward adds offset and slope above fade speed");
  passed &= expect_near(duty_feedforward(0.05, 0.01, 0.05, 0.10, 0.12), 0.0075, 1e-12,
    "offset fades near standstill");
  passed &= expect_near(duty_feedforward(5.0, 0.01, 0.05, 0.10, 0.12), 0.12, 1e-12,
    "feedforward respects duty cap");
  passed &= expect_near(duty_feedforward(-1.0, 0.01, 0.05, 0.10, 0.12), 0.0, 1e-12,
    "reverse speed gives no duty");

  // a=0.5, coast=0.4, release=0.4 -> coast length 0.2 m.
  passed &= expect_near(duty_only_stop_speed_limit(0.0, 0.5, 0.4, 0.4, 0.0), 0.0, 1e-12,
    "zero distance gives zero speed");
  passed &= expect_near(duty_only_stop_speed_limit(0.2, 0.5, 0.4, 0.4, 0.0), 0.4, 1e-12,
    "coast segment starts at release speed");
  passed &= expect_near(duty_only_stop_speed_limit(1.2, 0.5, 0.4, 0.4, 0.0),
    std::sqrt(0.16 + 1.0), 1e-12, "duty segment follows planning deceleration");
  passed &= expect_near(duty_only_stop_speed_limit(0.1, 0.5, 0.4, 0.4, 0.0),
    std::sqrt(0.08), 1e-12, "coast segment follows coast deceleration");
  {
    // With a response delay the duty segment is slower, and the coast segment
    // must not jump above the duty segment's end speed.
    const double end = duty_only_stop_speed_limit(0.2 + 1e-9, 0.5, 0.4, 0.4, 0.1);
    const double inside = duty_only_stop_speed_limit(0.19, 0.5, 0.4, 0.4, 0.1);
    passed &= expect_true(end < 0.4, "response delay lowers segment end");
    passed &= expect_true(inside <= end + 1e-9, "coast limit is continuous at boundary");
  }
  {
    double previous = 0.0;
    bool monotonic = true;
    for (int i = 1; i <= 300; ++i) {
      const double value = duty_only_stop_speed_limit(0.01 * i, 0.5, 0.4, 0.4, 0.1);
      monotonic &= value + 1e-12 >= previous;
      previous = value;
    }
    passed &= expect_true(monotonic, "limit increases with distance");
  }

  passed &= expect_true(duty_only_coast_release(0.2, 0.4, 0.4, 0.0, 0.45),
    "release exactly at coast distance");
  passed &= expect_true(!duty_only_coast_release(0.25, 0.4, 0.4, 0.0, 0.45),
    "no release before coast distance");
  passed &= expect_true(duty_only_coast_release(0.23, 0.4, 0.4, 0.1, 0.45),
    "release delay advances release");
  passed &= expect_true(!duty_only_coast_release(0.1, 1.0, 0.4, 0.0, 0.45),
    "no coast release above release speed cap");
  passed &= expect_true(duty_only_coast_release(-0.01, 1.0, 0.4, 0.0, 0.45),
    "passing the aim point always releases");
  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
