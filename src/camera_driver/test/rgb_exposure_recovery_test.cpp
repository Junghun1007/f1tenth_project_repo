#include "camera_driver/rgb_exposure_recovery.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

using Policy = camera_driver::RgbExposureRecovery;
using Phase = camera_driver::RgbTunnelPhase;
using Command = Policy::Command;
using Time = Policy::Clock::time_point;

void require(bool value, const char * why) {if (!value) {throw std::runtime_error(why);}}
Time at(int milliseconds) {return Time{} + std::chrono::milliseconds(milliseconds);}
Policy::Sample sample(int milliseconds, long exposure = 2000, int iso = 200)
{return {at(milliseconds), exposure, iso};}

Policy::Action send(Policy & p, Phase phase, int ms, std::optional<Policy::Sample> observation = {},
  std::optional<Policy::Exposure> manual = {})
{
  const auto action = p.step(phase, at(ms), observation, manual);
  if (action.command != Command::none) {p.commandSent(action);}
  return action;
}

void measure(Policy & p)
{
  for (int ms = 1000; ms < 1200; ms += 20) {
    require(send(p, Phase::outside, ms + 5, sample(ms, 9000, 800)).command == Command::none,
      "AE remains active during startup");
  }
  require(!p.saved(), "initial settling frames do not establish a baseline");
  for (int ms = 1200; ms <= 1380; ms += 20) {
    send(p, Phase::outside, ms + 5, sample(ms));
  }
  require(p.startupFinished() && p.saved() && p.saved()->exposure_us == 2000 && p.saved()->iso == 200,
    "capture one stable startup AE pair");
}

void enter(Policy & p)
{
  for (int ms = 1400; ms < 2000; ms += 20) {
    send(p, Phase::outside, ms + 5, sample(ms, 5000, 600));
  }
  send(p, Phase::entering, 2000, sample(1990, 12000, 800));
  send(p, Phase::inside, 2200, sample(2180, 12000, 800));
}

void test_exit_restore()
{
  Policy p;
  measure(p);
  enter(p);
  const auto action = send(p, Phase::exiting, 3000, sample(2990, 1000, 800));
  require(action.command == Command::restore && action.exposure_us == 2000 && action.iso == 200,
    "exit applies the exact startup pair, including when only one present value is higher");
  require(send(p, Phase::exiting, 3002, sample(2990)).command == Command::none,
    "old capture cannot confirm application");
  require(send(p, Phase::exiting, 3020, sample(3010)).command == Command::none,
    "first matching frame retains the pulse");
  require(send(p, Phase::exiting, 3022, sample(3010)).command == Command::none,
    "duplicate cannot count twice");
  require(send(p, Phase::exiting, 3040, sample(3030)).command == Command::automatic,
    "two fresh matches restore AE");
  require(send(p, Phase::exiting, 3050, sample(3040, 3000, 300)).command == Command::none,
    "same EXITING does not repeat the restore");
  for (int ms = 4000; ms < 5000; ms += 20) {
    send(p, Phase::outside, ms + 5, sample(ms, 1000, 100));
  }
  send(p, Phase::inside, 5000);
  const auto again = send(p, Phase::exiting, 6000, sample(5990, 9000, 800));
  require(again.command == Command::restore && again.exposure_us == 2000 && again.iso == 200,
    "later visits reuse the startup measurement");
}

void test_restore_returns_to_auto()
{
  for (const auto phase : {Phase::outside, Phase::entering, Phase::exiting}) {
    Policy p;
    measure(p);
    enter(p);
    send(p, Phase::exiting, 3000, sample(2990, 12000, 800));
    if (phase == Phase::exiting) {
      require(send(p, phase, 3199).command == Command::none, "hold ends at configured timeout");
    }
    require(send(p, phase, phase == Phase::exiting ? 3200 : 3001).command == Command::automatic,
      "restore ends on timeout or a new phase even with no frames");
  }

  Policy failed;
  measure(failed);
  enter(failed);
  require(failed.step(Phase::exiting, at(3000), sample(2990, 12000, 800)).command == Command::restore,
    "a restore command is attempted once");
  failed.commandFailed();
  require(failed.step(Phase::exiting, at(3002), {}).command == Command::automatic,
    "uncertain restore falls back to AE");
  failed.commandFailed();
  require(send(failed, Phase::exiting, 3003).command == Command::automatic, "failed AE is retried");
  require(send(failed, Phase::exiting, 3004).command == Command::none, "successful AE completes recovery");

  Policy changing;
  measure(changing);
  enter(changing);
  send(changing, Phase::exiting, 3000, sample(2990, 12000, 800));
  send(changing, Phase::exiting, 3010, sample(3005));
  send(changing, Phase::exiting, 3020, sample(3015, 9000, 800));
  require(send(changing, Phase::exiting, 3030, sample(3025)).command == Command::none,
    "matching confirmation must be consecutive");
  require(send(changing, Phase::exiting, 3040, sample(3035)).command == Command::automatic,
    "valid consecutive confirmation returns to AE");
}

void test_missing_reference()
{
  Policy p;
  for (int ms = 1000; ms < 3000; ms += 20) {
    send(p, Phase::outside, ms, sample(ms, (ms / 20) % 2 ? 2000 : 8000));
  }
  send(p, Phase::outside, 3000);
  require(p.startupFinished() && !p.saved(), "unstable startup times out without a baseline");
  send(p, Phase::inside, 3500, sample(3490, 12000, 800));
  require(send(p, Phase::exiting, 4000, sample(3990, 12000, 800)).command == Command::none,
    "missing reference keeps auto enabled at exit");

  Policy early_entry;
  send(early_entry, Phase::outside, 1000, sample(1000));
  send(early_entry, Phase::inside, 1100, sample(1090, 12000, 800));
  require(early_entry.startupFinished() && !early_entry.saved(), "entry closes incomplete startup measurement");

  Policy stale;
  measure(stale);
  enter(stale);
  require(send(stale, Phase::exiting, 3000).command == Command::none, "stale frame does not trigger a restore");

  Policy matching;
  measure(matching);
  enter(matching);
  require(send(matching, Phase::exiting, 3000, sample(2990)).command == Command::none,
    "matching exposure skips the restore");
}

void test_manual_mode()
{
  const Policy::Exposure fixed{8000, 400};
  Policy p;
  require(send(p, Phase::outside, 1000, sample(1000), fixed).command == Command::none,
    "saved manual mode lets startup measure AE first");
  for (int ms = 1200; ms < 1360; ms += 20) {
    require(send(p, Phase::outside, ms + 5, sample(ms), fixed).command == Command::none,
      "fixed ISO waits for the reference");
  }
  const auto manual = send(p, Phase::outside, 1365, sample(1360), fixed);
  require(p.saved() && manual.command == Command::manual && manual.exposure_us == 8000 && manual.iso == 400,
    "fixed exposure applies when AE measurement finishes");
  require(send(p, Phase::inside, 2000, {}, fixed).command == Command::none, "manual stays fixed inside");
  require(send(p, Phase::exiting, 3000, sample(2990, 12000, 800), fixed).command == Command::none,
    "exit does not override manual");
  const auto changed = send(p, Phase::exiting, 3005, {}, Policy::Exposure{8000, 600});
  require(changed.command == Command::manual && changed.iso == 600, "changed manual ISO is sent live");
  require(send(p, Phase::exiting, 3010).command == Command::automatic, "selecting AE applies live");
  require(send(p, Phase::exiting, 3015).command == Command::none, "AE does not reopen the same exit");

  Policy retry(.20, false);
  measure(retry);
  require(retry.step(Phase::outside, at(1500), {}, fixed).command == Command::manual,
    "manual works after startup measurement when exit recovery is disabled");
  retry.commandFailed();
  require(send(retry, Phase::outside, 1501, {}, fixed).command == Command::manual,
    "failed manual requests are retried while still selected");
  require(send(retry, Phase::outside, 1510).command == Command::automatic, "manual returns to auto");
  require(send(retry, Phase::exiting, 2000, sample(1990, 12000, 800)).command == Command::none,
    "the startup measurement is available with exit recovery disabled");

  Policy timeout;
  send(timeout, Phase::outside, 1000, {}, fixed);
  require(send(timeout, Phase::outside, 3000, {}, fixed).command == Command::manual,
    "missing startup frames do not postpone manual beyond the measurement timeout");

  Policy takeover;
  measure(takeover);
  enter(takeover);
  send(takeover, Phase::exiting, 3000, sample(2990, 9000, 800));
  require(send(takeover, Phase::exiting, 3001, {}, fixed).command == Command::manual,
    "operator-selected manual mode replaces an ongoing restore");
  require(send(takeover, Phase::exiting, 3200, {}, fixed).command == Command::none,
    "cancelled restore timeout does not clear fixed ISO");
}

void test_automatic_shutter_fixed_iso()
{
  Policy p;
  measure(p);
  const Policy::Exposure fixed{0, 400};
  const auto seed = send(p, Phase::outside, 1500, sample(1490), fixed);
  require(seed.command == Command::fixed_iso && seed.iso == 400 && seed.exposure_us == 1000,
    "fixed ISO uses camera metadata to seed the shutter");
  auto early = Policy::Sample{at(1550), 1000, 400, .10};
  require(send(p, Phase::outside, 1555, early, fixed).command == Command::none,
    "wait for shutter feedback after the sensor settles");
  auto dark = Policy::Sample{at(1630), 1000, 400, .10};
  const auto longer = send(p, Phase::outside, 1640, dark, fixed);
  require(longer.command == Command::fixed_iso && longer.exposure_us > 1000 && longer.iso == 400,
    "increase shutter time in darkness with ISO fixed");
  require(send(p, Phase::outside, 1770, dark, fixed).command == Command::none,
    "duplicate samples do not drive feedback");
  auto bright = Policy::Sample{at(1780), longer.exposure_us, 400, .80};
  const auto shorter = send(p, Phase::exiting, 1790, bright, fixed);
  require(shorter.command == Command::fixed_iso && shorter.exposure_us < longer.exposure_us && shorter.iso == 400,
    "reduce shutter time and retain fixed ISO at tunnel exit");
  require(p.saved()->iso == 200, "fixed ISO never overwrites the startup measurement");
  require(send(p, Phase::exiting, 1800).command == Command::automatic, "switch back to sensor auto");

  std::vector<std::uint8_t> plane(32 * 16, 235);
  for (int row = 0; row < 16; ++row) {
    std::fill_n(plane.begin() + row * 32, 24, 16);
  }
  require(camera_driver::rgbMeteredLuma(plane.data(), plane.size(), 24, 16, 32) == 0.,
    "luma metering ignores row padding");
  require(!camera_driver::rgbMeteredLuma(plane.data(), 24, 24, 16, 32),
    "luma metering rejects truncated frames");
}

int main()
{
  test_exit_restore();
  test_restore_returns_to_auto();
  test_missing_reference();
  test_manual_mode();
  test_automatic_shutter_fixed_iso();
  for (double value : {0., -.1, .51, std::numeric_limits<double>::quiet_NaN()}) {
    bool invalid = false;
    try {Policy invalid_policy(value);} catch (const std::invalid_argument &) {invalid = true;}
    require(invalid, "invalid hold must be rejected");
  }
  std::cout << "rgb_exposure_recovery_test passed\n";
}
