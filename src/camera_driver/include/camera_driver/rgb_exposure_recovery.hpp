#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

namespace camera_driver
{
enum class RgbTunnelPhase {outside, entering, inside, exiting};

// One worker owns normal RGB exposure and the temporary tunnel exit restore.
class RgbExposureRecovery
{
public:
  using Clock = std::chrono::steady_clock;
  struct Sample {Clock::time_point captured; long exposure_us; int iso;};
  struct Exposure {long exposure_us; int iso;};
  enum class Command {none, manual, restore, automatic};
  struct Action {Command command{Command::none}; long exposure_us{0}; int iso{0};};

  explicit RgbExposureRecovery(double max_hold_sec = .20, bool recover_on_exit = true)
  : max_hold_(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(
      checkedHold(max_hold_sec)))), startup_finished_(!recover_on_exit) {}

  Action step(RgbTunnelPhase phase, Clock::time_point now, std::optional<Sample> sample,
    std::optional<Exposure> manual = std::nullopt)
  {
    using namespace std::chrono_literals;
    const bool changed = phase != phase_;
    phase_ = phase;
    const bool new_sample = sample && (!latest_ || sample->captured > latest_->captured) &&
      sample->captured <= now && now - sample->captured <= 250ms &&
      sample->exposure_us > 0 && sample->exposure_us <= 1000000 && sample->iso > 0 && sample->iso <= 65535;
    if (new_sample) {latest_ = sample;}
    if (!startup_finished_) {measureStartup(now, new_sample ? sample : std::nullopt);}

    if (manual && startup_finished_) {
      if (applied_ && applied_->command == Command::manual &&
        applied_->exposure_us == manual->exposure_us && applied_->iso == manual->iso)
      {
        return {};
      }
      return {Command::manual, manual->exposure_us, manual->iso};
    }
    if (!applied_ || applied_->command == Command::manual) {return {Command::automatic};}
    if (applied_->command == Command::restore) {
      if (phase != RgbTunnelPhase::exiting || now - restore_at_ >= max_hold_) {return {Command::automatic};}
      if (new_sample && sample->captured > restore_at_) {
        const bool matches = std::abs(sample->exposure_us - applied_->exposure_us) <=
          std::max(100., applied_->exposure_us * .05) && sample->iso == applied_->iso;
        matched_ = matches ? matched_ + 1 : 0;
      }
      return matched_ >= 2 ? Action{Command::automatic} : Action{};
    }
    if (changed && phase == RgbTunnelPhase::exiting && saved_ && latest_ &&
      now - latest_->captured <= 250ms &&
      (latest_->exposure_us != saved_->exposure_us || latest_->iso != saved_->iso))
    {
      matched_ = 0;
      restore_at_ = now;
      return {Command::restore, saved_->exposure_us, saved_->iso};
    }
    return {};
  }

  void commandSent(Action action) {applied_ = action;}
  void commandFailed()
  {
    // Retry the current steady mode. A failed exit pulse must not be repeated.
    applied_.reset();
  }
  bool startupFinished() const {return startup_finished_;}
  std::optional<Sample> saved() const {return saved_;}

private:
  static double checkedHold(double value)
  {
    if (!std::isfinite(value) || value <= 0. || value > .5) {
      throw std::invalid_argument("RGB exposure recovery hold must be (0,0.5]s");
    }
    return value;
  }

  void measureStartup(Clock::time_point now, std::optional<Sample> sample)
  {
    using namespace std::chrono_literals;
    if (!startup_at_) {startup_at_ = now;}
    if (phase_ != RgbTunnelPhase::outside || now - *startup_at_ >= 2s) {
      startup_finished_ = true;
      history_.clear();
      return;
    }
    if (!sample || sample->captured < *startup_at_ + 200ms) {return;}
    history_.push_back(*sample);
    while (sample->captured - history_.front().captured > 250ms) {history_.pop_front();}
    if (history_.size() < 3 || sample->captured - history_.front().captured < 150ms) {return;}

    // Find the first stable AE window and preserve one actual exposure/ISO pair.
    auto first = history_.end();
    long low_us = sample->exposure_us, high_us = sample->exposure_us;
    int low_iso = sample->iso, high_iso = sample->iso;
    while (first != history_.begin()) {
      --first;
      low_us = std::min(low_us, first->exposure_us); high_us = std::max(high_us, first->exposure_us);
      low_iso = std::min(low_iso, first->iso); high_iso = std::max(high_iso, first->iso);
      if (high_us > low_us * 1.25 || high_iso > low_iso * 1.25) {return;}
      if (sample->captured - first->captured < 150ms || history_.end() - first < 3) {continue;}
      std::vector<Sample> window(first, history_.end());
      std::sort(window.begin(), window.end(), [](const Sample & a, const Sample & b) {
        return static_cast<double>(a.exposure_us) * a.iso < static_cast<double>(b.exposure_us) * b.iso;
      });
      saved_ = window[window.size() / 2];
      startup_finished_ = true;
      history_.clear();
      return;
    }
  }

  RgbTunnelPhase phase_{RgbTunnelPhase::outside};
  Clock::duration max_hold_;
  Clock::time_point restore_at_{};
  std::optional<Clock::time_point> startup_at_;
  std::deque<Sample> history_;
  std::optional<Sample> latest_, saved_;
  std::optional<Action> applied_{Action{Command::automatic}};
  bool startup_finished_;
  unsigned matched_{0};
};
}  // namespace camera_driver
