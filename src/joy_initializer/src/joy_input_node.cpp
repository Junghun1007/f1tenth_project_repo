/*
 * Copyright (c) 2020, Open Source Robotics Foundation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <SDL.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joy_initializer
{

class JoyInputNode final : public rclcpp::Node
{
public:
  JoyInputNode()
  : rclcpp::Node("joy_input_node")
  {
    device_id_ = declare_parameter<int>("device_id", 0);
    device_name_ = declare_parameter<std::string>("device_name", "");
    deadzone_ = declare_parameter<double>("deadzone", 0.05);
    autorepeat_rate_hz_ = declare_parameter<double>("autorepeat_rate", 50.0);
    sticky_buttons_ = declare_parameter<bool>("sticky_buttons", false);
    poll_interval_ms_ = declare_parameter<int>("coalesce_interval_ms", 1);
    reconnect_interval_sec_ =
      declare_parameter<double>("reconnect_interval_sec", 1.0);

    if (device_id_ < 0) {
      throw std::invalid_argument("device_id must be non-negative");
    }
    if (!std::isfinite(deadzone_) || deadzone_ < 0.0 || deadzone_ > 0.90) {
      throw std::invalid_argument("deadzone must be between 0.0 and 0.90");
    }
    if (
      !std::isfinite(autorepeat_rate_hz_) || autorepeat_rate_hz_ < 0.0 ||
      autorepeat_rate_hz_ > 1000.0)
    {
      throw std::invalid_argument("autorepeat_rate must be between 0 and 1000 Hz");
    }
    if (poll_interval_ms_ < 1 || poll_interval_ms_ > 1000) {
      throw std::invalid_argument("coalesce_interval_ms must be between 1 and 1000");
    }
    if (
      !std::isfinite(reconnect_interval_sec_) ||
      reconnect_interval_sec_ <= 0.0)
    {
      throw std::invalid_argument("reconnect_interval_sec must be positive");
    }

    unscaled_deadzone_ = 32767.0 * deadzone_;
    scale_ = -1.0 / ((1.0 - deadzone_) * 32767.0);
    autorepeat_period_sec_ = autorepeat_rate_hz_ > 0.0 ?
      1.0 / autorepeat_rate_hz_ : 0.0;

    publisher_ = create_publisher<sensor_msgs::msg::Joy>(
      "joy", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

    // Deliberately initialize only SDL's input subsystem. This node contains
    // no haptic subsystem initialization, force-feedback subscription, or
    // rumble API call.
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
      throw std::runtime_error(
              "SDL joystick input initialization failed: " +
              std::string(SDL_GetError()));
    }
    sdl_initialized_ = true;
    SDL_JoystickEventState(SDL_ENABLE);

    last_publish_time_ = now();
    tryOpenConnectedDevice();
    if (joystick_ == nullptr) {
      if (device_name_.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Joystick device_id=%d is not present. Waiting for an SDL device-add event.",
          device_id_);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Joystick '%s' is not present. Waiting for that exact device name.",
          device_name_.c_str());
      }
    }

    poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_interval_ms_),
      std::bind(&JoyInputNode::pollEvents, this));
  }

  ~JoyInputNode() override
  {
    if (poll_timer_) {
      poll_timer_->cancel();
    }
    closeJoystick();
    if (sdl_initialized_) {
      SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
      sdl_initialized_ = false;
    }
  }

private:
  bool deviceMatches(const int device_index) const
  {
    if (device_index < 0) {
      return false;
    }
    if (device_name_.empty()) {
      return device_index == device_id_;
    }
    const char * name = SDL_JoystickNameForIndex(device_index);
    return name != nullptr && device_name_ == name;
  }

  void tryOpenConnectedDevice()
  {
    last_open_attempt_time_ = now();
    if (joystick_ != nullptr) {
      return;
    }
    const int count = SDL_NumJoysticks();
    if (count < 0) {
      RCLCPP_WARN(get_logger(), "SDL could not enumerate joysticks: %s", SDL_GetError());
      return;
    }
    for (int index = 0; index < count; ++index) {
      if (deviceMatches(index)) {
        openJoystick(index);
        return;
      }
    }
  }

  void openJoystick(const int device_index)
  {
    if (joystick_ != nullptr || !deviceMatches(device_index)) {
      return;
    }
    last_open_attempt_time_ = now();

    SDL_Joystick * opened = SDL_JoystickOpen(device_index);
    if (opened == nullptr) {
      RCLCPP_ERROR(
        get_logger(), "Unable to open joystick index %d: %s",
        device_index, SDL_GetError());
      return;
    }

    const SDL_JoystickID instance_id = SDL_JoystickInstanceID(opened);
    const int axis_count = SDL_JoystickNumAxes(opened);
    const int button_count = SDL_JoystickNumButtons(opened);
    const int hat_count = SDL_JoystickNumHats(opened);
    if (instance_id < 0 || axis_count < 0 || button_count < 0 || hat_count < 0) {
      RCLCPP_ERROR(
        get_logger(), "Opened joystick has invalid capabilities: %s", SDL_GetError());
      SDL_JoystickClose(opened);
      return;
    }

    joystick_ = opened;
    joystick_instance_id_ = instance_id;
    axis_count_ = axis_count;
    hat_count_ = hat_count;
    joy_message_.axes.assign(
      static_cast<std::size_t>(axis_count_ + 2 * hat_count_), 0.0F);
    joy_message_.buttons.assign(static_cast<std::size_t>(button_count), 0);
    joy_message_.header.frame_id = "joy";

    for (int axis = 0; axis < axis_count_; ++axis) {
      std::int16_t value = 0;
      if (SDL_JoystickGetAxisInitialState(joystick_, axis, &value) == SDL_TRUE) {
        joy_message_.axes.at(static_cast<std::size_t>(axis)) = convertAxis(value);
      }
    }
    for (int button = 0; button < button_count; ++button) {
      joy_message_.buttons.at(static_cast<std::size_t>(button)) =
        SDL_JoystickGetButton(joystick_, button) != 0U ? 1 : 0;
    }
    for (int hat = 0; hat < hat_count_; ++hat) {
      updateHat(hat, SDL_JoystickGetHat(joystick_, hat));
    }

    const char * opened_name = SDL_JoystickName(joystick_);
    if (has_opened_before_) {
      RCLCPP_WARN(
        get_logger(),
        "Reopened joystick after a real SDL disconnect: %s "
        "(instance_id=%d, deadzone=%.3f)",
        opened_name == nullptr ? "unknown" : opened_name,
        static_cast<int>(joystick_instance_id_), deadzone_);
    } else {
      RCLCPP_INFO(
        get_logger(), "Opened joystick input: %s "
        "(initial connection, instance_id=%d, deadzone=%.3f)",
        opened_name == nullptr ? "unknown" : opened_name,
        static_cast<int>(joystick_instance_id_), deadzone_);
    }
    has_opened_before_ = true;
    publishCurrentState();
  }

  void closeJoystick()
  {
    if (joystick_ != nullptr) {
      SDL_JoystickClose(joystick_);
      joystick_ = nullptr;
    }
    joystick_instance_id_ = -1;
    axis_count_ = 0;
    hat_count_ = 0;
    joy_message_.axes.clear();
    joy_message_.buttons.clear();
  }

  void handleRemoval(const SDL_JoyDeviceEvent & event)
  {
    if (joystick_ == nullptr || event.which != joystick_instance_id_) {
      return;
    }
    RCLCPP_ERROR(
      get_logger(),
      "Joystick disconnected by SDL (instance_id=%d). Input publication is "
      "stopped until the same controller reconnects.",
      static_cast<int>(joystick_instance_id_));
    closeJoystick();
  }

  float convertAxis(std::int16_t value) const
  {
    if (value == -32768) {
      value = -32767;
    }
    double adjusted = static_cast<double>(value);
    if (adjusted > unscaled_deadzone_) {
      adjusted -= unscaled_deadzone_;
    } else if (adjusted < -unscaled_deadzone_) {
      adjusted += unscaled_deadzone_;
    } else {
      adjusted = 0.0;
    }
    return static_cast<float>(adjusted * scale_);
  }

  bool handleAxis(const SDL_JoyAxisEvent & event)
  {
    if (
      joystick_ == nullptr || event.which != joystick_instance_id_ ||
      event.axis >= joy_message_.axes.size())
    {
      return false;
    }
    const auto index = static_cast<std::size_t>(event.axis);
    const float converted = convertAxis(event.value);
    if (joy_message_.axes.at(index) == converted) {
      return false;
    }
    joy_message_.axes.at(index) = converted;
    return true;
  }

  bool handleButton(const SDL_JoyButtonEvent & event, const bool pressed)
  {
    if (
      joystick_ == nullptr || event.which != joystick_instance_id_ ||
      event.button >= joy_message_.buttons.size())
    {
      return false;
    }
    const auto index = static_cast<std::size_t>(event.button);
    if (sticky_buttons_) {
      if (pressed) {
        joy_message_.buttons.at(index) = 1 - joy_message_.buttons.at(index);
        return true;
      }
      return false;
    }
    const int value = pressed ? 1 : 0;
    if (joy_message_.buttons.at(index) == value) {
      return false;
    }
    joy_message_.buttons.at(index) = value;
    return true;
  }

  void updateHat(const int hat_index, const std::uint8_t value)
  {
    if (hat_index < 0 || hat_index >= hat_count_) {
      return;
    }
    const auto first = static_cast<std::size_t>(axis_count_ + 2 * hat_index);
    joy_message_.axes.at(first) =
      (value & SDL_HAT_LEFT) != 0U ? 1.0F :
      ((value & SDL_HAT_RIGHT) != 0U ? -1.0F : 0.0F);
    joy_message_.axes.at(first + 1U) =
      (value & SDL_HAT_UP) != 0U ? 1.0F :
      ((value & SDL_HAT_DOWN) != 0U ? -1.0F : 0.0F);
  }

  bool handleHat(const SDL_JoyHatEvent & event)
  {
    if (
      joystick_ == nullptr || event.which != joystick_instance_id_ ||
      event.hat >= static_cast<std::uint8_t>(hat_count_))
    {
      return false;
    }
    updateHat(event.hat, event.value);
    return true;
  }

  void publishCurrentState()
  {
    if (joystick_ == nullptr) {
      return;
    }
    joy_message_.header.stamp = now();
    publisher_->publish(joy_message_);
    last_publish_time_ = joy_message_.header.stamp;
  }

  void pollEvents()
  {
    bool state_changed = false;
    SDL_Event event;
    int processed_events = 0;
    constexpr int kMaximumEventsPerPoll = 256;
    while (
      processed_events < kMaximumEventsPerPoll &&
      SDL_PollEvent(&event) == 1)
    {
      ++processed_events;
      switch (event.type) {
        case SDL_JOYDEVICEADDED:
          openJoystick(event.jdevice.which);
          break;
        case SDL_JOYDEVICEREMOVED:
          handleRemoval(event.jdevice);
          break;
        case SDL_JOYAXISMOTION:
          state_changed = handleAxis(event.jaxis) || state_changed;
          break;
        case SDL_JOYBUTTONDOWN:
          state_changed = handleButton(event.jbutton, true) || state_changed;
          break;
        case SDL_JOYBUTTONUP:
          state_changed = handleButton(event.jbutton, false) || state_changed;
          break;
        case SDL_JOYHATMOTION:
          state_changed = handleHat(event.jhat) || state_changed;
          break;
        default:
          break;
      }
    }

    if (
      joystick_ != nullptr &&
      SDL_JoystickGetAttached(joystick_) != SDL_TRUE)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joystick no longer attached according to SDL (instance_id=%d).",
        static_cast<int>(joystick_instance_id_));
      closeJoystick();
      return;
    }

    if (joystick_ == nullptr) {
      if (
        (now() - last_open_attempt_time_).seconds() >=
        reconnect_interval_sec_)
      {
        tryOpenConnectedDevice();
      }
      return;
    }
    const double elapsed_sec = (now() - last_publish_time_).seconds();
    const bool autorepeat_due =
      autorepeat_period_sec_ > 0.0 && elapsed_sec >= autorepeat_period_sec_;
    if (state_changed || autorepeat_due) {
      publishCurrentState();
    }
  }

  int device_id_{0};
  std::string device_name_;
  double deadzone_{0.05};
  double unscaled_deadzone_{0.0};
  double scale_{0.0};
  double autorepeat_rate_hz_{50.0};
  double autorepeat_period_sec_{0.02};
  bool sticky_buttons_{false};
  int poll_interval_ms_{1};
  double reconnect_interval_sec_{1.0};

  bool sdl_initialized_{false};
  bool has_opened_before_{false};
  SDL_Joystick * joystick_{nullptr};
  SDL_JoystickID joystick_instance_id_{-1};
  int axis_count_{0};
  int hat_count_{0};

  sensor_msgs::msg::Joy joy_message_;
  rclcpp::Time last_publish_time_;
  rclcpp::Time last_open_attempt_time_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace joy_initializer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joy_initializer::JoyInputNode>());
  rclcpp::shutdown();
  return 0;
}
