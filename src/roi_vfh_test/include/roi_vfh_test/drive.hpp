#pragma once
#include <auto_control/control_core.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace roi_vfh_test {
struct DriveOptions {
  double maximum_duty{.12},start_duty{.053},rise{.04},fall{.03};
  double kp{1},ki{.5},kd{0},integral_limit{1.5};
  double ff_offset{.01},ff_slope{.05},ff_fade_speed{.1};
  double brake_max{2},brake_speed_gain{.8},brake_hysteresis{.05};
  double brake_rise{1.5},brake_fall{6};
  double servo_left{1},servo_center{.5},servo_right{0},max_steer{.5235987755982988};
  bool inverted{true};
  void validate()const {
    for(double v:{maximum_duty,start_duty,rise,fall,kp,ki,kd,integral_limit,ff_offset,ff_slope,
      ff_fade_speed,brake_max,brake_speed_gain,brake_hysteresis,brake_rise,brake_fall,
      servo_left,servo_center,servo_right,max_steer})
      if(!std::isfinite(v) || v<0)throw std::invalid_argument("invalid VFH drive parameter");
    if(maximum_duty<=0 || maximum_duty>1 || start_duty>maximum_duty || rise<=0 || fall<=0 ||
      ff_fade_speed<=0 || brake_max<=0 || brake_max>8 || brake_speed_gain<=0 ||
      brake_hysteresis<=0 || brake_rise<=0 || brake_fall<=0 ||
      servo_left>1 || servo_center>1 || servo_right>1 || max_steer<=0 || max_steer>1.0472 ||
      (servo_left-servo_center)*(servo_right-servo_center)>=0)
      throw std::invalid_argument("VFH drive limits/servo calibration invalid");
  }
};
struct DriveCommand {
  double duty{0},brake{0},servo{.5},steer{0};
  bool brake_mode{true};
};
// Lightweight longitudinal adapter: reuse auto_control's normalized PID,
// feedforward calibration, slew helper, staged braking and exact servo mapping.
// It does not run auto_control's lane planner or its higher speed floor.
class DriveController {
  DriveOptions options_;
  auto_control::SpeedPid pid_;
  DriveCommand output_;
  bool braking_{true},departure_{true};
public:
  explicit DriveController(const DriveOptions& options):options_(options),
    pid_(options.kp,options.ki,options.kd,options.integral_limit,-1,1) {
    options_.validate();output_.servo=options_.servo_center;
  }
  DriveCommand update(bool active,double target,double speed,double steer,double dt) {
    if(!active || !std::isfinite(target) || !std::isfinite(speed) || !std::isfinite(steer) || target<=0) {
      pid_.reset();departure_=true;braking_=true;
      output_.duty=0;output_.brake=options_.brake_max;output_.brake_mode=true;
      return output_; // Retain last commanded steering on all stop paths.
    }
    dt=std::isfinite(dt)?std::clamp(dt,0.001,.1):.001;
    output_.steer=std::clamp(steer,-options_.max_steer,options_.max_steer);
    output_.servo=auto_control::steering_angle_to_servo(output_.steer,options_.max_steer,
      options_.servo_left,options_.servo_center,options_.servo_right,options_.inverted);
    const double overspeed=std::abs(speed)-target;
    const bool need_brake=overspeed>(braking_?options_.brake_hysteresis*.5:options_.brake_hysteresis);
    if(need_brake || braking_) {
      pid_.reset();output_.duty=0;output_.brake_mode=true;
      const double requested=need_brake?auto_control::staged_brake_current(overspeed,0,
        options_.brake_speed_gain,0,options_.brake_max):0;
      output_.brake=auto_control::move_toward(output_.brake,requested,
        (requested>output_.brake?options_.brake_rise:options_.brake_fall)*dt);
      if(output_.brake<=1e-6)output_.brake=0;
      braking_=need_brake || output_.brake>0;
      // Even when current becomes zero, publish ONLY brake=0 this cycle.
      // The VESC bridge replaces pending motor commands by mode.
      return output_;
    }
    output_.brake=0;output_.brake_mode=false;
    const double ff=std::clamp(options_.ff_offset*std::min(1.,target/options_.ff_fade_speed)+
      options_.ff_slope*target,0.,options_.maximum_duty);
    const double effort=pid_.update(target,speed,ff/options_.maximum_duty,dt);
    const double desired=std::clamp(effort*options_.maximum_duty,0.,options_.maximum_duty);
    if(departure_ && std::abs(speed)<.05 && desired>0) {
      output_.duty=std::min(options_.start_duty,desired);departure_=false;
    } else {
      output_.duty=auto_control::move_toward(output_.duty,desired,
        (desired>output_.duty?options_.rise:options_.fall)*dt);
      if(std::abs(speed)>=.05)departure_=false;
    }
    pid_.apply_output_limit(effort,output_.duty/options_.maximum_duty);
    return output_;
  }
};
}
