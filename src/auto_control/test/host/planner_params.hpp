// 테스트 전용: 골든 데이터의 params → PlannerParams / ControlParams
#pragma once
#include "fixture.hpp"
#include "auto_control/drive/core/frenet_planner.hpp"

inline auto_control::drive::PlannerParams planner_params_from(const nlohmann::json & P) {
  auto_control::drive::PlannerParams p;
  const auto & q = P["proposed"];
  p.s_max = q["s_max"]; p.ds = q["ds"]; p.l_max = q["l_max"]; p.dl = q["dl"];
  p.ds_lane = q["ds_lane"]; p.l_max_lane = q["l_max_lane"]; p.dl_lane = q["dl_lane"]; p.lane_half = q["lane_half"];
  p.hard_clearance = q["hard_clearance"]; p.soft_clearance = q["soft_clearance"];
  p.relaxed_hard_clearance = q["relaxed_hard_clearance"]; p.relaxed_soft_clearance = q["relaxed_soft_clearance"];
  p.free_hard_clearance = q["free_hard_clearance"]; p.free_soft_clearance = q["free_soft_clearance"];
  p.w_obstacle = q["w_obstacle"]; p.w_lane = q["w_lane"]; p.w_lane_linear = q["w_lane_linear"];
  p.lane_quad_cap = q["lane_quad_cap"]; p.w_lane_linear_free = q["w_lane_linear_free"];
  p.lane_tolerance = q["lane_tolerance"]; p.lane_return_distance = q["lane_return_distance"];
  p.w_offset = q["w_offset"]; p.w_slope = q["w_slope"]; p.w_curvature = q["w_curvature"]; p.max_curvature = q["max_curvature"];
  p.rollout_sec = q["rollout_sec"]; p.rollout_retry_margin = q["rollout_retry_margin"]; p.backward_extension = q["backward_extension"];
  const auto & v = P["vehicle"];
  p.wheelbase = v["wheelbase"]; p.length = v["length"]; p.width = v["width"]; p.rear_overhang = v["rear_overhang"];
  p.max_steer_rad = v["max_steer_deg"].get<double>() * M_PI / 180.0; p.steer_rate_rad_s = v["steer_rate_deg_s"].get<double>() * M_PI / 180.0;
  return p;
}

inline auto_control::drive::ControlParams control_params_from(const nlohmann::json & P) {
  auto_control::drive::ControlParams c;
  const auto & q = P["control"];
  c.stanley_gain = q["stanley_gain"]; c.softening_speed = q["softening_speed"]; c.heading_lookahead = q["heading_lookahead"];
  c.geometry_window = q["geometry_window"]; c.corner_heading_threshold_rad = q["corner_heading_threshold_deg"].get<double>() * M_PI / 180.0;
  c.corner_opposing_ratio = q["corner_opposing_ratio"]; c.steering_current_weight = q["steering_current_weight"];
  c.max_speed = q["max_speed"]; c.obstacle_speed = q["obstacle_speed"]; c.rollout_fail_speed = q["rollout_fail_speed"];
  c.latency = P["perception"]["latency"];
  return c;
}
