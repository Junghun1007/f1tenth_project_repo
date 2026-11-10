#pragma once
#include "roi_vfh_test/motion.hpp"
#include <optional>
#include <string>

namespace roi_vfh_test {
struct LaneInputLimits {
  int minimum_points{8};
  double minimum_x{.01},maximum_x{2.98},maximum_gap{.15};
};
struct LaneReference {
  std::vector<Point> points;
  std::string state{"missing"};
  bool trusted{false},has_memory{false};
  double memory_age{0};
};
// This stores full trusted geometry, not a heading or a mission classification.
// A missing lane never expires the return preference while odometry is continuous.
class LaneMemory {
  std::vector<Point> trusted_world_,pending_world_;
  double pending_since_{-1},pending_at_{-1},trusted_at_{-1};
  std::optional<double> evidence_stamp_;
  bool evidence_trusted_{false};
public:
  void reset() {
    trusted_world_.clear();pending_world_.clear();pending_since_=pending_at_=trusted_at_=-1;
    evidence_stamp_.reset();evidence_trusted_=false;
  }
  LaneReference resolve(const std::vector<Point>& observed,const MotionPose& pose,const Config& c,
      std::optional<double> capture=std::nullopt,const LaneInputLimits& limits={}) {
    LaneReference result;
    if(c.lane_memory_enabled==0) {trusted_world_.clear();trusted_at_=-1;}
    const auto worldPoint=[&](Point p) {
      return Point{pose.x+std::cos(pose.yaw)*p.x-std::sin(pose.yaw)*p.y,
        pose.y+std::sin(pose.yaw)*p.x+std::cos(pose.yaw)*p.y};
    };
    const auto localPoint=[&](Point p) {
      const double x=p.x-pose.x,y=p.y-pose.y;
      return Point{std::cos(pose.yaw)*x+std::sin(pose.yaw)*y,
        -std::sin(pose.yaw)*x+std::cos(pose.yaw)*y};
    };
    std::vector<auto_control::Point> samples;samples.reserve(observed.size());
    for(auto p:observed)samples.push_back({p.x,p.y});
    const auto ordered=auto_control::build_ordered_path(samples,limits.minimum_points,
      c.minimum_path_length,limits.minimum_x,limits.maximum_x,limits.maximum_gap,c.geometry_window);
    std::vector<Point> lane;
    if(ordered)for(auto p:ordered->points)lane.push_back({p.x,p.y});
    const double evidence=capture.value_or(pose.t);
    if(ordered && Polyline(lane).length()>=c.lane_trust_length) {
      // Reusing the same camera exposure must not advance its confirmation time.
      if(!evidence_stamp_ || evidence>*evidence_stamp_) {
        std::vector<Point> world;world.reserve(lane.size());
        for(auto p:lane)world.push_back(worldPoint(p));
        bool consistent=!pending_world_.empty() && evidence-pending_at_<=.15;
        if(consistent) {
          const Polyline prior(pending_world_);
          const auto probe=worldPoint(lane.front());
          consistent=laneDistance(probe,pending_world_)<=c.lane_trust_offset &&
            std::abs(wrap(prior.heading(probe)-(pose.yaw+ordered->heading(c.lane_lookahead))))
              <=c.lane_trust_angle_deg*pi/180;
        }
        if(!consistent)pending_since_=evidence;
        pending_world_=std::move(world);pending_at_=evidence;evidence_stamp_=evidence;
        evidence_trusted_=evidence-pending_since_+1e-9>=c.lane_trust_duration;
        if(evidence_trusted_ && c.lane_memory_enabled!=0) {trusted_world_=pending_world_;trusted_at_=evidence;}
      }
      result.trusted=evidence_trusted_;
      result.state=result.trusted?"trusted":"confirming";
    }else {
      pending_world_.clear();pending_since_=-1;evidence_trusted_=false;
      if(ordered)result.state="too short";
    }
    result.has_memory=trusted_at_>=0;
    result.memory_age=result.has_memory?std::max(0.,pose.t-trusted_at_):0.;
    if(result.trusted)result.points=std::move(lane);
    else {
      for(auto p:trusted_world_)result.points.push_back(localPoint(p));
      if(result.points.size()>=2) {
        const auto last=result.points.back(),previous=result.points[result.points.size()-2];
        const double heading=std::atan2(last.y-previous.y,last.x-previous.x);
        if(std::cos(heading)>.15 && last.x<c.horizon+c.reference_distance) {
          const double extra=(c.horizon+c.reference_distance-last.x)/std::cos(heading);
          result.points.push_back({last.x+extra*std::cos(heading),last.y+extra*std::sin(heading)});
        }
      }
      if(result.points.size()<2)result.points={{0,0},{std::max(c.horizon,c.reference_distance),0}};
    }
    return result;
  }
  std::vector<Point> remembered(const MotionPose& pose)const {
    std::vector<Point> result;result.reserve(trusted_world_.size());
    for(auto p:trusted_world_) {
      const double x=p.x-pose.x,y=p.y-pose.y;
      result.push_back({std::cos(pose.yaw)*x+std::sin(pose.yaw)*y,
        -std::sin(pose.yaw)*x+std::cos(pose.yaw)*y});
    }
    return result;
  }
};

}
