#pragma once
#include "bev_handoff/avoidance_handoff.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace auto_control::avoidance
{
struct Options
{
  double half_width{.15}, half_length{.25}, margin{.04}, unknown_extent{.04};
  double step{.025}, max_offset{.40}, offset_step{.05}, transition{.70};
  double max_curvature{2.0}, max_speed{.5}, lateral_acceleration{.4};
  double deceleration{.5}, clear_sec{1.0};
};
inline void validate(const Options & o)
{
  for (double v : {o.half_width,o.half_length,o.margin,o.unknown_extent,o.step,o.max_offset,
      o.offset_step,o.transition,o.max_curvature,o.max_speed,o.lateral_acceleration,
      o.deceleration,o.clear_sec}) {
    if (!std::isfinite(v) || v<=0) {throw std::invalid_argument("avoidance options must be finite and positive");}
  }
  if (o.half_width>.5 || o.half_length>1 || o.margin>.3 || o.unknown_extent>.5 ||
    o.step<.02 || o.step>.05 || o.max_offset>.6 || o.offset_step<.04 || o.offset_step>o.max_offset ||
    o.transition<.3 || o.transition>2 || o.max_curvature>5 || o.max_speed>1 ||
    o.lateral_acceleration>2 || o.deceleration>2 || o.clear_sec<.3 || o.clear_sec>5) {
    throw std::invalid_argument("avoidance preview settings exceed bounded planner limits");
  }
}
inline double smooth(double t)
{
  t=std::clamp(t,0.0,1.0);
  return t*t*t*(10+t*(-15+6*t));
}
inline bool finite(const cv::Point2d & p) {return std::isfinite(p.x)&&std::isfinite(p.y);}
inline double distance(const cv::Point2d & a,const cv::Point2d & b) {return std::hypot(a.x-b.x,a.y-b.y);}
inline double cross(const cv::Point2d & a,const cv::Point2d & b) {return a.x*b.y-a.y*b.x;}
inline bool intersects(cv::Point2d a,cv::Point2d b,const bev_handoff::SafetyBox & box)
{
  double lo=0,hi=1;
  const auto axis=[&](double p,double d,double lower,double upper) {
    if (std::abs(d)<1e-9) {return p>=lower&&p<=upper;}
    double t0=(lower-p)/d,t1=(upper-p)/d;
    if (t0>t1) {std::swap(t0,t1);}
    lo=std::max(lo,t0); hi=std::min(hi,t1); return lo<=hi;
  };
  return axis(a.x,b.x-a.x,box.x0,box.x1)&&axis(a.y,b.y-a.y,box.y0,box.y1);
}
inline bool hits(const std::vector<cv::Point2d> & path,const bev_handoff::SafetyBox & box)
{
  for (std::size_t i=1;i<path.size();++i) {if (intersects(path[i-1],path[i],box)) {return true;}}
  return false;
}
inline double pointSegment(cv::Point2d p,cv::Point2d a,cv::Point2d b)
{
  const auto d=b-a;
  const double t=d.dot(d)>1e-12?std::clamp((p-a).dot(d)/d.dot(d),0.0,1.0):0;
  return distance(p,a+t*d);
}
inline double segmentDistance(cv::Point2d a,cv::Point2d b,cv::Point2d c,cv::Point2d d)
{
  const auto u=b-a,v=d-c;
  const double denominator=cross(u,v);
  if (std::abs(denominator)>1e-12) {
    const double t=cross(c-a,v)/denominator, w=cross(c-a,u)/denominator;
    if (t>=0 && t<=1 && w>=0 && w<=1) {return 0;}
  }
  return std::min({pointSegment(a,c,d),pointSegment(b,c,d),pointSegment(c,a,b),pointSegment(d,a,b)});
}
struct Corridor {double left,right;};
// Preview planner deliberately requires a forward, single-valued path and two
// observed boundaries. It does not invent lane width across unobserved gaps.
class Planner
{
public:
  explicit Planner(Options options):o_(options) {validate(o_);}
  void reset() {held_side_=0; clear_since_=0;}
  bev_handoff::AvoidancePreview plan(const bev_handoff::PlanningLane & lane,
    const bev_handoff::ObstacleFrame & obstacles,double stamp)
  {
    bev_handoff::AvoidancePreview out;
    out.header=lane.header; out.lane_received_at=lane.received_at;
    out.depth_captured_at=obstacles.captured_at;
    out.x_max=lane.x_max; out.y_max=lane.y_max; out.meter_per_pixel=lane.meter_per_pixel;
    out.width=lane.width; out.height=lane.height;
    if (!lane.valid || lane.center.size()<3) {out.status="WAIT: lane"; clear_since_=0; return out;}
    for (std::size_t i=0;i<lane.center.size();++i) {
      if (!finite(lane.center[i]) || (i && (lane.center[i].x<=lane.center[i-1].x+1e-5 ||
        distance(lane.center[i],lane.center[i-1])>.2))) {
        out.status="WAIT: non-forward lane"; clear_since_=0; return out;
      }
    }
    const double start=lane.center.front().x,end=lane.center.back().x;
    if (start<0 || start>.6 || end-start<.8 || end>lane.x_max || lane.boundaries.empty()) {
      out.status="WAIT: short lane"; clear_since_=0; return out;
    }
    std::size_t segment=1;
    std::vector<Corridor> corridor;
    for (double x=start;x<=end && out.original.size()<160;x+=o_.step) {
      while (segment+1<lane.center.size() && lane.center[segment].x<x) {++segment;}
      const auto & a=lane.center[segment-1]; const auto & b=lane.center[segment];
      const double y=a.y+(b.y-a.y)*(x-a.x)/(b.x-a.x);
      out.original.emplace_back(x,y);
      double left=std::numeric_limits<double>::infinity(),right=-left;
      for (const auto & line:lane.boundaries) {
        for (std::size_t j=1;j<line.size();++j) {
          const auto & p=line[j-1]; const auto & q=line[j];
          if (!finite(p)||!finite(q)||distance(p,q)>.2||x<std::min(p.x,q.x)||x>std::max(p.x,q.x)||std::abs(q.x-p.x)<1e-6) {continue;}
          const double boundary=p.y+(q.y-p.y)*(x-p.x)/(q.x-p.x);
          if (boundary>y) {left=std::min(left,boundary);}
          if (boundary<y) {right=std::max(right,boundary);}
        }
      }
      if (!std::isfinite(left)||!std::isfinite(right)) {
        out.status="WAIT: both boundaries"; clear_since_=0; return out;
      }
      corridor.push_back({left,right});
    }
    // Capsule along the candidate heading covers the rectangular vehicle.
    // Display boxes describe the envelope for a forward-facing vehicle;
    // the collision check below rotates the capsule for each candidate pose.
    const double radius=o_.half_width+o_.margin+o_.unknown_extent;
    std::vector<bev_handoff::SafetyBox> collision_boxes;
    for (const auto & cluster:obstacles.clusters) {
      if (cluster.surface_xy.empty()) {continue;}
      bev_handoff::SafetyBox box{1e9,-1e9,1e9,-1e9};
      for (const auto & p:cluster.surface_xy) {
        if (!std::isfinite(p.x)||!std::isfinite(p.y)) {continue;}
        box.x0=std::min(box.x0,double(p.x)); box.x1=std::max(box.x1,double(p.x));
        box.y0=std::min(box.y0,double(p.y)); box.y1=std::max(box.y1,double(p.y));
      }
      if (box.x0>box.x1) {continue;}
      box.x0-=radius; box.x1+=radius; box.y0-=radius; box.y1+=radius;
      collision_boxes.push_back(box);
      box.x0-=o_.half_length; box.x1+=o_.half_length;
      out.boxes.push_back(box);
    }
    double first=1e9,last=-1e9;
    for (const auto & box:out.boxes) {
      if (hits(out.original,box)) {first=std::min(first,box.x0); last=std::max(last,box.x1);}
    }
    const auto check=[&](bev_handoff::AvoidanceCandidate & c) {
      c.valid=false;
      for (std::size_t i=0;i<c.path.size();++i) {
        const auto & p=c.path[i];
        const auto & before=c.path[i?i-1:i];
        const auto & after=c.path[std::min(i+1,c.path.size()-1)];
        const auto tangent=(after-before)*(1.0/std::max(1e-9,distance(before,after)));
        const auto front=p+o_.half_length*tangent,rear=p-o_.half_length*tangent;
        // Sampling allowance accounts for translation and rotation between
        // adjacent poses, using the accepted maximum curvature as the bound.
        const double sampling=.5*std::max(distance(p,before),distance(p,after))*(1+o_.half_length*o_.max_curvature);
        const double lane_radius=o_.half_width+o_.margin+sampling;
        if (p.y>=corridor[i].left || p.y<=corridor[i].right ||
          std::max(front.y,rear.y)+lane_radius>lane.y_max ||
          std::min(front.y,rear.y)-lane_radius<-lane.y_max) {c.reason="lane edge"; return;}
        for (std::size_t j=1;j<corridor.size();++j) {
          if (std::abs(out.original[j].x-p.x)>o_.half_length+lane_radius+o_.step) {continue;}
          for (int side=0;side<2;++side) {
            const cv::Point2d a(out.original[j-1].x,side?corridor[j-1].right:corridor[j-1].left);
            const cv::Point2d b(out.original[j].x,side?corridor[j].right:corridor[j].left);
            if (segmentDistance(rear,front,a,b)<lane_radius) {c.reason="lane clearance"; return;}
          }
        }
        for (auto box:collision_boxes) {
          box.x0-=sampling; box.x1+=sampling; box.y0-=sampling; box.y1+=sampling;
          if (intersects(rear,front,box)) {c.reason="obstacle"; return;}
        }
      }
      for (std::size_t i=1;i+1<c.path.size();++i) {
        const auto a=c.path[i]-c.path[i-1],b=c.path[i+1]-c.path[i];
        const double denominator=distance(c.path[i-1],c.path[i])*distance(c.path[i],c.path[i+1])*distance(c.path[i-1],c.path[i+1]);
        if (denominator<1e-9) {c.reason="degenerate"; return;}
        c.max_curvature=std::max(c.max_curvature,2*std::abs(cross(a,b))/denominator);
      }
      if (c.max_curvature>o_.max_curvature) {c.reason="curvature"; return;}
      c.valid=true; c.reason="valid";
    };
    bev_handoff::AvoidanceCandidate nominal; nominal.path=out.original; check(nominal);
    if (nominal.valid) {first=1e9;}
    if (first==1e9) {
      if (!nominal.valid) {out.status="BLOCKED: "+nominal.reason; clear_since_=0; return out;}
      if (held_side_) {
        if (!clear_since_ || stamp<clear_since_) {clear_since_=stamp;}
        if (stamp-clear_since_<o_.clear_sec) {out.status="WAIT: confirm clear"; return out;}
        held_side_=0;
      }
      out.selected=out.original; out.max_curvature=nominal.max_curvature; out.status="CLEAR";
    } else {
      clear_since_=0;
      if (first-start<o_.transition || end-last<o_.transition) {
        out.status="BLOCKED: transition space"; return out;
      }
      double best=1e9;
      for (int side:{1,-1}) {
        for (double magnitude=o_.offset_step;magnitude<=o_.max_offset+1e-9;magnitude+=o_.offset_step) {
          bev_handoff::AvoidanceCandidate candidate; candidate.offset=side*magnitude;
          for (const auto & p:out.original) {
            // Use all observed approach/return space to reduce curvature;
            // transition is the minimum required length, not a fixed ramp.
            const double weight=p.x<first?smooth((p.x-start)/(first-start)):
              p.x>last?1-smooth((p.x-last)/(end-last)):1;
            candidate.path.emplace_back(p.x,p.y+candidate.offset*weight);
          }
          check(candidate);
          const double cost=magnitude+.05*candidate.max_curvature;
          if (candidate.valid && (!held_side_||held_side_==side) && cost<best) {
            best=cost; out.selected=candidate.path; out.max_curvature=candidate.max_curvature;
            out.status=side>0?"LEFT":"RIGHT";
          }
          out.candidates.push_back(std::move(candidate));
        }
      }
      if (out.selected.empty()) {out.status="BLOCKED: no safe candidate"; return out;}
      held_side_=out.status=="LEFT"?1:-1;
    }
    out.recommended_speed=std::min(o_.max_speed,std::sqrt(o_.lateral_acceleration/std::max(.001,out.max_curvature)));
    // Advisory speed also allows a stop before the first nominal-path obstacle.
    if (first<1e9) {out.recommended_speed=std::min(out.recommended_speed,std::sqrt(2*o_.deceleration*std::max(0.0,first-start)));}
    return out;
  }
  void unavailable() {clear_since_=0;}
private:
  Options o_;
  int held_side_{0};
  double clear_since_{0};
};
}  // namespace auto_control::avoidance
