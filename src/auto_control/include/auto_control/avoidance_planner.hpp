#pragma once
#include "bev_handoff/avoidance_handoff.hpp"
#include "auto_control/control_core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace auto_control::avoidance
{
struct Options
{
  bool control{false}, centerline_fallback{true};
  double obstacle_size{.13}, motion_margin{0};
  double half_width{.15}, half_length{.25}, margin{.04}, unknown_extent{.04};
  double step{.025}, max_offset{.40}, offset_step{.05}, transition{.70};
  double max_curvature{2.0}, max_speed{.5}, lateral_acceleration{.4};
  double deceleration{.5}, clear_sec{1.0}, stop_response{.35}; // stop_response: legacy YAML compatibility only
  // Imported from the EXISTING controller YAML, not additional departure gates.
  int minimum_points{8};
  double minimum_span{.12}, minimum_x{.01}, maximum_x{2.98}, maximum_gap{.15}, geometry_window{.16};
};
inline void validate(const Options & o)
{
  for (double v : {o.half_width,o.half_length,o.margin,o.unknown_extent,o.step,o.max_offset,
      o.offset_step,o.transition,o.max_curvature,o.max_speed,o.lateral_acceleration,
      o.deceleration,o.clear_sec,o.obstacle_size}) {
    if (!std::isfinite(v) || v<=0) {throw std::invalid_argument("avoidance options must be finite and positive");}
  }
  if (o.half_width>.5 || o.half_length>1 || o.margin>.3 || o.unknown_extent>.5 ||
    o.step<.02 || o.step>.05 || o.max_offset>.6 || o.offset_step<.04 || o.offset_step>o.max_offset ||
    o.transition<.3 || o.transition>2 || o.max_curvature>5 || o.max_speed>1 ||
    o.lateral_acceleration>2 || o.deceleration>2 || o.clear_sec<.3 || o.clear_sec>5 || o.obstacle_size>1) {
    throw std::invalid_argument("avoidance preview settings exceed bounded planner limits");
  }
  if (o.minimum_points<2 || !std::isfinite(o.minimum_span) || o.minimum_span<=0 ||
    !std::isfinite(o.minimum_x) || !std::isfinite(o.maximum_x) || o.maximum_x<=o.minimum_x ||
    !std::isfinite(o.maximum_gap) || o.maximum_gap<=0 || !std::isfinite(o.geometry_window) || o.geometry_window<=0) {
    throw std::invalid_argument("Invalid shared auto_control path settings");
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
// Centerline driving is the default. Only an obstacle conflict requests a new path.
class Planner
{
public:
  explicit Planner(Options options):o_(options) {validate(o_);}
  void reset() {held_side_=0; clear_since_=0; target_.reset();}
  bev_handoff::AvoidancePreview plan(const bev_handoff::PlanningLane & lane,
    const bev_handoff::ObstacleFrame & obstacles,double stamp)
  {
    bev_handoff::AvoidancePreview out;
    out.header=lane.header; out.lane_received_at=lane.received_at;
    out.depth_captured_at=obstacles.captured_at;
    out.x_max=lane.x_max; out.y_max=lane.y_max; out.meter_per_pixel=lane.meter_per_pixel;
    out.width=lane.width; out.height=lane.height;
    if (!lane.valid || lane.center.size()>2000) {out.status="WAIT: centerline input"; return out;}
    std::vector<Point> input;
    for (const auto & p:lane.center) {input.push_back({p.x,p.y});}
    const auto reference=build_ordered_path(input,o_.minimum_points,o_.minimum_span,
      o_.minimum_x,o_.maximum_x,o_.maximum_gap,o_.geometry_window);
    if (!reference) {out.status="WAIT: centerline input"; return out;}
    // Ordered arc length handles horizontal/reversing corner segments exactly
    // as the normal controller does. No forward-X, 25cm-start or 50cm-span gate.
    const double length=reference->arc_m.back();
    const auto count=std::min<std::size_t>(159,std::max<std::size_t>(
      static_cast<std::size_t>(o_.minimum_points-1),static_cast<std::size_t>(std::ceil(length/o_.step))));
    std::vector<double> arc;
    std::vector<cv::Point2d> normals;
    for (std::size_t i=0;i<=count;++i) {
      const double s=length*i/count,heading=reference->heading(s);
      const auto p=reference->point(s);
      out.original.emplace_back(p.x,p.y); arc.push_back(s);
      normals.emplace_back(-std::sin(heading),std::cos(heading));
    }
    out.horizon_m=length;
    std::vector<bev_handoff::SafetyBox> boxes;
    const double radius=o_.half_width+o_.margin+std::max(o_.unknown_extent,o_.obstacle_size/2);
    for (const auto & cluster:obstacles.clusters) {
      bev_handoff::SafetyBox box{1e9,-1e9,1e9,-1e9};
      for (const auto & p:cluster.surface_xy) {
        if (!std::isfinite(p.x)||!std::isfinite(p.y)) {continue;}
        box.x0=std::min(box.x0,double(p.x)); box.x1=std::max(box.x1,double(p.x));
        box.y0=std::min(box.y0,double(p.y)); box.y1=std::max(box.y1,double(p.y));
      }
      if (box.x0>box.x1) {continue;}
      const double y=(box.y0+box.y1)/2;
      box.y0=std::min(box.y0,y-o_.obstacle_size/2)-radius;
      box.y1=std::max(box.y1,y+o_.obstacle_size/2)+radius;
      box.x0-=radius+o_.motion_margin;
      box.x1+=o_.obstacle_size+radius+o_.motion_margin;
      boxes.push_back(box);
      box.x0-=o_.half_length; box.x1+=o_.half_length;
      out.boxes.push_back(box);
    }
    const auto body=[&](const cv::Point2d & p,double heading,bool first) {
      const cv::Point2d tangent(std::cos(heading),std::sin(heading));
      return std::make_pair(first?cv::Point2d(-o_.half_length,0):p-o_.half_length*tangent,
        p+o_.half_length*tangent);
    };
    const auto collision=[&](const std::pair<cv::Point2d,cv::Point2d> & pose,
        const bev_handoff::SafetyBox & source,double allowance) {
      auto box=source; box.x0-=allowance; box.x1+=allowance;
      box.y0-=allowance; box.y1+=allowance;
      return intersects(pose.first,pose.second,box);
    };
    // Inspect the actual baseline points/segments, not a cropped substitute.
    double first=length,last=0;
    bool blocked=false;
    std::optional<cv::Point2d> current_target;
    for (std::size_t i=0;i<reference->points.size();++i) {
      const auto & p=reference->points[i];
      const double s=reference->arc_m[i];
      const double gap=std::max(i?s-reference->arc_m[i-1]:0,
        i+1<reference->arc_m.size()?reference->arc_m[i+1]-s:0);
      const double allowance=.5*gap*(1+o_.half_length*std::max(o_.max_curvature,std::abs(reference->curvature(s))));
      const auto pose=body({p.x,p.y},reference->heading(s),i==0);
      for (const auto & box:boxes) {
        if (!collision(pose,box,allowance)) {continue;}
        if (!blocked || s<first) {current_target=cv::Point2d((box.x0+box.x1)/2,(box.y0+box.y1)/2);}
        blocked=true; first=std::min(first,s); last=std::max(last,s);
      }
    }
    bool target_visible=!target_;
    if (target_) {
      double nearest=.25;
      for (const auto & box:boxes) {
        const cv::Point2d center((box.x0+box.x1)/2,(box.y0+box.y1)/2);
        const double d=distance(center,*target_);
        if (d<nearest) {nearest=d; current_target=center; target_visible=true;}
      }
      if (target_visible && current_target) {target_=current_target;}
    }
    if (o_.control && held_side_ && !target_visible) {
      out.status="BLOCKED: target lost; reset after inspection"; return out;
    }
    if (!blocked) {
      if (held_side_) {
        if (!clear_since_ || stamp<clear_since_) {clear_since_=stamp;}
        if (stamp-clear_since_<o_.clear_sec) {out.status="WAIT: confirm clear"; return out;}
        reset();
      }
      // A positive, fresh obstacle-clear decision selects the normal path_ in
      // auto_control. Missing/invalid depth is never treated as obstacle-clear.
      out.follow_centerline=true; out.selected=out.original;
      out.status="CENTERLINE"; out.recommended_speed=o_.max_speed;
      return out;
    }
    clear_since_=0;
    std::vector<std::pair<cv::Point2d,cv::Point2d>> edges;
    std::size_t edge_points=0;
    for (const auto & line:lane.boundaries) {
      if ((edge_points+=line.size())>2000) {out.status="WAIT: boundary sample limit"; return out;}
      for (std::size_t j=1;j<line.size();++j) {
        if (finite(line[j-1])&&finite(line[j])&&distance(line[j-1],line[j])<=.2) {
          edges.emplace_back(line[j-1],line[j]);
        }
      }
    }
    if (!std::isfinite(lane.lane_width_m)||lane.lane_width_m<=0) {out.status="WAIT: lane width"; return out;}
    // Cross-sections follow the local centerline normal, including corners.
    std::vector<std::pair<cv::Point2d,cv::Point2d>> corridor;
    std::vector<bool> covered;
    for (std::size_t i=0;i<out.original.size();++i) {
      const auto p=out.original[i],n=normals[i];
      double left=std::numeric_limits<double>::infinity(),right=-left;
      for (const auto & edge:edges) {
        const auto d=edge.second-edge.first;
        const double den=cross(n,d);
        if (std::abs(den)<1e-9) {continue;}
        const double along=cross(edge.first-p,n)/den;
        if (along<0 || along>1) {continue;}
        const double across=cross(edge.first-p,d)/den;
        if (across>0) {left=std::min(left,across);}
        if (across<0) {right=std::max(right,across);}
      }
      const bool missing=!std::isfinite(left)||!std::isfinite(right);
      covered.push_back(!missing || o_.centerline_fallback);
      if (missing) {out.inferred_boundaries=true;}
      if (!std::isfinite(left)) {left=lane.lane_width_m/2;}
      if (!std::isfinite(right)) {right=-lane.lane_width_m/2;}
      corridor.emplace_back(p+left*n,p+right*n);
    }
    const double lead=std::hypot(out.original.front().x,out.original.front().y);
    const double approach=lead+first;
    if (approach<=1e-6) {out.status="BLOCKED: obstacle at vehicle"; return out;}
    const double maneuver_end=std::min(length,last+o_.transition);
    const auto check=[&](bev_handoff::AvoidanceCandidate & c) {
      OrderedPath path; path.geometry_window_m=o_.geometry_window;
      double s=0;
      for (std::size_t i=0;i<c.path.size();++i) {
        const auto p=c.path[i];
        if (!finite(p)||p.x<0||p.x>lane.x_max||std::abs(p.y)>lane.y_max) {c.reason="BEV edge"; return;}
        if (i) {const double gap=distance(c.path[i-1],p); if (gap<1e-5||gap>o_.maximum_gap) {c.reason="path gap"; return;} s+=gap;}
        path.points.push_back({p.x,p.y}); path.arc_m.push_back(s);
      }
      if (path.points.size()<static_cast<std::size_t>(o_.minimum_points)||s<o_.minimum_span) {c.reason="centerline input"; return;}
      for (std::size_t i=0;i<c.path.size();++i) {
        const double k=std::abs(path.curvature(path.arc_m[i]));
        c.max_curvature=std::max(c.max_curvature,k);
        if (k>o_.max_curvature) {c.reason="curvature"; return;}
        const auto pose=body(c.path[i],path.heading(path.arc_m[i]),i==0);
        const double gap=std::max(i?path.arc_m[i]-path.arc_m[i-1]:0,
          i+1<c.path.size()?path.arc_m[i+1]-path.arc_m[i]:0);
        const double sampling=.5*gap*(1+o_.half_length*o_.max_curvature);
        const double lane_radius=o_.half_width+o_.margin+sampling;
        if (!covered[i]) {c.reason="unobserved lane edge"; return;}
        const auto n=normals[i];
        if ((c.path[i]-corridor[i].first).dot(n)>=0 || (c.path[i]-corridor[i].second).dot(n)<=0 ||
          std::max(pose.first.y,pose.second.y)+lane_radius>lane.y_max ||
          std::min(pose.first.y,pose.second.y)-lane_radius<-lane.y_max) {c.reason="lane edge"; return;}
        for (std::size_t j=1;j<corridor.size();++j) {
          if (segmentDistance(pose.first,pose.second,corridor[j-1].first,corridor[j].first)<lane_radius ||
            segmentDistance(pose.first,pose.second,corridor[j-1].second,corridor[j].second)<lane_radius) {c.reason="lane clearance"; return;}
        }
        for (const auto & edge:edges) {
          if (std::max(edge.first.x,edge.second.x)<std::min(pose.first.x,pose.second.x)-lane_radius ||
            std::min(edge.first.x,edge.second.x)>std::max(pose.first.x,pose.second.x)+lane_radius ||
            std::max(edge.first.y,edge.second.y)<std::min(pose.first.y,pose.second.y)-lane_radius ||
            std::min(edge.first.y,edge.second.y)>std::max(pose.first.y,pose.second.y)+lane_radius) {continue;}
          if (segmentDistance(pose.first,pose.second,edge.first,edge.second)<lane_radius) {c.reason="observed lane clearance"; return;}
        }
        for (const auto & box:boxes) {if (collision(pose,box,sampling)) {c.reason="obstacle"; return;}}
      }
      c.valid=true; c.reason="valid";
    };
    double best=std::numeric_limits<double>::infinity();
    for (int side:{1,-1}) {
      for (double magnitude=o_.offset_step;magnitude<=o_.max_offset+1e-9;magnitude+=o_.offset_step) {
        bev_handoff::AvoidanceCandidate candidate; candidate.offset=side*magnitude;
        for (std::size_t i=0;i<arc.size();++i) {
          double weight=smooth((lead+arc[i])/approach);
          if (arc[i]>last && length-last>=o_.transition) {weight*=1-smooth((arc[i]-last)/o_.transition);}
          candidate.path.push_back(out.original[i]+normals[i]*(candidate.offset*weight));
          if (arc[i]>=maneuver_end) {break;}
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
    held_side_=out.status=="LEFT"?1:-1; if (current_target) {target_=current_target;}
    out.recommended_speed=std::min(o_.max_speed,std::sqrt(o_.lateral_acceleration/std::max(.001,out.max_curvature)));
    if (!o_.control) {out.recommended_speed=std::min(out.recommended_speed,std::sqrt(2*o_.deceleration*approach));}
    if (out.inferred_boundaries) {out.status+=": centerline corridor";}
    return out;
  }
  void unavailable() {clear_since_=0;}
private:
  Options o_;
  int held_side_{0};
  double clear_since_{0};
  std::optional<cv::Point2d> target_;
};
}  // namespace auto_control::avoidance
