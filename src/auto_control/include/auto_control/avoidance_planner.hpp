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
  double step{.025}, max_offset{.40}, transition{.70};
  double max_curvature{2.0}, max_speed{.5}, lateral_acceleration{.4};
  double deceleration{.5}, clear_sec{1.0};
  // Imported from the EXISTING controller YAML, not additional departure gates.
  int minimum_points{8};
  double minimum_span{.12}, minimum_x{.01}, maximum_x{2.98}, maximum_gap{.15}, geometry_window{.16};
};
inline void validate(const Options & o)
{
  for (double v : {o.half_width,o.half_length,o.margin,o.unknown_extent,o.step,o.max_offset,
      o.transition,o.max_curvature,o.max_speed,o.lateral_acceleration,
      o.deceleration,o.clear_sec,o.obstacle_size}) {
    if (!std::isfinite(v) || v<=0) {throw std::invalid_argument("avoidance options must be finite and positive");}
  }
  if (o.half_width>.5 || o.half_length>1 || o.margin>.3 || o.unknown_extent>.5 ||
    o.step<.02 || o.step>.05 || o.max_offset>.6 ||
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
// Exact interval of translations along normal for which a body segment
// intersects the (already inflated) obstacle AABB. The separating axes of a
// segment and rectangle are X, Y, and the segment's perpendicular. Intersect
// their translation intervals instead of projecting the entire rectangle onto
// normal: far corners outside the longitudinal overlap must not force a detour.
inline std::optional<std::pair<double,double>> blockedOffsets(
  const std::pair<cv::Point2d,cv::Point2d> & pose,const cv::Point2d & normal,
  const bev_handoff::SafetyBox & box,double allowance)
{
  const cv::Point2d center((box.x0+box.x1)/2,(box.y0+box.y1)/2);
  const double hx=(box.x1-box.x0)/2+allowance,hy=(box.y1-box.y0)/2+allowance;
  const auto segment=pose.second-pose.first;
  double low=-std::numeric_limits<double>::infinity(),high=-low;
  for (const auto & axis:{cv::Point2d(1,0),cv::Point2d(0,1),cv::Point2d(-segment.y,segment.x)}) {
    if (axis.dot(axis)<1e-18) {continue;}
    const double a=pose.first.dot(axis),b=pose.second.dot(axis);
    const double extent=hx*std::abs(axis.x)+hy*std::abs(axis.y);
    const double box_low=center.dot(axis)-extent,box_high=center.dot(axis)+extent;
    const double shift=normal.dot(axis);
    if (std::abs(shift)<1e-12) {
      if (std::max(a,b)<box_low || std::min(a,b)>box_high) {return std::nullopt;}
      continue;
    }
    double enter=(box_low-std::max(a,b))/shift,leave=(box_high-std::min(a,b))/shift;
    if (enter>leave) {std::swap(enter,leave);}
    low=std::max(low,enter); high=std::min(high,leave);
    if (low>high) {return std::nullopt;}
  }
  return std::make_pair(low,high);
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
// Build one local displacement profile along the existing ordered centerline.
// Each blocking obstacle chooses its OWN passing side; no global side latch.
class Planner
{
  struct Target {cv::Point2d center; int side;};
  struct Region {double begin, end, offset;};
public:
  explicit Planner(Options options):o_(options) {validate(o_);}
  void reset() {targets_.clear(); clear_since_=0;}
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
    std::vector<cv::Point2d> centers;
    // Keep the established physical/uncertainty envelope. Changing how the path
    // is constructed must not silently shrink obstacle or vehicle dimensions.
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
      centers.emplace_back((box.x0+box.x1)/2,y);
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
    // Per-object matching stabilizes passing direction across frames without
    // forcing all later objects to share the first object's direction.
    std::vector<int> sides(boxes.size(),0);
    std::vector<bool> matched(boxes.size(),false);
    std::vector<Target> visible_targets;
    for (auto & target:targets_) {
      std::size_t best=boxes.size(); double nearest=.25;
      for (std::size_t j=0;j<centers.size();++j) {
        const double d=distance(target.center,centers[j]);
        if (!matched[j] && d<nearest) {best=j; nearest=d;}
      }
      if (best==boxes.size()) {
        if (o_.control) {out.status="BLOCKED: target lost; reset after inspection"; return out;}
        continue;
      }
      matched[best]=true; sides[best]=target.side; target.center=centers[best];
      visible_targets.push_back(target);
    }
    targets_=std::move(visible_targets);
    // Geometry is independent of obstacle count; compute it once per frame.
    struct Sample {
      cv::Point2d normal;
      std::pair<cv::Point2d,cv::Point2d> pose;
      double arc,gap,allowance;
    };
    std::vector<Sample> baseline;
    baseline.reserve(reference->points.size());
    for (std::size_t i=0;i<reference->points.size();++i) {
      const auto p=reference->points[i]; const double s=reference->arc_m[i];
      const double heading=reference->heading(s);
      const double gap=std::max(i?s-reference->arc_m[i-1]:0,
        i+1<reference->arc_m.size()?reference->arc_m[i+1]-s:0);
      const double allowance=.5*gap*(1+o_.half_length*std::max(o_.max_curvature,std::abs(reference->curvature(s))));
      baseline.push_back({{-std::sin(heading),std::cos(heading)},
        body({p.x,p.y},heading,i==0),s,gap,allowance});
    }
    std::vector<Region> regions;
    std::vector<bool> active(boxes.size(),false);
    std::vector<int> preferred_sides(boxes.size(),0);
    std::vector<Target> active_targets;
    const double sampling=.5*(length/count)*(1+o_.half_length*o_.max_curvature);
    for (std::size_t j=0;j<boxes.size();++j) {
      // Find the obstacle's local side with respect to the nearest segment,
      // not vehicle Y. This keeps left/right meaningful around corners.
      double nearest=std::numeric_limits<double>::infinity(),at=0;
      for (std::size_t i=1;i<reference->points.size();++i) {
        const auto a=reference->points[i-1],b=reference->points[i];
        const cv::Point2d origin(a.x,a.y),d(b.x-a.x,b.y-a.y);
        const double t=std::clamp((centers[j]-origin).dot(d)/d.dot(d),0.0,1.0);
        const double gap=distance(centers[j],origin+t*d);
        if (gap<nearest) {nearest=gap; at=reference->arc_m[i-1]+t*(reference->arc_m[i]-reference->arc_m[i-1]);}
      }
      const auto middle=reference->point(at);
      const double heading=reference->heading(at);
      const cv::Point2d normal(-std::sin(heading),std::cos(heading));
      const double lateral=(centers[j]-cv::Point2d(middle.x,middle.y)).dot(normal);
      // Exact-center objects deterministically pass left. Matching then holds
      // that choice for this object only; the final lane check still applies.
      const int side=sides[j]?sides[j]:(lateral>0?-1:1);
      preferred_sides[j]=side;
      double begin=length,end=0,offset=0;
      bool blocking=false;
      for (const auto & sample:baseline) {
        if (!collision(sample.pose,boxes[j],sample.allowance)) {continue;}
        blocking=true;
        begin=std::min(begin,std::max(0.0,sample.arc-sample.gap));
        end=std::max(end,std::min(length,sample.arc+sample.gap));
        const auto forbidden=blockedOffsets(sample.pose,sample.normal,boxes[j],std::max(sampling,sample.allowance));
        if (forbidden) {
          const double required=(side>0?forbidden->second:forbidden->first)+side*1e-4;
          offset=side>0?std::max(offset,required):std::min(offset,required);
        }
      }
      if (!blocking) {continue;}
      ++out.obstacle_count; active[j]=true;
      if (std::abs(offset)>o_.max_offset) {
        out.status=cv::format("BLOCKED: offset %.2fm > %.2fm (cluster=%zu xy=%.2f,%.2f s=%.2f..%.2f)",
          std::abs(offset),o_.max_offset,j+1,centers[j].x,centers[j].y,begin,end);
        return out;
      }
      regions.push_back({begin,end,offset});
      active_targets.push_back({centers[j],side});
    }
    if (regions.empty()) {
      if (!targets_.empty()) {
        if (!clear_since_ || stamp<clear_since_) {clear_since_=stamp;}
        if (stamp-clear_since_<o_.clear_sec) {out.status="WAIT: confirm clear"; return out;}
        reset();
      }
      out.follow_centerline=true; out.selected=out.original;
      out.status="CENTERLINE"; out.recommended_speed=o_.max_speed; return out;
    }
    clear_since_=0;
    std::vector<std::pair<cv::Point2d,cv::Point2d>> edges;
    std::size_t edge_points=0;
    for (const auto & line:lane.boundaries) {
      if ((edge_points+=line.size())>2000) {out.status="WAIT: boundary sample limit"; return out;}
      for (std::size_t j=1;j<line.size();++j) {
        if (finite(line[j-1])&&finite(line[j])&&distance(line[j-1],line[j])<=.2) {edges.emplace_back(line[j-1],line[j]);}
      }
    }
    if (!std::isfinite(lane.lane_width_m)||lane.lane_width_m<=0) {out.status="WAIT: lane width"; return out;}
    std::vector<std::pair<cv::Point2d,cv::Point2d>> corridor;
    std::vector<bool> covered;
    for (std::size_t i=0;i<out.original.size();++i) {
      const auto p=out.original[i],n=normals[i];
      double left=std::numeric_limits<double>::infinity(),right=-left;
      for (const auto & edge:edges) {
        const auto d=edge.second-edge.first; const double den=cross(n,d);
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
        if (k>o_.max_curvature) {c.reason="curvature at s="+std::to_string(path.arc_m[i]); return;}
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
            segmentDistance(pose.first,pose.second,corridor[j-1].second,corridor[j].second)<lane_radius) {c.reason="lane clearance at s="+std::to_string(path.arc_m[i]); return;}
        }
        for (const auto & edge:edges) {
          if (std::max(edge.first.x,edge.second.x)<std::min(pose.first.x,pose.second.x)-lane_radius ||
            std::min(edge.first.x,edge.second.x)>std::max(pose.first.x,pose.second.x)+lane_radius ||
            std::max(edge.first.y,edge.second.y)<std::min(pose.first.y,pose.second.y)-lane_radius ||
            std::min(edge.first.y,edge.second.y)>std::max(pose.first.y,pose.second.y)+lane_radius) {continue;}
          if (segmentDistance(pose.first,pose.second,edge.first,edge.second)<lane_radius) {c.reason="observed lane clearance"; return;}
        }
        for (const auto & box:boxes) {if (collision(pose,box,sampling)) {c.reason="obstacle at s="+std::to_string(path.arc_m[i]); return;}}
      }
      c.valid=true; c.reason="valid";
    };
    // Only three smoothing lengths, independent of the number of obstacles.
    // Each attempt is already a multi-obstacle path, not a global left/right offset.
    for (double scale:{1.0,1.5,2.0}) {
      std::sort(regions.begin(),regions.end(),[](const Region & a,const Region & b) {return a.begin<b.begin;});
      std::vector<Region> merged;
      for (const auto & region:regions) {
        if (std::abs(region.offset)>o_.max_offset) {
          out.status=cv::format("BLOCKED: offset %.2fm > %.2fm (s=%.2f..%.2f)",
            std::abs(region.offset),o_.max_offset,region.begin,region.end);
          return out;
        }
        if (merged.empty() || region.begin>merged.back().end+1e-9) {merged.push_back(region); continue;}
        auto & previous=merged.back();
        // A zero-offset zone means a previously clear object now requires a
        // return to the baseline. Do not overwrite it with a passing plateau.
        if ((previous.offset>0)!=(region.offset>0) || (previous.offset<0)!=(region.offset<0)) {
          out.status="BLOCKED: opposing obstacle zones overlap"; return out;
        }
        previous.end=std::max(previous.end,region.end);
        previous.offset=region.offset>0?std::max(previous.offset,region.offset):std::min(previous.offset,region.offset);
      }
      regions=std::move(merged); out.region_count=regions.size();
      const double transition=o_.transition*scale;
      const double maneuver_end=std::min(length,regions.back().end+transition);
      bev_handoff::AvoidanceCandidate candidate; candidate.transition_m=transition;
      for (std::size_t i=0;i<arc.size();++i) {
        const double offset=displacement(arc[i],regions,transition,lead,length);
        candidate.max_offset_m=std::max(candidate.max_offset_m,std::abs(offset));
        candidate.path.push_back(out.original[i]+normals[i]*offset);
        if (arc[i]>=maneuver_end) {break;}
      }
      // A detour may encounter objects that did not intersect the baseline.
      // Activate ALL such objects in this pass, not just a single nearest one.
      OrderedPath proposed; proposed.geometry_window_m=o_.geometry_window;
      double proposed_arc=0;
      for (std::size_t i=0;i<candidate.path.size();++i) {
        if (i) {proposed_arc+=distance(candidate.path[i-1],candidate.path[i]);}
        proposed.points.push_back({candidate.path[i].x,candidate.path[i].y});
        proposed.arc_m.push_back(proposed_arc);
      }
      std::vector<std::pair<cv::Point2d,cv::Point2d>> proposed_poses;
      std::vector<double> proposed_allowances;
      for (std::size_t i=0;i<candidate.path.size();++i) {
        proposed_poses.push_back(body(candidate.path[i],proposed.heading(proposed.arc_m[i]),i==0));
        const double gap=std::max(i?proposed.arc_m[i]-proposed.arc_m[i-1]:0,
          i+1<proposed.arc_m.size()?proposed.arc_m[i+1]-proposed.arc_m[i]:0);
        proposed_allowances.push_back(.5*gap*(1+o_.half_length*o_.max_curvature));
      }
      bool added=false;
      for (std::size_t j=0;j<boxes.size();++j) {
        if (active[j]) {continue;}
        const int side=preferred_sides[j];
        double begin=length,end=0,offset=0;
        bool hit=false;
        for (std::size_t i=0;i<candidate.path.size();++i) {
          const double allowance=proposed_allowances[i];
          if (!collision(proposed_poses[i],boxes[j],allowance)) {continue;}
          hit=true;
          begin=std::min(begin,std::max(0.0,arc[i]-length/count));
          end=std::max(end,std::min(length,arc[i]+length/count));
          const auto forbidden=blockedOffsets(proposed_poses[i],normals[i],boxes[j],allowance);
          if (forbidden) {
            const double current_offset=(candidate.path[i]-out.original[i]).dot(normals[i]);
            const double required=current_offset+(side>0?forbidden->second:forbidden->first)+side*1e-4;
            offset=side>0?std::max(offset,required):std::min(offset,required);
          }
        }
        if (!hit) {continue;}
        regions.push_back({begin,end,offset}); active[j]=true; added=true;
        ++out.obstacle_count; active_targets.push_back({centers[j],side});
      }
      if (added) {
        candidate.reason="additional obstacle constraints";
        out.candidates.push_back(std::move(candidate));
        continue;
      }
      check(candidate);
      if (candidate.valid) {
        out.selected=candidate.path; out.max_curvature=candidate.max_curvature;
        out.status="LOCAL: obstacles="+std::to_string(out.obstacle_count)+" zones="+std::to_string(out.region_count);
        // Preserve every previously committed target until a confirmed clear
        // decision, rather than dropping an occluded object when another appears.
        for (const auto & target:active_targets) {
          const auto found=std::find_if(targets_.begin(),targets_.end(),[&](const Target & old) {
            return distance(old.center,target.center)<1e-6;
          });
          if (found==targets_.end()) {targets_.push_back(target);}
        }
        out.recommended_speed=std::min(o_.max_speed,std::sqrt(o_.lateral_acceleration/std::max(.001,out.max_curvature)));
        if (!o_.control) {out.recommended_speed=std::min(out.recommended_speed,std::sqrt(2*o_.deceleration*(lead+regions.front().begin)));}
        out.candidates.push_back(std::move(candidate));
        return out;
      }
      out.candidates.push_back(std::move(candidate));
    }
    out.status="BLOCKED: "+out.candidates.back().reason;
    return out;
  }
  void unavailable() {clear_since_=0;}
private:
  static double blend(double a,double b,double s,double begin,double end)
  {return a+(b-a)*smooth((s-begin)/std::max(1e-9,end-begin));}
  static double displacement(double s,const std::vector<Region> & regions,
    double transition,double lead,double length)
  {
    const auto & first=regions.front();
    if (s<first.begin) {
      return blend(0,first.offset,s,std::max(-lead,first.begin-transition),first.begin);
    }
    for (std::size_t j=0;j<regions.size();++j) {
      const auto & current=regions[j];
      if (s<=current.end) {return current.offset;}
      if (j+1<regions.size()) {
        const auto & next=regions[j+1];
        if (s>=next.begin) {continue;}
        if (next.begin-current.end<2*transition || current.offset*next.offset>0) {
          // A direct smooth connection can change sign: right -> left -> right
          // for any number of ordered obstacles, without summing opposite bumps.
          return blend(current.offset,next.offset,s,current.end,next.begin);
        }
        if (s<current.end+transition) {return blend(current.offset,0,s,current.end,current.end+transition);}
        return blend(0,next.offset,s,next.begin-transition,next.begin);
      }
      if (length-current.end<transition) {return current.offset;}
      return blend(current.offset,0,s,current.end,current.end+transition);
    }
    return 0;
  }
  Options o_;
  std::vector<Target> targets_;
  double clear_since_{0};
};
}  // namespace auto_control::avoidance
