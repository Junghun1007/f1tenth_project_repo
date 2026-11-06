#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>
namespace roi_vfh_test {
constexpr double pi=3.14159265358979323846;
struct Point {double x{},y{};};
struct Pose {double x{},y{},yaw{},steer{};};
struct Grid {
  int width{},height{}; double resolution{},x0{},y0{};
  std::vector<std::int8_t> cells;
  int at(double x,double y) const {
    const int u=int(std::floor((x-x0)/resolution)),v=int(std::floor((y-y0)/resolution));
    return u<0 || v<0 || u>=width || v>=height ? -1 : cells[std::size_t(v)*width+u];
  }
  void validate() const {
    if (width<1 || height<1 || width>2000 || height>2000 ||
      std::size_t(width)*height>250000 || cells.size()!=std::size_t(width)*height ||
      !std::isfinite(resolution) || resolution<.01 || resolution>.25 || !std::isfinite(x0) || !std::isfinite(y0))
      throw std::invalid_argument("invalid metric terrain grid");
  }
};
// Unknown (-1) is traversable for this preview; positive occupancy remains blocked.
inline bool blockedTerrain(int value) {return value>0;}
struct Config {
  double speed{.7},horizon{1.4},step{.04},wheelbase{.33},width{.30},rear{.25},front{.25};
  double margin{.02},max_steer{30*pi/180},steer_rate{240*pi/180};
  double lane_width{.65},departure{.40},deceleration{.8},delay{.20};
  double current_steer{0};
  double reference_distance{1.0},hist_low{1.0},hist_high{2.0};
  void validate() const {
    for (double a:{speed,horizon,step,wheelbase,width,rear,front,margin,max_steer,steer_rate,
      lane_width,departure,deceleration,delay,reference_distance,hist_low,hist_high})
      if (!std::isfinite(a) || a<0) throw std::invalid_argument("nonfinite/negative VFH parameter");
    if (!std::isfinite(current_steer) || std::abs(current_steer)>max_steer || speed<.05 || speed>1 || horizon<.3 || horizon>2.5 || step<.02 || step>.08 ||
      wheelbase<.1 || wheelbase>1 || width<.1 || width>1 || rear>1 || front>.5 || margin>.2 ||
      max_steer<.05 || max_steer>pi/3 || steer_rate<.1 || steer_rate>20 ||
      lane_width<width || lane_width>2 || departure>.4 || deceleration<.1 || deceleration>3 || delay>1 || reference_distance<.3 || hist_low>=hist_high || hist_high>1000)
      throw std::invalid_argument("VFH parameter outside test limits");
  }
};
inline double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
inline double laneDistance(Point p,const std::vector<Point>& lane) {
  double best=1e9;
  for (std::size_t i=1;i<lane.size();++i) {
    const auto a=lane[i-1],b=lane[i]; const double dx=b.x-a.x,dy=b.y-a.y,d2=dx*dx+dy*dy;
    if (d2<1e-9) continue;
    const double t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/d2,0.,1.);
    best=std::min(best,std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy));
  }
  return best;
}
struct Candidate {std::vector<Pose> path; double score{},length{},direction{}; bool valid{};};
struct Plan {
  std::vector<Candidate> candidates; std::vector<Pose> path;
  std::array<double,61> histogram{};
  std::array<bool,61> binary{},masked{};
  double speed{},steer{},length{}; bool guidance{},rear_unverified{true};
  std::string status{"STOP: no feasible route"};
};
// VFH+ four-stage histogram reduction, followed by Ackermann rollout verification.
// The preview explicitly reports that negative-X (rear) terrain is unobserved.
class Planner {
  std::array<bool,61> binary_{};
  bool guidance_{false}; int guide_confirm_{0},exit_confirm_{0}; double previous_angle_{0},binary_rotation_residual_{0};
public:
  void reset() {guidance_=false; guide_confirm_=exit_confirm_=0; previous_angle_=0;binary_rotation_residual_=0;binary_.fill(false);}
  void rotateReference(double yaw) {
    previous_angle_=wrap(previous_angle_-yaw);
    binary_rotation_residual_+=yaw;
    const int shift=int(std::lround(binary_rotation_residual_/(2.5*pi/180)));
    binary_rotation_residual_-=shift*(2.5*pi/180);
    if(shift) {
      const auto old=binary_;
      for(int b=0;b<61;++b)binary_[b]=(b+shift>=0 && b+shift<61)?old[b+shift]:true;
    }
  }
  bool guidance() const {return guidance_;}
  Plan plan(const Grid& g,const std::vector<Point>& lane,const Config& c,const Grid* current=nullptr) {
    g.validate(); c.validate(); Plan out;
    if(current) {
      current->validate();
      if(current->width!=g.width || current->height!=g.height || current->resolution!=g.resolution ||
        current->x0!=g.x0 || current->y0!=g.y0)throw std::invalid_argument("current terrain geometry mismatch");
    }
    const Grid& evidence=current?*current:g;
    struct Band {double x,lo,hi;};std::vector<Band> bands,best_bands;
    double last_center=0;
    for(double x=.3;x<std::min(c.horizon+.5,g.x0+g.width*g.resolution);x+=g.resolution) {
      if(!bands.empty() && x-bands.back().x>.15) {
        if(best_bands.empty() || bands.back().x-bands.front().x>best_bands.back().x-best_bands.front().x)best_bands=bands;
        bands.clear();last_center=0;
      }
      double wall=0;bool have=false,found=false;Band best{};double error=1e9;
      for(int v=0;v<evidence.height;++v){
        const double y=evidence.y0+(v+.5)*evidence.resolution;
        if(evidence.at(x,y)<50)continue;
        const double gap=y-wall,center=(y+wall)/2;
        if(have && gap>=c.width+2*c.margin && gap<c.lane_width+.3 &&
          (bands.empty() || (std::abs(center-last_center)<.18 && x-bands.back().x<=.15))) {
          const double e=std::abs(center-last_center);
          bool free=true;for(double q=wall+evidence.resolution;q<y-evidence.resolution*.5;q+=evidence.resolution)if(blockedTerrain(evidence.at(x,q)))free=false;
          if(free && e<error){best={x,wall,y};error=e;found=true;}
        }
        wall=y;have=true;
      }
      if(found){bands.push_back(best);last_center=(best.lo+best.hi)/2;}
    }
    if(!best_bands.empty() && (bands.empty() || best_bands.back().x-best_bands.front().x>bands.back().x-bands.front().x))bands=std::move(best_bands);
    const bool pair_valid=bands.size()>=5 && bands.back().x-bands.front().x>=.35;
    bool lane_blocked=false;
    for(const auto& band:bands) {
      Point near{};double distance=1e9;
      for(auto p:lane)if(std::abs(p.x-band.x)<distance){near=p;distance=std::abs(p.x-band.x);}
      if(near.y<band.lo+c.width/2 || near.y>band.hi-c.width/2 || blockedTerrain(evidence.at(band.x,near.y)))lane_blocked=true;
    }
    const bool corridor=pair_valid && lane_blocked;
    std::vector<Point> reference=lane;
    if(lane.size()<2 && (!guidance_ || !pair_valid)) {out.status="STOP: lane/corridor reference unavailable";return out;}
    if(lane.size()>1000) {out.status="STOP: lane sample limit";return out;}
    for(auto p:lane)if(!std::isfinite(p.x)||!std::isfinite(p.y)){out.status="STOP: invalid lane";return out;}
    if (corridor) ++guide_confirm_; else guide_confirm_=0;
    if (guide_confirm_>=3) guidance_=true;
    if (guidance_ && !pair_valid) {if (++exit_confirm_>=8) guidance_=false;} else exit_confirm_=0;
    out.guidance=guidance_;
    if(guidance_ && pair_valid) {
      reference.clear();for(const auto& b:bands)reference.push_back({b.x,(b.lo+b.hi)/2});
    }
    if(reference.size()<2){out.status="STOP: lost corridor";return out;}
    Point ref=reference.back();
    for(auto p:reference)if(std::hypot(p.x,p.y)>=c.reference_distance){ref=p;break;}
    const double target_angle=std::clamp(std::atan2(ref.y,std::max(.05,ref.x)),-pi/2,pi/2);
    const double half=c.width/2+c.margin, radius=half+g.resolution*.71;
    const double turn_radius=c.wheelbase/std::tan(c.max_steer);
    const double front_turn_radius=std::hypot(turn_radius,c.wheelbase);
    double left_limit=pi,right_limit=-pi;
    for (int v=0;v<g.height;++v) for (int u=0;u<g.width;++u) {
      if (!blockedTerrain(g.cells[std::size_t(v)*g.width+u])) continue;
      const double x=g.x0+(u+.5)*g.resolution,y=g.y0+(v+.5)*g.resolution;
      if (x<0 || (x<c.front && std::abs(y)<c.width/2)) continue;
      const double d=std::hypot(x,y); if (d<.05 || d>c.horizon+.3) continue;
      const double angle=std::atan2(y,x);
      // Minimum-turn-circle masking, VFH+ stage 3. Rectangle verification follows.
      if (angle>0 && std::hypot(x+c.wheelbase,y-turn_radius)<front_turn_radius+radius) left_limit=std::min(left_limit,angle);
      if (angle<0 && std::hypot(x+c.wheelbase,y+turn_radius)<front_turn_radius+radius) right_limit=std::max(right_limit,angle);
      const double spread=std::asin(std::min(1.,radius/d));
      for (int b=0;b<61;++b) {
        const double a=(-75+2.5*b)*pi/180;
        if (std::abs(wrap(a-angle))<=spread) out.histogram[b]+=std::max(0.,1-d*d/((c.horizon+.3)*(c.horizon+.3)))*std::pow(g.resolution/.05,2);
      }
    }
    for (int b=0;b<61;++b) {
      if (out.histogram[b]>c.hist_high) binary_[b]=true;
      else if (out.histogram[b]<c.hist_low) binary_[b]=false;
      out.binary[b]=binary_[b];
      const double angle=(-75+2.5*b)*pi/180;
      out.masked[b]=binary_[b] || angle>=left_limit || angle<=right_limit;
    }
    // Lane departure allowance is rasterized once; path sampling uses constant-time queries.
    std::vector<std::uint8_t> allowed(g.cells.size(),1);
    if (!guidance_) for (int v=0;v<g.height;++v) for (int u=0;u<g.width;++u)
      allowed[std::size_t(v)*g.width+u]=laneDistance({g.x0+(u+.5)*g.resolution,g.y0+(v+.5)*g.resolution},lane)
        <=c.lane_width/2+c.departure;
    if(guidance_ && pair_valid)for(int v=0;v<g.height;++v)for(int u=0;u<g.width;++u){
      const double x=g.x0+(u+.5)*g.resolution,y=g.y0+(v+.5)*g.resolution;
      if(x<bands.front().x)continue;
      const Band* near=&bands.front();for(const auto& band:bands)if(std::abs(band.x-x)<std::abs(near->x-x))near=&band;
      allowed[std::size_t(v)*g.width+u]=std::abs(near->x-x)<=.10 && y>near->lo && y<near->hi;
    }
    const auto footprint=[&](const Pose& p) {
      const double cs=std::cos(p.yaw),sn=std::sin(p.yaw),pad=g.resolution*.71;
      const double extent=std::hypot(c.rear+pad,half+pad)+c.front;
      const int u0=std::max(0,int(std::floor((p.x-extent-g.x0)/g.resolution)));
      const int u1=std::min(g.width-1,int(std::ceil((p.x+extent-g.x0)/g.resolution)));
      const int v0=int(std::floor((p.y-extent-g.y0)/g.resolution));
      const int v1=int(std::ceil((p.y+extent-g.y0)/g.resolution));
      // Sample all footprint cells, including subcell intersection padding.
      for (int v=v0;v<=v1;++v) for (int u=u0;u<=u1;++u) {
        const double x=g.x0+(u+.5)*g.resolution,y=g.y0+(v+.5)*g.resolution;
        const double dx=x-p.x,dy=y-p.y,longitudinal=dx*cs+dy*sn,lateral=-dx*sn+dy*cs;
        if (longitudinal < -c.rear-pad || longitudinal>c.front+c.margin+pad || std::abs(lateral)>half+pad) continue;
        if (x<0) continue; // No rear sensor/history: Rear terrain remains unverified.
        if (x<=c.front && std::abs(y)<=c.width/2) continue; // Current self-occupied area only.
        if (v<0 || v>=g.height || blockedTerrain(g.cells[std::size_t(v)*g.width+u]) || !allowed[std::size_t(v)*g.width+u]) return false;
      }
      // Explicit forward/lateral bounds, even when the bounding loop was clipped.
      for (double a:{-c.rear,c.front+c.margin}) for (double b:{-half,half}) {
        const double x=p.x+a*cs-b*sn,y=p.y+a*sn+b*cs;
        if (x>=0 && (x>=g.x0+g.width*g.resolution || y<g.y0 || y>=g.y0+g.height*g.resolution)) return false;
      }
      return true;
    };
    // Keep the preferred lane direction, prior direction, and low-density valley directions.
    std::vector<double> angles;
    for (int b=0;b<61;) {
      if (out.masked[b]) {++b;continue;}
      const int first=b;while (b+1<61 && !out.masked[b+1])++b;
      const double lo=(-75+2.5*first)*pi/180,hi=(-75+2.5*b)*pi/180;
      // Narrow valley: center. Wide valley: two inset borders and in-valley references.
      if (hi-lo<=40*pi/180) angles.push_back((lo+hi)/2);
      else {
        const double a=lo+20*pi/180,z=hi-20*pi/180;
        angles.push_back(a);angles.push_back(z);
        for (double ref:{target_angle,0.,previous_angle_}) if (ref>=a && ref<=z) angles.push_back(ref);
      }
      ++b;
    }
    // Additional in-valley directions let a long Ackermann body form a lane-change curve.
    for(int b=6;b<61;b+=4)if(!out.masked[b])angles.push_back((-75+2.5*b)*pi/180);
    std::sort(angles.begin(),angles.end(),[&](double a,double b){const double da=std::abs(a-target_angle),db=std::abs(b-target_angle);return std::abs(da-db)<1e-9 ? a<b : da<db;});
    std::vector<double> targets;
    for (double a:angles) {
      bool duplicate=false; for (double b:targets) if (std::abs(a-b)<.04) duplicate=true;
      if (!duplicate) targets.push_back(a);
      if (targets.size()>=17) break;
    }
    double best_score=1e9;
    const double dt=c.step/c.speed;
    for (double angle:targets) for (int profile=0;profile<3;++profile) {
      Candidate candidate;candidate.direction=angle; Pose p;p.steer=c.current_steer; candidate.path.push_back(p);
      const Point goal{c.reference_distance*std::cos(angle),c.reference_distance*std::sin(angle)};
      for (double s=c.step;s<=c.horizon+1e-8;s+=c.step) {
        double desired;
        if (profile==0) {
          const double dx=goal.x-p.x,dy=goal.y-p.y,d2=dx*dx+dy*dy;
          desired=std::atan2(2*c.wheelbase*(-std::sin(p.yaw)*dx+std::cos(p.yaw)*dy),std::max(.1,d2));
        } else {
          const double gain=profile==1 ? 2.0 : 3.0;
          const double initial=std::atan(2*c.wheelbase*std::sin(angle)/c.reference_distance)*gain;
          desired=s<c.horizon/3 ? initial : (s<2*c.horizon/3 ? -initial : 0);
        }
        desired=std::clamp(desired,-c.max_steer,c.max_steer);
        p.steer+=std::clamp(desired-p.steer,-c.steer_rate*dt,c.steer_rate*dt);
        const double turn=c.step*std::tan(p.steer)/c.wheelbase;
        p.x+=c.step*std::cos(p.yaw+turn/2);p.y+=c.step*std::sin(p.yaw+turn/2);p.yaw=wrap(p.yaw+turn);
        if (std::abs(p.yaw)>=pi/2 || !footprint(p)) break;
        candidate.path.push_back(p);candidate.length=s;
      }
      const double usable=std::max(0.,candidate.length-c.step);
      candidate.valid=usable>=.25 && candidate.path.size()>2;
      if (candidate.valid) {
        const auto last=candidate.path.back();
        candidate.score=4*(c.horizon-candidate.length)+1.5*std::abs(wrap(angle-target_angle))+
          .4*std::abs(angle)+.6*std::abs(wrap(angle-previous_angle_))+.3*std::abs(last.yaw-target_angle);
        if (candidate.score<best_score) {
          best_score=candidate.score;out.path=candidate.path;out.length=usable;
          out.steer=candidate.path[1].steer;
        }
      }
      out.candidates.push_back(std::move(candidate));
    }
    if (out.path.empty()) return out;
    const auto last=out.path.back();previous_angle_=std::atan2(last.y,last.x);
    // v*delay+v^2/(2a) <= checked remaining path (unknown allowed); nominal speed is an upper bound.
    out.speed=std::min(c.speed,-c.deceleration*c.delay+
      std::sqrt(c.deceleration*c.deceleration*c.delay*c.delay+2*c.deceleration*out.length));
    double curvature=0;for (auto p:out.path)curvature=std::max(curvature,std::abs(std::tan(p.steer)/c.wheelbase));
    out.speed=std::min(out.speed,std::sqrt(.4/std::max(.01,curvature)));
    out.status=guidance_ ? "GUIDANCE: terrain corridor" : (std::abs(previous_angle_-target_angle)>.1 ? "AVOID: lane direction preferred" : "LANE: route (unknown allowed)");
    return out;
  }
};
}
