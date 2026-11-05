#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace roi_lidar
{
class Controls
{
public:
  enum class Type {Real, Integer, Boolean};
  struct Entry
  {
    const char * name;
    const char * unit;
    Type type;
    bool depth;
    std::string draft{};
    double observed{0};
    bool pending{false};
  };
  struct Change {std::string name; double value; Type type;};
  static constexpr const char * windowName() {return "ROI LiDAR - parameters";}
private:
  std::vector<Entry> entries_{
    {"height.max_m","m",Type::Real,false},
    {"ground.distance_m","m",Type::Real,false},
    {"ground.distance_per_meter","m per m",Type::Real,false},
    {"ground.max_distance_m","m",Type::Real,false},
    {"ground.enabled","0=off / 1=on",Type::Boolean,false},
    {"terrain.below_ground_m","m",Type::Real,false},
    {"terrain.cell_m","m",Type::Real,false},
    {"terrain.min_ground_samples","samples per cell",Type::Integer,false},
    {"terrain.min_obstacle_samples","samples per cell",Type::Integer,false},
    {"terrain.vehicle_width_m","m (reference marker)",Type::Real,false},
    {"points.pixel_stride","pixels",Type::Integer,false},
    {"preview.show_terrain","0=off / 1=on",Type::Boolean,false},
    {"preview.show_projected_points","0=off / 1=on",Type::Boolean,false},
    {"preview.terrain_alpha","0..1",Type::Real,false},
    {"preview.point_radius","pixels",Type::Integer,false},
    {"preview.width_m","m",Type::Real,false},
    {"preview.forward_m","m",Type::Real,false},
    {"camera.fps","frames per second",Type::Real,true},
    {"depth.confidence_threshold","0..255",Type::Integer,true},
    {"depth.left_right_check","0=off / 1=on",Type::Boolean,true},
    {"depth.left_right_threshold","0..128",Type::Integer,true},
    {"depth.subpixel","0=off / 1=on",Type::Boolean,true},
    {"depth.ir_dot_projector_intensity","0..1",Type::Real,true},
    {"depth.ir_flood_light_intensity","0..1",Type::Real,true}
  };
  bool ready_{false}, apply_requested_{false}, reload_requested_{false}, select_all_{false};
  int active_{-1};
  std::string message_;
  static cv::Rect field(std::size_t index)
  {
    return {322+int(index/12)*520,104+int(index%12)*40,174,30};
  }
  static cv::Rect applyButton() {return {20,598,490,44};}
  static cv::Rect reloadButton() {return {530,598,490,44};}
  static std::string format(double value)
  {
    std::ostringstream text; text<<std::setprecision(10)<<value; return text.str();
  }
  void restore(Entry & entry) {entry.draft=format(entry.observed); entry.pending=false;}
  static double parse(const Entry & entry)
  {
    std::size_t consumed=0;
    const double value=std::stod(entry.draft,&consumed);
    if (consumed!=entry.draft.size() || !std::isfinite(value)) {
      throw std::invalid_argument("enter a finite number");
    }
    if (entry.type==Type::Boolean && value!=0 && value!=1) {
      throw std::invalid_argument("boolean must be 0 or 1");
    }
    if (entry.type==Type::Integer && (std::floor(value)!=value ||
      value<std::numeric_limits<int>::min() || value>std::numeric_limits<int>::max())) {
      throw std::invalid_argument("enter an integer");
    }
    return value;
  }
  static void mouse(int event,int x,int y,int,void * data)
  {
    if (event!=cv::EVENT_LBUTTONDOWN) {return;}
    auto & self=*static_cast<Controls *>(data);
    for (std::size_t i=0;i<self.entries_.size();++i) {
      if (field(i).contains(cv::Point(x,y))) {
        self.active_=static_cast<int>(i); self.select_all_=true; return;
      }
    }
    self.active_=-1;
    if (applyButton().contains(cv::Point(x,y))) {self.apply_requested_=true;}
    if (reloadButton().contains(cv::Point(x,y))) {self.reload_requested_=true;}
  }
  void keyboard(int key)
  {
    if (key==13 || key==10 || key=='a' || key=='A') {apply_requested_=true; active_=-1; return;}
    if (key==9) {
      active_=active_<0 ? 0 : (active_+1)%static_cast<int>(entries_.size());
      select_all_=true; return;
    }
    if (active_<0 || key<0) {return;}
    auto & entry=entries_[active_];
    if (key==27) {restore(entry); active_=-1; return;}
    if (key==8 || key==127 || key==0xff08) {
      if (select_all_) {entry.draft.clear();}
      else if (!entry.draft.empty()) {entry.draft.pop_back();}
    } else if ((key>='0' && key<='9') || key=='.' || key=='-' || key=='+' || key=='e' || key=='E') {
      if (select_all_) {entry.draft.clear();}
      if (entry.draft.size()<28) {entry.draft.push_back(static_cast<char>(key));}
    } else {return;}
    select_all_=false; entry.pending=true;
  }
  void draw()
  {
    cv::Mat canvas(690,1040,CV_8UC3,cv::Scalar(242,242,242));
    const auto text=[&](const std::string & value,cv::Point at,double scale,cv::Scalar color) {
      cv::putText(canvas,value,at,cv::FONT_HERSHEY_SIMPLEX,scale,color,1,cv::LINE_AA);
    };
    text("ROI PARAMETERS - numeric input",{20,30},.72,{35,35,35});
    text("Click a value and type. Enter / A / Apply: apply all edits together. Tab: next field.",{20,56},.46,{55,55,55});
    text("* Camera setting: restarts streaming on Apply. Units match ROS/YAML. Booleans: 0 or 1.",{20,80},.44,{55,55,55});
    for (std::size_t i=0;i<entries_.size();++i) {
      const auto & entry=entries_[i];
      const auto box=field(i);
      const int left=20+int(i/12)*520;
      text(std::string(entry.depth ? "* " : "")+entry.name,{left,box.y+13},.39,{35,35,35});
      text(entry.unit,{left,box.y+29},.32,{105,105,105});
      cv::rectangle(canvas,box,entry.pending ? cv::Scalar(220,245,255) : cv::Scalar(255,255,255),-1);
      cv::rectangle(canvas,box,active_==static_cast<int>(i) ? cv::Scalar(220,120,30) : cv::Scalar(165,165,165),
        active_==static_cast<int>(i) ? 2 : 1);
      if (active_==static_cast<int>(i) && select_all_) {
        cv::rectangle(canvas,box.tl()+cv::Point(3,3),box.br()-cv::Point(3,3),{220,170,100},-1);
      }
      const auto visible=entry.draft.size()>16 ? entry.draft.substr(entry.draft.size()-16) : entry.draft;
      text(visible+(active_==static_cast<int>(i) && !select_all_ ? "_" : ""),
        box.tl()+cv::Point(7,21),.51,{25,25,25});
    }
    cv::rectangle(canvas,applyButton(),{75,150,70},-1);
    cv::rectangle(canvas,reloadButton(),{175,175,175},-1);
    text("Apply edits (Enter / A)",{135,627},.60,{255,255,255});
    text("Reload current values / discard edits",{560,627},.53,{35,35,35});
    text(message_.substr(0,132),{20,669},.43,{35,65,190});
    cv::imshow(windowName(),canvas);
  }
public:
  void reset()
  {
    ready_=false; active_=-1; apply_requested_=reload_requested_=false;
    for (auto & entry:entries_) {entry.pending=false;}
  }
  bool editing() const {return active_>=0;}
  void message(const std::string & text) {message_=text;}
  void setDraft(const std::string & name,const std::string & value)
  {
    for (auto & entry:entries_) {
      if (name==entry.name) {entry.draft=value; entry.pending=true; return;}
    }
    throw std::invalid_argument("Unknown numeric control: "+name);
  }
  std::vector<Change> poll(const std::function<double(const char *)> & read,int key=-1)
  {
    if (!ready_) {
      cv::namedWindow(windowName(),cv::WINDOW_AUTOSIZE);
      cv::setMouseCallback(windowName(),mouse,this);
      for (auto & entry:entries_) {entry.observed=read(entry.name); restore(entry);}
      ready_=true;
    }
    for (std::size_t i=0;i<entries_.size();++i) {
      auto & entry=entries_[i];
      const double actual=read(entry.name);
      if (actual!=entry.observed) {
        entry.observed=actual; restore(entry);
        if (active_==static_cast<int>(i)) {select_all_=true;}
      }
    }
    keyboard(key);
    if (reload_requested_) {
      for (auto & entry:entries_) {restore(entry);}
      active_=-1; reload_requested_=false; apply_requested_=false;
      message_="Reloaded current applied values.";
    }
    std::vector<Change> changes;
    if (apply_requested_) {
      apply_requested_=false;
      try {
        for (auto & entry:entries_) {
          if (!entry.pending) {continue;}
          const double value=parse(entry);
          if (value!=entry.observed) {changes.push_back({entry.name,value,entry.type});}
          else {restore(entry);}
        }
        if (changes.empty()) {message_="No changed values to apply.";}
      } catch (const std::exception &) {
        changes.clear();
        message_="Invalid numeric entry. Use finite numbers, integers for counts, and 0/1 for booleans.";
      }
    }
    draw(); return changes;
  }
};
}
