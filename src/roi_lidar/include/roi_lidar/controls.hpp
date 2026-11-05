#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <sstream>
#include <iomanip>
#include <utility>
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
    const char * label;
    double minimum, step;
    int maximum;
    Type type;
    bool depth;
    int position{-1};
    double observed{0};
    bool pending{false};
  };
  struct Change {std::string name; double value; Type type;};
private:
  std::vector<Entry> entries_{
    {"height.max_m","Max obstacle height (cm, +1)",.01,.01,299,Type::Real,false},
    {"ground.distance_m","Ground base (mm)",0,.001,150,Type::Real,false},
    {"ground.distance_per_meter","Ground growth (mm/m)",0,.001,30,Type::Real,false},
    {"ground.max_distance_m","Ground cap (mm)",0,.001,300,Type::Real,false},
    {"ground.enabled","Remove floor from XY points",0,1,1,Type::Boolean,false},
    {"terrain.below_ground_m","Below-ground tolerance (mm)",0,.001,300,Type::Real,false},
    {"terrain.cell_m","Cell size (cm, +2)",.02,.01,23,Type::Real,false},
    {"terrain.min_ground_samples","Floor samples per cell (+1)",1,1,19,Type::Integer,false},
    {"terrain.min_obstacle_samples","Hazard samples per cell (+1)",1,1,19,Type::Integer,false},
    {"points.pixel_stride","Pixel stride (+1)",1,1,15,Type::Integer,false},
    {"preview.show_terrain","Show terrain colors",0,1,1,Type::Boolean,false},
    {"preview.show_projected_points","Show all obstacle XY points",0,1,1,Type::Boolean,false},
    {"preview.terrain_alpha","Terrain opacity (%)",0,.01,100,Type::Real,false},
    {"preview.point_radius","Point radius (+1)",1,1,3,Type::Integer,false},
    {"preview.width_m","BEV width (cm, +10)",.10,.10,59,Type::Real,false},
    {"preview.forward_m","BEV forward (cm, +10)",.10,.10,99,Type::Real,false},
    {"camera.fps","FPS request (+1)",1,1,119,Type::Real,true},
    {"depth.confidence_threshold","Confidence threshold",0,1,255,Type::Integer,true},
    {"depth.left_right_check","Left-right check",0,1,1,Type::Boolean,true},
    {"depth.left_right_threshold","Left-right threshold",0,1,20,Type::Integer,true},
    {"depth.subpixel","Subpixel",0,1,1,Type::Boolean,true},
    {"depth.ir_dot_projector_intensity","IR dots (%)",0,.01,100,Type::Real,true},
    {"depth.ir_flood_light_intensity","IR flood (%)",0,.01,100,Type::Real,true}
  };
  bool ready_{false};
  std::string message_;
  const char * window(bool depth) const {return depth ? "ROI LiDAR - depth settings (A apply)" : "ROI LiDAR - live controls";}
public:
  void reset() {ready_=false; for (auto & e:entries_) {e.pending=false; e.position=-1;}}
  void message(const std::string & text) {message_=text;}
  std::vector<Change> poll(const std::function<double(const char *)> & read, bool apply_depth)
  {
    if (!ready_) {
      cv::namedWindow(window(false),cv::WINDOW_AUTOSIZE);
      cv::namedWindow(window(true),cv::WINDOW_AUTOSIZE);
      for (auto & e:entries_) {
        cv::createTrackbar(e.label,window(e.depth),nullptr,e.maximum);
        e.observed=read(e.name);
        e.position=std::clamp(static_cast<int>(std::lround((e.observed-e.minimum)/e.step)),0,e.maximum);
        cv::setTrackbarPos(e.label,window(e.depth),e.position);
      }
      ready_=true;
    }
    std::vector<Change> changes;
    for (auto & e:entries_) {
      int position=cv::getTrackbarPos(e.label,window(e.depth));
      const double actual=read(e.name);
      if (position!=e.position) {e.pending=true; e.position=position;}
      if (actual!=e.observed) {
        // External ros2 param changes take precedence over staged GUI values.
        e.observed=actual; e.pending=false;
        e.position=std::clamp(static_cast<int>(std::lround((actual-e.minimum)/e.step)),0,e.maximum);
        cv::setTrackbarPos(e.label,window(e.depth),e.position);
      }
      if (e.pending && (!e.depth || apply_depth)) {
        changes.push_back({e.name,e.minimum+e.position*e.step,e.type}); e.pending=false;
      }
    }
    for (const bool depth:{false,true}) {
      cv::Mat help(145,620,CV_8UC3,cv::Scalar(245,245,245));
      cv::putText(help,depth ? "Stage camera settings here; press A in any window to apply." :
        "Live processing changes; no camera restart.",{10,23},cv::FONT_HERSHEY_SIMPLEX,.43,{40,40,40},1);
      cv::putText(help,depth ? "A restarts streaming; the startup ground plane stays fixed." :
        "Ground base <= cap < max height. Invalid edits are rejected.",{10,46},cv::FONT_HERSHEY_SIMPLEX,.43,{40,40,40},1);
      cv::putText(help,"Save current values: ros2 param dump /roi_lidar",{10,69},cv::FONT_HERSHEY_SIMPLEX,.43,{40,40,40},1);
      std::ostringstream values;
      values<<std::fixed<<std::setprecision(3)<<"max Z="<<read("height.max_m")
        <<"m | ground="<<read("ground.distance_m")<<"m | cap="<<read("ground.max_distance_m")
        <<"m | cell="<<read("terrain.cell_m")<<"m";
      cv::putText(help,values.str(),{10,92},cv::FONT_HERSHEY_SIMPLEX,.40,{40,40,40},1);
      cv::putText(help,message_.substr(0,82),{10,117},cv::FONT_HERSHEY_SIMPLEX,.39,{30,50,190},1);
      cv::putText(help,"Slider position + offset in label = displayed unit value.",{10,138},cv::FONT_HERSHEY_SIMPLEX,.39,{70,70,70},1);
      cv::imshow(window(depth),help);
    }
    return changes;
  }
};
}
