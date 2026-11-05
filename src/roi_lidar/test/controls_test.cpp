#include "roi_lidar/controls.hpp"
#include <map>
#include <stdexcept>
#include <iostream>
int main() {
  using namespace roi_lidar;
  std::map<std::string,double> values{{"height.max_m",1},{"ground.distance_m",.05},{"ground.distance_per_meter",.01},
    {"ground.max_distance_m",.12},{"ground.enabled",1},{"terrain.below_ground_m",.05},{"terrain.cell_m",.05},
    {"terrain.min_ground_samples",2},{"terrain.min_obstacle_samples",2},{"points.pixel_stride",2},
    {"preview.show_terrain",1},{"preview.show_projected_points",1},{"preview.terrain_alpha",.4},
    {"preview.point_radius",1},{"preview.width_m",1.5},{"preview.forward_m",4},{"camera.fps",90},
    {"depth.confidence_threshold",200},{"depth.left_right_check",1},{"depth.left_right_threshold",10},
    {"depth.subpixel",1},{"depth.ir_dot_projector_intensity",.7},{"depth.ir_flood_light_intensity",0}};
  Controls c; auto read=[&](const char * n) {return values.at(n);};
  if (!c.poll(read,false).empty()) {throw std::runtime_error("Initial GUI modified parameters");}
  cv::setTrackbarPos("Max obstacle height (cm, +1)","ROI LiDAR - live controls",49);
  auto changes=c.poll(read,false);
  if (changes.size()!=1 || changes[0].name!="height.max_m" || std::abs(changes[0].value-.5)>1e-9) {throw std::runtime_error("Height slider units/apply failed");}
  values["height.max_m"]=.5; c.poll(read,false);
  cv::setTrackbarPos("Confidence threshold","ROI LiDAR - depth settings (A apply)",120);
  if (!c.poll(read,false).empty()) {throw std::runtime_error("Camera changed before A");}
  changes=c.poll(read,true);
  if (changes.size()!=1 || changes[0].name!="depth.confidence_threshold" || changes[0].value!=120) {throw std::runtime_error("Staged camera change failed");}
  values["depth.confidence_threshold"]=120; c.poll(read,false);
  values["height.max_m"]=.75;
  if (!c.poll(read,false).empty() || cv::getTrackbarPos("Max obstacle height (cm, +1)","ROI LiDAR - live controls")!=74) {throw std::runtime_error("External parameter sync failed");}
  cv::waitKey(1); cv::destroyAllWindows();
  std::cout<<"GUI tests passed: live height, units, staged camera apply, external parameter synchronization\n";
}
