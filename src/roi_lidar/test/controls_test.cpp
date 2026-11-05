#include "roi_lidar/controls.hpp"
#include <map>
#include <stdexcept>
#include <iostream>
int main() {
  using namespace roi_lidar;
  std::map<std::string,double> values{{"height.max_m",1},{"ground.distance_m",.05},{"ground.distance_per_meter",.01},
    {"ground.max_distance_m",.12},{"ground.enabled",1},{"terrain.below_ground_m",.05},{"terrain.cell_m",.05},
    {"terrain.min_ground_samples",2},{"terrain.min_obstacle_samples",2},{"terrain.vehicle_width_m",.3},{"points.pixel_stride",2},
    {"preview.show_terrain",1},{"preview.show_projected_points",1},{"preview.terrain_alpha",.4},
    {"preview.point_radius",1},{"preview.width_m",1.5},{"preview.forward_m",4},{"camera.fps",90},
    {"depth.confidence_threshold",200},{"depth.left_right_check",1},{"depth.left_right_threshold",10},
    {"depth.subpixel",1},{"depth.ir_dot_projector_intensity",.7},{"depth.ir_flood_light_intensity",0}};
  Controls c; auto read=[&](const char * n) {return values.at(n);};
  if (!c.poll(read).empty()) {throw std::runtime_error("Initial GUI modified parameters");}
  c.poll(read,9); // Focus the first numeric field; typing replaces its selected value.
  c.poll(read,'0'); c.poll(read,'.'); c.poll(read,'5');
  if (!c.poll(read).empty()) {throw std::runtime_error("Draft applied before confirmation");}
  auto changes=c.poll(read,13);
  if (changes.size()!=1 || changes[0].name!="height.max_m" || std::abs(changes[0].value-.5)>1e-9) {throw std::runtime_error("Numeric height input/apply failed");}
  values["height.max_m"]=.5; c.poll(read);
  c.setDraft("depth.confidence_threshold","120");
  if (!c.poll(read).empty()) {throw std::runtime_error("Camera changed before confirmation");}
  changes=c.poll(read,'a');
  if (changes.size()!=1 || changes[0].name!="depth.confidence_threshold" || changes[0].value!=120) {throw std::runtime_error("Numeric camera change failed");}
  values["depth.confidence_threshold"]=120; c.poll(read);
  values["height.max_m"]=.75;
  if (!c.poll(read).empty()) {throw std::runtime_error("External change unexpectedly reapplied");}
  c.setDraft("height.max_m","0.75");
  if (!c.poll(read,13).empty()) {throw std::runtime_error("External parameter sync failed");}
  c.setDraft("ground.enabled","2");
  c.setDraft("terrain.cell_m","0.08");
  if (!c.poll(read,13).empty()) {throw std::runtime_error("Invalid boolean allowed a partial batch");}
  c.setDraft("ground.enabled","1");
  changes=c.poll(read,13);
  if (changes.size()!=1 || changes[0].name!="terrain.cell_m") {throw std::runtime_error("Corrected numeric batch failed");}
  cv::waitKey(1); cv::destroyAllWindows();
  std::cout<<"GUI tests passed: numeric input, explicit apply, invalid batch rejection, external parameter synchronization\n";
}
