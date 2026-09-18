#include "point_cloud/obstacle_clusters.hpp"

#include <iostream>
#include <set>
#include <string>

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}
void add(point_cloud::Cloud & c, float x, float y, float z)
{
  c.xyz.insert(c.xyz.end(), {x,y,z});
  ++c.valid_points; ++c.width; c.height = 1;
}
void patch(point_cloud::Cloud & c, float x, float y, float z, int n = 40)
{
  for (int i = 0; i < n; ++i) {add(c, x + 0.008f*(i%8), y + 0.008f*(i/8), z);}
}
}
int main()
{
  using namespace point_cloud;
  Cloud scene;
  patch(scene,1,-0.3f,0.12f); patch(scene,1,0.3f,0.14f);
  // Dense residual floor and a low bridge must neither be displayed nor merge objects.
  for (int i = 0; i < 100; ++i) {patch(scene,1,-0.3f + 0.006f*i,0.02f);}
  patch(scene,2,0,0.04f); // dense residual above candidate height but below support height
  patch(scene,2.5f,0,0.2f,8); // sparse high noise
  add(scene,0,0,std::numeric_limits<float>::quiet_NaN());
  add(scene,std::numeric_limits<float>::infinity(),0,0.2f);
  patch(scene,0.5f,0,2.5f); // outside candidate height
  const auto result = obstacleClusters(scene,{});
  check(result.accepted == 2 && result.ids.size() == 80, "Objects/floor/noise classification incorrect");
  check(result.points.width == 80 && result.points.height == 1 && result.points.valid_points == 80,
    "Compact output layout/count incorrect");
  check(std::set<std::uint32_t>(result.ids.begin(),result.ids.end()).size() == 2,"Object labels merged");
  for (std::size_t i = 0; i < 80*3; ++i) {
    check(result.points.xyz[i] == scene.xyz[i], "Original obstacle points/height not preserved");
  }
  Cloud outlier;
  patch(outlier,1,0,0.04f); add(outlier,1.02f,0.01f,0.3f);
  check(obstacleClusters(outlier,{}).accepted == 0,"One high outlier promoted floor to obstacle");
  patch(outlier,1,0,0.1f,8);
  check(obstacleClusters(outlier,{}).accepted == 0,"Insufficient high-point fraction accepted");
  auto relaxed = ClusterOptions{}; relaxed.min_support_ratio = 0.1;
  check(obstacleClusters(outlier,relaxed).accepted == 1,"Live support ratio setting not applied");

  Cloud tiny;
  for (int i = 0; i < 100; ++i) {add(tiny,1,0,0.1f);}
  check(obstacleClusters(tiny,{}).accepted == 0,"Zero-size speckle accepted");
  auto size = ClusterOptions{}; size.min_extent_m = 0;
  check(obstacleClusters(tiny,size).accepted == 1,"Extent setting ignored");

  Cloud separated;
  patch(separated,1,0,0.1f); patch(separated,1.2f,0,0.1f);
  check(obstacleClusters(separated,{}).accepted == 2,"Separated clusters merged");
  auto joined = ClusterOptions{}; joined.tolerance_m = 0.16;
  check(obstacleClusters(separated,joined).accepted == 1,"Tolerance change did not join clusters");

  Cloud boundary;
  patch(boundary,1,-0.026f,0.1f); // crosses zero and cell boundaries
  check(obstacleClusters(boundary,{}).accepted == 1,"Negative/cell boundary split obstacle");
  auto strict = ClusterOptions{}; strict.min_points = 41;
  check(obstacleClusters(boundary,strict).accepted == 0,"Minimum original point count ignored");
  const auto empty = obstacleClusters(Cloud{},{});
  check(empty.accepted == 0 && empty.points.xyz.empty() && empty.ids.empty(),"Old clusters persisted into empty frame");

  for (int i = 0; i < 8; ++i) {
    ClusterOptions invalid;
    switch(i) {
      case 0: invalid.cell_size_m = 0; break;
      case 1: invalid.tolerance_m = 1; break;
      case 2: invalid.min_height_m = -1; break;
      case 3: invalid.max_height_m = invalid.min_height_m; break;
      case 4: invalid.support_height_m = 3; break;
      case 5: invalid.min_support_ratio = 0; break;
      case 6: invalid.min_points = 0; break;
      case 7: invalid.min_extent_m = std::numeric_limits<double>::quiet_NaN(); break;
    }
    bool rejected = false;
    try {obstacleClusters(scene,invalid);} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected,"Invalid cluster option accepted");
  }
  std::cout << "Cluster tests passed: multiple obstacles, floor bridges, height support, sparse noise, extent, live settings, compact points, labels, empty frames\n";
}
