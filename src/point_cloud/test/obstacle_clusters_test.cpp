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

  // Regression: a persistent 4.5cm sheet surrounds and bridges two cones.
  // Such points pass min_height and temporal persistence, but must not supply connectivity.
  Cloud water;
  for (int i=0; i<5; ++i) {patch(water,1,0,0.12f); patch(water,1.7f,0,0.12f);}
  for (int x=0;x<101;++x) {
    for (int y=0;y<81;++y) {add(water,0.9f+0.01f*x,-0.4f+0.01f*y,0.045f);}
  }
  add(water,1.4f,0,0.2f); // isolated high noise cannot recruit the surrounding floor
  const auto dry = obstacleClusters(water,{});
  check(dry.accepted==2 && dry.points.valid_points>=400,"Low sheet merged or erased cones");
  for (std::size_t j=0;j<dry.points.xyz.size();j+=3) {
    const double x=dry.points.xyz[j], y=dry.points.xyz[j+1];
    check((x>=0.98-1e-6 && x<=1.076+1e-6) || (x>=1.68-1e-6 && x<=1.776+1e-6),
      "Base expansion leaked along low sheet");
    check(y>=-0.02-1e-6 && y<=0.052+1e-6,"Base expansion exceeded allowed radius");
  }
  auto core_only = ClusterOptions{}; core_only.base_radius_m=0;
  const auto cores = obstacleClusters(water,core_only);
  check(cores.accepted==2 && cores.points.valid_points==400,"Core-only mode retained floor or high outlier");
  for (std::size_t j=2;j<cores.points.xyz.size();j+=3) {
    check(cores.points.xyz[j]>=core_only.support_height_m,"Low point attached in core-only mode");
  }
  Cloud shared_base;
  for (int i=0;i<5;++i) {patch(shared_base,1,0,0.12f); patch(shared_base,1.2f,0,0.12f);}
  for (int i=0;i<30;++i) {add(shared_base,1.0f+0.01f*i,0,0.045f);}
  auto wide_base = ClusterOptions{}; wide_base.base_radius_m=0.08;
  check(obstacleClusters(shared_base,wide_base).accepted==2,"Overlapping base regions merged independent cores");
  Cloud fake_core;
  patch(fake_core,1,0,0.045f);
  for (int i=0;i<5;++i) {add(fake_core,1.02f,0.01f,0.12f);}
  check(obstacleClusters(fake_core,{}).accepted==0,"Floor inflated high-core extent");

  for (int i = 0; i < 11; ++i) {
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
      case 8: invalid.support_height_m = invalid.min_height_m; break;
      case 9: invalid.base_radius_m = -0.01; break;
      case 10: invalid.base_radius_m = 0.21; break;
    }
    bool rejected = false;
    try {obstacleClusters(scene,invalid);} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected,"Invalid cluster option accepted");
  }
  std::cout << "Cluster tests passed: multiple obstacles, floor bridges, height support, sparse noise, extent, live settings, compact points, labels, empty frames\n";
}
