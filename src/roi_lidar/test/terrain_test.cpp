#include "roi_lidar/terrain.hpp"
#include <iostream>
using namespace roi_lidar;
static void check(bool ok,const char * text) {if (!ok) {throw std::runtime_error(text);}}
int main()
{
  Options o; ViewOptions v; TerrainOptions t;
  auto scan=emptyScan(o);
  auto grid=terrainGrid(scan,o,v,t);
  check(grid.width==80 && grid.height==30,"Metric terrain shape incorrect");
  for (const auto cell:grid.cells) {check(cell==-1,"Unobserved cell became ground");}
  const auto index=[&](double x,double y) {
    return static_cast<std::size_t>(std::floor((y+v.width_m/2)/t.cell_m))*grid.width+
      static_cast<std::size_t>(std::floor(x/t.cell_m));
  };
  scan.surface_points={{1.01,.01,0.01},{1.02,.02,-.01},
    {2.01,.01,.2},{2.02,.02,.3},{3.01,.01,-.12},{3.02,.02,-.13}};
  grid=terrainGrid(scan,o,v,t);
  check(grid.cells[index(1.01,.01)]==0,"Measured floor lost");
  check(grid.cells[index(2.01,.01)]==100,"Positive obstacle lost");
  check(grid.cells[index(3.01,.01)]==100,"Below-plane hazard called floor");
  check(grid.cells[index(.51,.01)]==-1,"Ray was filled as free without ground evidence");
  check(grid.cells[index(2.51,.01)]==-1,"Unknown behind obstacle filled");
  scan.surface_points.push_back({1.03,.03,.2});
  grid=terrainGrid(scan,o,v,t);
  check(grid.cells[index(1.01,.01)]==-1,"Single hazard outlier did not veto green floor");
  scan.surface_points.push_back({1.03,.03,.25});
  grid=terrainGrid(scan,o,v,t);
  check(grid.cells[index(1.01,.01)]==100,"Obstacle did not dominate ground");
  scan.surface_points={{.01,.01,2},{-1,0,0},{1,2,0},{4,0,0}};
  grid=terrainGrid(scan,o,v,t);
  check(grid.ground_cells==0 && grid.obstacle_cells==0,"Height/XY limits ignored");
  grid=terrainGrid(emptyScan(o),o,v,t);
  check(grid.ground_cells==0 && grid.obstacle_cells==0,"History retained after empty input");
  bool rejected=false;
  try {auto bad=t; bad.cell_m=0; terrainGrid(scan,o,v,bad);} catch (const std::invalid_argument &) {rejected=true;}
  check(rejected,"Invalid cell size accepted");
  std::cout<<"Terrain tests passed: ground, obstacles, depression, unknown, conflict, bounds, no history\n";
}
