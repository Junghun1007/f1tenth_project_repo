#pragma once
#include "roi_lidar/scan.hpp"
#include "roi_lidar/view.hpp"

namespace roi_lidar
{
struct TerrainOptions
{
  double cell_m{0.05}, below_ground_m{0.05};
  int min_ground_samples{2}, min_obstacle_samples{2};
  double vehicle_width_m{0.30}; // Reference only; no footprint inflation.
};
inline void validateTerrain(const TerrainOptions & t, const ViewOptions & view)
{
  if (!std::isfinite(t.cell_m) || t.cell_m<0.02 || t.cell_m>0.25 ||
    !std::isfinite(t.below_ground_m) || t.below_ground_m<0 || t.below_ground_m>0.30 ||
    !std::isfinite(t.vehicle_width_m) || t.vehicle_width_m<=0 || t.vehicle_width_m>3 ||
    t.min_ground_samples<1 || t.min_ground_samples>1000 ||
    t.min_obstacle_samples<1 || t.min_obstacle_samples>1000 ||
    std::ceil(view.forward_m/t.cell_m)*std::ceil(view.width_m/t.cell_m)>250000) {
    throw std::invalid_argument("terrain: cell=0.02..0.25m, below_ground=0..0.30m, samples=1..1000, grid<=250000 cells");
  }
}
struct Terrain
{
  // ROS grid order: X increases by column, Y increases by row.
  int width{0}, height{0};
  double cell_m{0.05}, y_min{0}, x_max{0}, y_max{0};
  std::vector<std::int8_t> cells; // -1 unknown, 0 observed ground, 100 obstacle/depression
  std::size_t ground_cells{0}, obstacle_cells{0};
};
inline Terrain terrainGrid(const Scan & scan, const Options & o,
  const ViewOptions & view, const TerrainOptions & t)
{
  validateView(view); validateTerrain(t,view);
  Terrain out;
  out.width=static_cast<int>(std::ceil(view.forward_m/t.cell_m));
  out.height=static_cast<int>(std::ceil(view.width_m/t.cell_m));
  out.cell_m=t.cell_m; out.y_min=-view.width_m/2;
  out.x_max=view.forward_m; out.y_max=view.width_m/2;
  const auto size=static_cast<std::size_t>(out.width)*out.height;
  out.cells.assign(size,-1);
  std::vector<int> ground(size,0), obstacle(size,0);
  for (const auto & p:scan.surface_points) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]) ||
      p[0]<0 || p[0]>=view.forward_m || p[1]<out.y_min || p[1]>=out.y_max ||
      p[2]>o.max_height) {continue;}
    const int x=static_cast<int>(p[0]/t.cell_m);
    const int y=static_cast<int>((p[1]-out.y_min)/t.cell_m);
    const auto index=static_cast<std::size_t>(y)*out.width+x;
    const double margin=groundMargin(p[0],p[1],o);
    if (p[2]<-t.below_ground_m || p[2]>margin) {++obstacle[index];}
    else {++ground[index];}
  }
  for (std::size_t i=0;i<size;++i) {
    if (obstacle[i]>=t.min_obstacle_samples) {out.cells[i]=100; ++out.obstacle_cells;}
    // Even one unsupported hazard sample vetoes a green cell.
    else if (obstacle[i]==0 && ground[i]>=t.min_ground_samples) {out.cells[i]=0; ++out.ground_cells;}
  }
  return out;
}
}
