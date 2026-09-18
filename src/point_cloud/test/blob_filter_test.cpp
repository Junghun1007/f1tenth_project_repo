#include "point_cloud/blob_filter.hpp"

#include <opencv2/core/utility.hpp>
#include <iostream>
#include <set>
#include <string>

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}
void add(point_cloud::ClusterResult & c,int x,int y,float z=0.12f,std::uint32_t id=1)
{
  c.points.xyz.insert(c.points.xyz.end(),{1.005f+x*0.01f,-0.295f+y*0.01f,z});
  c.ids.push_back(id); ++c.points.width; ++c.points.valid_points; c.points.height=1; c.accepted=1;
}
void block(point_cloud::ClusterResult & c,int offset=0,float z=0.12f)
{
  for (int y=0;y<8;++y) {for (int x=offset;x<offset+8;++x) {add(c,x,y,z);}}
}
}
int main()
{
  cv::setNumThreads(1);  // Keep CPU regression tests deterministic.
  using namespace point_cloud;
  Cloud projected;
  for(int i=0;i<40;++i) {
    projected.xyz.insert(projected.xyz.end(),{1.0f+0.008f*(i%8),-0.02f+0.008f*(i/8),0.12f});
    ++projected.width; ++projected.valid_points; projected.height=1;
  }
  check(filterBlobs(obstacleClusters(projected,{}),{},{}).clusters.accepted==1,
    "Default spatial cluster does not survive default shape settings");
  ClusterResult cone; block(cone);
  const auto clean=filterBlobs(cone,{},{}).clusters;
  check(clean.accepted==1 && clean.points.valid_points==64,"Solid blob rejected");
  check(clean.points.xyz==cone.points.xyz,"Original XYZ coordinates not preserved");

  ClusterResult tail=cone;
  for (int x=8;x<40;++x) {add(tail,x,3);}
  const auto trimmed=filterBlobs(tail,{},{}).clusters;
  check(trimmed.accepted==1 && trimmed.points.valid_points>=64,"Tail caused whole cone to be rejected");
  for (std::size_t j=0;j<trimmed.points.xyz.size();j+=3) {
    check(trimmed.points.xyz[j]<1.115f,"Long thin tail survived opening");
  }
  ClusterResult bridge=cone; block(bridge,20);
  for (int x=8;x<20;++x) {add(bridge,x,3);}
  const auto split=filterBlobs(bridge,{},{}).clusters;
  check(split.accepted==2 && std::set<std::uint32_t>(split.ids.begin(),split.ids.end()).size()==2,
    "Thin bridge did not split into two blobs");
  check(split.points.valid_points>=128,"Split erased blob bodies");

  ClusterResult line;
  for (int x=0;x<40;++x) {add(line,x,3);}
  check(filterBlobs(line,{},{}).clusters.accepted==0,"Thin line accepted");
  ClusterResult diagonal;
  for (int x=0;x<40;++x) {add(diagonal,x,x);}
  auto no_open=BlobOptions{}; no_open.opening_radius_cells=0; no_open.closing_radius_cells=0;
  check(filterBlobs(diagonal,no_open,{}).clusters.accepted==0,"Diagonal line evaded thickness rule");

  ClusterResult ribbon;
  for (int y=0;y<4;++y) {for (int x=0;x<40;++x) {add(ribbon,x,y);}}
  check(filterBlobs(ribbon,{},{}).clusters.accepted==0,"Long ribbon evaded aspect ratio");
  auto broad=BlobOptions{}; broad.max_aspect_ratio=12;
  check(filterBlobs(ribbon,broad,{}).clusters.accepted==1,"Aspect ratio setting ignored");

  ClusterResult ring;
  for (int y=0;y<12;++y) {
    for (int x=0;x<12;++x) {if(x==0 || x==11 || y==0 || y==11) {add(ring,x,y);}}
  }
  check(filterBlobs(ring,no_open,{}).clusters.accepted==0,"Empty outline passed fill ratio");
  no_open.min_fill_ratio=0.2;
  check(filterBlobs(ring,no_open,{}).clusters.accepted==1,"Fill-ratio adjustment ignored");

  ClusterResult hole;
  for (int y=0;y<8;++y) {for (int x=0;x<8;++x) {if(x!=4) {add(hole,x,y);}}}
  const auto filled=filterBlobs(hole,{},{}).clusters;
  check(filled.accepted==1 && filled.points.xyz==hole.points.xyz,"Closing fabricated points or failed to join tiny gap");

  ClusterResult small;
  for (int i=0;i<100;++i) {add(small,0,0);}
  auto area=BlobOptions{}; area.closing_radius_cells=0; area.opening_radius_cells=0;
  area.min_thickness_m=0.005;
  check(filterBlobs(small,area,{}).clusters.accepted==0,"Point density inflated occupied area");

  ClusterResult unsupported=cone; block(unsupported,20,0.045f);
  for (int x=8;x<20;++x) {add(unsupported,x,3);}
  check(filterBlobs(unsupported,{},{}).clusters.accepted==1,"Detached low blob borrowed high evidence");
  auto strict=ClusterOptions{}; strict.min_points=65;
  check(filterBlobs(cone,{},strict).clusters.accepted==0,"Post-split minimum point count ignored");

  auto disabled=BlobOptions{}; disabled.enabled=false;
  const auto bypass=filterBlobs(tail,disabled,{}).clusters;
  check(bypass.points.xyz==tail.points.xyz && bypass.ids==tail.ids,"Disabled filter changed data");
  check(filterBlobs(ClusterResult{},{},{}).clusters.points.xyz.empty(),"Empty frame retains old points");
  auto huge=cone; huge.points.xyz[0]=100000;
  const auto bounded=filterBlobs(huge,{},{});
  check(bounded.grid_limit_rejections==1 && bounded.clusters.accepted==0,"Unbounded raster allocation");
  for (int i=0;i<6;++i) {
    BlobOptions bad;
    switch(i) {
      case 0: bad.cell_size_m=0; break;
      case 1: bad.opening_radius_cells=6; break;
      case 2: bad.min_area_m2=-1; break;
      case 3: bad.min_fill_ratio=1.1; break;
      case 4: bad.max_aspect_ratio=0; break;
      case 5: bad.min_thickness_m=std::numeric_limits<double>::quiet_NaN(); break;
    }
    bool rejected=false;
    try {filterBlobs(cone,bad,{});} catch(const std::invalid_argument &) {rejected=true;}
    check(rejected,"Invalid blob parameters accepted");
  }
  std::cout << "Blob tests passed: solid bodies, attached tails, split bridges, diagonal/ribbon rejection, occupancy, no fake points, height revalidation, bounded memory\n";
}
