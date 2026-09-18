#include "point_cloud/temporal_filter.hpp"
#include "point_cloud/obstacle_clusters.hpp"

#include <iostream>
#include <string>

namespace
{
void check(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}
void patch(point_cloud::Cloud & cloud, float x, float z=0.12f)
{
  for (int i=0; i<40; ++i) {
    cloud.xyz.insert(cloud.xyz.end(), {x+0.008f*(i%8), -0.02f+0.008f*(i/8), z});
    ++cloud.width; ++cloud.valid_points;
  }
  cloud.height=1;
}
point_cloud::Cloud point(float x=1, float z=0.12f)
{
  return {1,1,1,{x,0,z}};
}
}
int main()
{
  using namespace point_cloud;
  TemporalFilter filter;
  TemporalOptions options;
  options.enabled = true;
  for (int frame=0; frame<12; ++frame) {
    Cloud c;
    patch(c,1 + 0.001f*(frame%2));
    if (frame%2==0) {patch(c,1.12f);} // blinking extension connected by XY clustering
    const auto original = c;
    filter.apply(c,1+frame*0.02,options,0.03,2);
    check(c.width==original.width && c.height==original.height,"Organized layout changed");
    const auto clusters=obstacleClusters(c,{});
    if (frame<3) {check(c.valid_points==0 && clusters.accepted==0,"Insufficient frames admitted");}
    else {
      check(c.valid_points==40 && clusters.accepted==1,"Stable cone/blinking clutter not separated");
      for (std::size_t j=0;j<120;++j) {check(c.xyz[j]==original.xyz[j],"Output points were averaged/replayed");}
    }
  }
  Cloud absent;
  filter.apply(absent,1.24,options,0.03,2);
  check(absent.xyz.empty() && absent.valid_points==0,"Vanished points ghosted");
  auto delayed=point();
  filter.apply(delayed,2.0,options,0.03,2);
  check(delayed.valid_points==0,"Expired observations retained");

  // Even persistent low clutter must fail the new spatial core rule.
  filter.clear();
  for (int frame=0;frame<6;++frame) {
    Cloud c; patch(c,1); patch(c,1.12f,0.045f);
    filter.apply(c,2.2+frame*0.02,options,0.03,2);
    if (frame>=3) {
      check(c.valid_points==80,"Persistent low clutter should reach spatial classifier");
      const auto clean=obstacleClusters(c,{});
      check(clean.accepted==1 && clean.points.valid_points==40,"Persistent low tail survived core classifier");
    }
  }

  // Thousands of points in a frame cannot count as several observations.
  filter.clear(); Cloud dense;
  for (int i=0;i<1000;++i) {dense.xyz.insert(dense.xyz.end(),{1,0,0.12f});}
  dense.width=1000; dense.height=1; dense.valid_points=1000;
  filter.apply(dense,3.0,options,0.03,2);
  check(dense.valid_points==0,"Point density counted as frame persistence");
  for (int i=0;i<5;++i) {
    auto duplicate=point(); filter.apply(duplicate,3.0,options,0.03,2);
    check(duplicate.valid_points==0,"Duplicate stamp admitted");
  }
  auto next=point(); filter.apply(next,3.02,options,0.03,2);
  check(next.valid_points==0,"Duplicate stamps built evidence");

  // Small jitter across a 4cm voxel boundary is matched in 3D.
  filter.clear();
  for (int i=0;i<4;++i) {
    auto c=point(i%2==0 ? 0.999f : 1.001f);
    filter.apply(c,4+i*0.02,options,0.03,2);
    check(c.valid_points==(i==3 ? 1U : 0U),"Voxel boundary jitter not tolerated");
  }
  auto different_height=point(1,0.3f);
  filter.apply(different_height,4.08,options,0.03,2);
  check(different_height.valid_points==0,"XY-only match ignored Z displacement");
  auto moved=point(1.3f);
  filter.apply(moved,4.10,options,0.03,2);
  check(moved.valid_points==0,"Unseen location inherited persistence");

  // Sliding-window eviction, even while old frames still satisfy max_age_sec.
  filter.clear();
  for (int i=0;i<4;++i) {auto c=point(); filter.apply(c,5+i*0.01,options,0.03,2);}
  for (int i=4;i<9;++i) {Cloud c; filter.apply(c,5+i*0.01,options,0.03,2);}
  auto returned=point(); filter.apply(returned,5.09,options,0.03,2);
  check(returned.valid_points==0,"Frame window did not expire evidence");

  // Toggle, settings changes, clear/reconnect and timestamp rewind start fresh.
  options.enabled=false;
  auto bypass=point(); filter.apply(bypass,6,options,0.03,2);
  check(bypass.valid_points==1,"Disabled filter changed input");
  options.enabled=true;
  auto enabled=point(); filter.apply(enabled,6.02,options,0.03,2);
  check(enabled.valid_points==0,"Re-enable inherited observations");
  options.min_hits=1;
  auto immediate=point(); filter.apply(immediate,6.04,options,0.03,2);
  check(immediate.valid_points==1,"min_hits=1 failed");
  options.min_hits=2;
  auto changed=point(); filter.apply(changed,6.06,options,0.03,2);
  check(changed.valid_points==0,"Config change retained history");
  auto confirmed=point(); filter.apply(confirmed,6.08,options,0.03,2);
  check(confirmed.valid_points==1,"Second fresh frame not admitted");
  auto rewind=point(); filter.apply(rewind,1,options,0.03,2);
  check(rewind.valid_points==0,"Rewound clock retained evidence");
  filter.clear();
  auto reset=point(); filter.apply(reset,1.02,options,0.03,2);
  check(reset.valid_points==0,"Explicit reset retained evidence");

  options.min_hits=1;
  auto low=point(1,0.01f); filter.apply(low,7,options,0.03,2);
  check(low.valid_points==0,"Low floor admitted as temporal evidence");
  auto invalid=point(); invalid.xyz[1]=std::numeric_limits<float>::quiet_NaN();
  filter.apply(invalid,7.02,options,0.03,2);
  check(invalid.valid_points==0 && std::isnan(invalid.xyz[0]) && std::isnan(invalid.xyz[2]),"NaN handling failed");
  for (int i=0;i<6;++i) {
    TemporalOptions bad;
    switch(i) {
      case 0: bad.voxel_size_m=0; break;
      case 1: bad.match_distance_m=0.2; break;
      case 2: bad.window_frames=0; break;
      case 3: bad.min_hits=6; break;
      case 4: bad.max_age_sec=0; break;
      case 5: bad.match_distance_m=std::numeric_limits<double>::quiet_NaN(); break;
    }
    bool rejected=false;
    try {validateTemporal(bad);} catch (const std::invalid_argument &) {rejected=true;}
    check(rejected,"Invalid temporal setting accepted");
  }
  std::cout << "Temporal tests passed: stable cone, blinking extension, frame votes, 3D jitter, no ghosts, expiry, resets, toggles\n";
}
