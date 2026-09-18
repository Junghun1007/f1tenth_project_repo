#include "point_cloud/depth_source.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>

int main()
{
  using namespace point_cloud;
  validate(Config{});
  if (Config{}.fps != 50.0 || Config{}.ground.distance_m != 0.03) {
    throw std::runtime_error("Requested FPS/ground defaults lost");
  }
  for (const auto & name : {"400p", "480p", "720p", "800p"}) {
    Config c;
    c.resolution = name;
    validate(c);
  }
  const auto rejects = [](const std::function<void(Config &)> & change) {
      Config c;
      change(c);
      try {validate(c);} catch (const std::invalid_argument &) {return;}
      throw std::runtime_error("Invalid settings unexpectedly accepted");
    };
  rejects([](Config & c) {c.temporal.min_hits = c.temporal.window_frames + 1;});
  rejects([](Config & c) {c.cluster.support_height_m = c.cluster.min_height_m;});
  rejects([](Config & c) {c.cluster.base_radius_m = -1;});
  rejects([](Config & c) {c.blob.min_area_m2 = 0;});
  rejects([](Config & c) {c.cluster.min_points = 0;});
  rejects([](Config & c) {c.resolution = "1080p";});
  rejects([](Config & c) {c.dot_intensity = 1.1;});
  rejects([](Config & c) {c.flood_intensity = -0.1;});
  rejects([](Config & c) {c.fps = std::numeric_limits<double>::quiet_NaN();});
  rejects([](Config & c) {c.confidence = 256;});
  rejects([](Config & c) {c.subpixel = true; c.extended = true;});
  rejects([](Config & c) {c.subpixel = true; c.subpixel_bits = 5; c.median = "3x3";});
  rejects([](Config & c) {c.projection.max_depth_m = c.projection.min_depth_m = 2;});
  rejects([](Config & c) {c.projection.pixel_stride = 0;});
  rejects([](Config & c) {c.max_age_sec = 0;});
  rejects([](Config & c) {c.mode = "unknown";});
  rejects([](Config & c) {c.median = "9x9";});
  std::cout << "Configuration tests passed\n";
}
