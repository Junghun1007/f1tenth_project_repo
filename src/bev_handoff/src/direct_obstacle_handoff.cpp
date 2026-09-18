#include "bev_handoff/direct_obstacle_handoff.hpp"
#include <atomic>
#include <cmath>
#include <deque>
#include <mutex>
namespace bev_handoff
{
namespace
{
std::shared_ptr<const DirectDepthFrame> depth;
std::mutex mutex;
std::shared_ptr<const ObstacleReference> reference;
std::deque<std::shared_ptr<const ObstacleFrame>> history;
double seconds(const std_msgs::msg::Header & h)
{return double(h.stamp.sec)+double(h.stamp.nanosec)*1e-9;}
}
void publishObstacleReference(std::shared_ptr<const ObstacleReference> value)
{std::lock_guard<std::mutex> lock(mutex); reference=std::move(value); history.clear();}
std::shared_ptr<const ObstacleReference> obstacleReference()
{std::lock_guard<std::mutex> lock(mutex); return reference;}
std::string obstacleDeviceId()
{std::lock_guard<std::mutex> lock(mutex); return reference ? reference->device_id : std::string{};}
void publishDirectDepth(std::shared_ptr<const DirectDepthFrame> frame)
{std::atomic_store(&depth,std::move(frame));}
std::shared_ptr<const DirectDepthFrame> latestDirectDepth()
{return std::atomic_load(&depth);}
void clearObstacles()
{std::lock_guard<std::mutex> lock(mutex); history.clear();}
void publishObstacles(std::shared_ptr<const ObstacleFrame> frame)
{
  if (!frame) {return;}
  std::lock_guard<std::mutex> lock(mutex);
  if (!history.empty() && seconds(frame->header)<=seconds(history.back()->header)) {history.clear();}
  history.push_back(std::move(frame));
  while (history.size()>32) {history.pop_front();}
}
std::shared_ptr<const ObstacleFrame> matchingObstacles(
  const std_msgs::msg::Header & image,double max_delta_sec,double max_age_sec)
{
  const auto now=std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(mutex);
  std::shared_ptr<const ObstacleFrame> best;
  double closest=max_delta_sec;
  for (const auto & frame:history) {
    const double age=std::chrono::duration<double>(now-frame->captured_at).count();
    const double delta=std::abs(seconds(image)-seconds(frame->header));
    if (frame->header.frame_id==image.frame_id && age>=0 && age<=max_age_sec && delta<=closest) {
      best=frame; closest=delta;
    }
  }
  return best;
}
}
