#ifndef BEV_HANDOFF__DIRECT_BEV_HANDOFF_HPP_
#define BEV_HANDOFF__DIRECT_BEV_HANDOFF_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

namespace bev_handoff
{

// The cv::Mat owns the BEV allocation through OpenCV reference counting. A
// consumer may retain this shared frame without copying its pixel buffer.
struct DirectBevFrame
{
  cv::Mat bgr;
  std_msgs::msg::Header header;
  std::chrono::steady_clock::time_point bev_input_received_at;
  std::uint64_t source_generation{0U};
};

using DirectBevFrameCallback =
  std::function<void(std::shared_ptr<const DirectBevFrame>)>;

// Exactly one process-local consumer is supported. The callback must only
// enqueue/replace the latest frame; expensive inference must run elsewhere.
std::uint64_t registerDirectBevConsumer(DirectBevFrameCallback callback);
void unregisterDirectBevConsumer(std::uint64_t consumer_id);
bool deliverDirectBevFrame(std::shared_ptr<const DirectBevFrame> frame);

}  // namespace bev_handoff

#endif  // BEV_HANDOFF__DIRECT_BEV_HANDOFF_HPP_
