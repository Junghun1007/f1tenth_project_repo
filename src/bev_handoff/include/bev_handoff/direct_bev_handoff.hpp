#ifndef BEV_HANDOFF__DIRECT_BEV_HANDOFF_HPP_
#define BEV_HANDOFF__DIRECT_BEV_HANDOFF_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

#include <opencv2/core.hpp>
#include <std_msgs/msg/header.hpp>

namespace bev_handoff
{

// device_owner keeps the CUDA BEV allocation alive while a consumer uses it.
// The optional cv::Mat independently owns a host copy when one is requested.
struct DirectBevFrame
{
  // Optional CPU image for local preview/capture. The autonomous inference
  // path uses device_bgr and does not require this download.
  cv::Mat bgr;
  const std::uint8_t * device_bgr{nullptr};
  std::size_t device_stride{0U};
  int width{0};
  int height{0};
  std::shared_ptr<const void> device_owner;
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
