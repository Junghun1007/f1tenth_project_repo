#ifndef LINE_DETACTOR__TENSORRT_LANE_BACKEND_HPP_
#define LINE_DETACTOR__TENSORRT_LANE_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace line_detactor
{

struct LaneInferenceTiming
{
  std::uint64_t preprocessing_nanoseconds{0U};
  std::uint64_t execution_nanoseconds{0U};
  std::uint64_t postprocessing_nanoseconds{0U};
};

class TensorRtLaneBackend
{
public:
  TensorRtLaneBackend(
    const std::string & model_path,
    const std::string & engine_cache_path,
    int input_width,
    int input_height,
    std::size_t workspace_size_bytes,
    float mask_threshold,
    float overlay_alpha);
  ~TensorRtLaneBackend();

  TensorRtLaneBackend(const TensorRtLaneBackend &) = delete;
  TensorRtLaneBackend & operator=(const TensorRtLaneBackend &) = delete;
  TensorRtLaneBackend(TensorRtLaneBackend &&) noexcept;
  TensorRtLaneBackend & operator=(TensorRtLaneBackend &&) noexcept;

  LaneInferenceTiming infer_bgr(
    const std::uint8_t * bgr,
    std::size_t data_size,
    std::size_t source_stride);

  const std::uint8_t * preview_bgr_data() const noexcept;
  int input_width() const noexcept;
  int input_height() const noexcept;
  const std::string & engine_cache_path() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace line_detactor

#endif  // LINE_DETACTOR__TENSORRT_LANE_BACKEND_HPP_
