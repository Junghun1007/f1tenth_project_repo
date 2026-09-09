#ifndef LINE_DETACTOR__LANE_SMOOTHING_HPP_
#define LINE_DETACTOR__LANE_SMOOTHING_HPP_

namespace line_detactor
{

struct LaneSmoothingConfig
{
  bool enabled{false};
  double strength{8.0};
  bool correction_limit_enabled{true};
  double max_correction_px{2.0};
  double max_row_jump_px{4.0};
  int min_segment_rows{12};
};

// One unambiguous interior run per row; weight == 0 splits the segment.
// Shared CPU/CUDA layout. All coordinates use the original BEV resolution.
struct LaneRow
{
  float center{0.0F};
  float weight{0.0F};
  float shift{0.0F};
};

void validate_lane_smoothing(const LaneSmoothingConfig & config);
void smooth_lane_rows(LaneRow * rows, int height, const LaneSmoothingConfig & config);

}  // namespace line_detactor

#endif
