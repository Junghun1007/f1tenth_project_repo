#include "line_detactor/lane_smoothing.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace line_detactor
{
namespace
{

// Natural cubic smoothing spline on unit-spaced y samples:
// min sum(w_i * (f_i - x_i)^2) + lambda * integral(f''(y)^2 dy).
// Reinsch formulation: (R + lambda Q^T W^-1 Q) u = Q^T x,
// f = x - lambda W^-1 Q u. The system is symmetric pentadiagonal.
void smooth_segment(LaneRow * rows, const int count, const LaneSmoothingConfig & config)
{
  const int size = count - 2;
  std::vector<std::array<double, 3>> matrix(size, {0.0, 0.0, 0.0});
  std::vector<double> rhs(size, 0.0);
  constexpr double q[3] = {1.0, -2.0, 1.0};
  for (int i = 0; i < size; ++i) {
    rhs[i] = rows[i].center - 2.0 * rows[i + 1].center + rows[i + 2].center;
    for (int offset = 0; offset <= 2 && offset <= i; ++offset) {
      const int j = i - offset;
      double value = offset == 0 ? 2.0 / 3.0 : (offset == 1 ? 1.0 / 6.0 : 0.0);
      for (int k = i; k <= j + 2; ++k) {
        value += config.strength * q[k - i] * q[k - j] /
          std::max(0.05, static_cast<double>(rows[k].weight));
      }
      matrix[i][offset] = value;
    }
  }

  // Banded Cholesky, followed by forward/back substitution; O(count).
  for (int i = 0; i < size; ++i) {
    for (int j = std::max(0, i - 2); j <= i; ++j) {
      double value = matrix[i][i - j];
      for (int k = std::max(0, i - 2); k < j; ++k) {
        value -= matrix[i][i - k] * matrix[j][j - k];
      }
      if (i == j) {
        if (!std::isfinite(value) || value <= 0.0) {
          return;  // Keep the original segment if the solve is ill-conditioned.
        }
        matrix[i][0] = std::sqrt(value);
      } else {
        matrix[i][i - j] = value / matrix[j][0];
      }
    }
    for (int j = std::max(0, i - 2); j < i; ++j) {
      rhs[i] -= matrix[i][i - j] * rhs[j];
    }
    rhs[i] /= matrix[i][0];
  }
  for (int i = size - 1; i >= 0; --i) {
    for (int j = i + 1; j <= std::min(size - 1, i + 2); ++j) {
      rhs[i] -= matrix[j][j - i] * rhs[j];
    }
    rhs[i] /= matrix[i][0];
  }

  std::vector<double> shifts(count, 0.0);
  double maximum_shift = 0.0;
  for (int i = 0; i < count; ++i) {
    for (int j = std::max(0, i - 2); j <= std::min(size - 1, i); ++j) {
      shifts[i] -= config.strength * q[i - j] * rhs[j] /
        std::max(0.05, static_cast<double>(rows[i].weight));
    }
    if (!std::isfinite(shifts[i])) {
      return;
    }
    maximum_shift = std::max(maximum_shift, std::abs(shifts[i]));
  }
  // Scale the whole segment's correction, avoiding pointwise clamp corners.
  const double scale = config.correction_limit_enabled && maximum_shift > 0.0 ?
    std::min(1.0, config.max_correction_px / maximum_shift) : 1.0;
  for (int i = 0; i < count; ++i) {
    rows[i].shift = static_cast<float>(scale * shifts[i]);
  }
}

}  // namespace

void validate_lane_smoothing(const LaneSmoothingConfig & config)
{
  if (!std::isfinite(config.strength) || config.strength < 0.0 || config.strength > 1.0e6) {
    throw std::invalid_argument("smoothing_strength must be in [0,1000000]");
  }
  if (!std::isfinite(config.max_correction_px) || config.max_correction_px < 0.0) {
    throw std::invalid_argument("smoothing_max_correction_px must be finite and nonnegative");
  }
  if (!std::isfinite(config.max_row_jump_px) || config.max_row_jump_px <= 0.0) {
    throw std::invalid_argument("smoothing_max_row_jump_px must be finite and positive");
  }
  if (config.min_segment_rows < 3) {
    throw std::invalid_argument("smoothing_min_segment_rows must be at least 3");
  }
}

void smooth_lane_rows(LaneRow * rows, const int height, const LaneSmoothingConfig & config)
{
  for (int i = 0; i < 2 * height; ++i) {
    rows[i].shift = 0.0F;
  }
  if (!config.enabled || config.strength == 0.0) {
    return;
  }
  for (int lane = 0; lane < 2; ++lane) {
    auto * channel = rows + lane * height;
    int begin = 0;
    while (begin < height) {
      if (channel[begin].weight <= 0.0F) {
        ++begin;
        continue;
      }
      int end = begin + 1;
      while (end < height && channel[end].weight > 0.0F &&
        std::abs(channel[end].center - channel[end - 1].center) <= config.max_row_jump_px)
      {
        ++end;
      }
      if (end - begin >= config.min_segment_rows) {
        smooth_segment(channel + begin, end - begin, config);
      }
      begin = end;
    }
  }
}

}  // namespace line_detactor
