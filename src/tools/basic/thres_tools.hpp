#pragma once

#include <algorithm>

namespace tools {

inline double limitZeroThres(double in, double zero_thres, double max) {
  return std::min(in < zero_thres ? 0. : in, max);
}
} // namespace tools
