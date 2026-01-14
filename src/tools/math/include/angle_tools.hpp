#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace rm_ultra_tools {
inline double limitRadian(double angle,
                          std::pair<double, double> range = {
                              -std::numbers::pi, std::numbers::pi}) {
  const double low = range.first;
  const double high = range.second;
  const double width = high - low;
  angle = std::fmod(angle - low, width);
  if (angle < 0.0)
    angle += width;   // 处理负数 fmod
  return angle + low; // 回到 (low, high]
}

inline double shortestAngularDistance(double from, double to) {
  return limitRadian(to - from);
}

inline double angle2Radian(double angle) {
  return angle * std::numbers::pi / 180.;
}

inline double radian2Angle(double radian) {
  return radian * 180. / std::numbers::pi;
}

inline double limitZeroThres(double in, double zero_thres, double max) {
  return std::min(in < zero_thres ? 0. : in, max);
}

} // namespace rm_ultra_tools
