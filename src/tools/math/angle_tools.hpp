#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include <Eigen/Dense>

namespace tools {
inline double limitRadian(double radian,
                          std::pair<double, double> range = {
                              -std::numbers::pi, std::numbers::pi}) {
  const double low = range.first;
  const double high = range.second;
  const double width = high - low;
  radian = std::fmod(radian - low, width);
  if (radian < 0.0)
    radian += width;   // 处理负数 fmod
  return radian + low; // 回到 (low, high]
}

inline double shortestRadianDistance(double from, double to) {
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

inline Eigen::Quaterniond rpyToQuaterniond(const Eigen::Vector3d &rpy_angle) {
  Eigen::AngleAxisd roll(rpy_angle.x(), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(rpy_angle.y(), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(rpy_angle.z(), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q{yaw * pitch * roll};
  q.normalize();
  return q;
}

inline Eigen::Quaterniond
rpyAngleToQuaterniond(const Eigen::Vector3d &rpy_angle) {
  return rpyAngleToQuaterniond({angle2Radian(rpy_angle.x()),
                                angle2Radian(rpy_angle.y()),
                                angle2Radian(rpy_angle.z())});
}

} // namespace tools
