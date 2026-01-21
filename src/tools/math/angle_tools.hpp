#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include <Eigen/Dense>

namespace tools {
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

inline Eigen::Quaterniond rpyToQuaterniond(const Eigen::Vector3d &rpy_angle) {
  Eigen::AngleAxisd roll_angle(angle2Radian(rpy_angle.x()),
                               Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(angle2Radian(rpy_angle.y()),
                                Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(angle2Radian(rpy_angle.z()),
                              Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q{yaw_angle * pitch_angle * roll_angle};
  q.normalize();
  return q;
}

} // namespace tools
