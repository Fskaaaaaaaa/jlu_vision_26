#pragma once

#include "types/Basic.hpp"

#include <Eigen/Dense>

namespace auto_aim {

struct RobotStatus {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d orientation;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d angular_acceleration;
}; // 这个就是机器人自己维护的了

struct RobotContrl {
  types::Vector3d traction_direction;
}; // 这个要保证曝露出底层变量，好取地址给ftxui用
   // 同时也需要构造引用保存app的成员互斥量来确保线程安全

} // namespace auto_aim
