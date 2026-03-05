// Copyright (c) 2026 I Love CCB. All Rights Reserved.
#include "types.hpp"
#include "math/angle_tools.hpp"

#include <gtsam/geometry/Rot2.h>

#include <vector>

auto_aim::ArmorPositionYaw::ArmorPositionYaw(const types::Armor &armor) {
  this->position = armor.position;
  this->yaw = gtsam::Rot2::fromAngle(armor.getRpy()(2));
}

auto_aim::ArmorPositionYaw::ArmorPositionYaw(
    const Eigen::Vector3d &center_position, double center_yaw, double radius,
    double dz, int armor_numbers, ArmorIndex index) {
  auto between_angle = (2 * std::numbers::pi) / armor_numbers;
  auto armor_yaw = center_yaw + static_cast<int>(index) * between_angle;
  auto armor_x = center_position.x() - radius * std::cos(armor_yaw);
  auto armor_y = center_position.y() - radius * std::sin(armor_yaw);
  auto armor_z = center_position.z() + dz;
  this->position = Eigen::Vector3d{armor_x, armor_y, armor_z};
  this->yaw = gtsam::Rot2::fromAngle(armor_yaw);
}

Eigen::Quaterniond auto_aim::ArmorPositionYaw::getRotation(double pitch,
                                                           double roll) const {
  return tools::rpyToQuaterniond({roll, pitch, yaw.theta()});
}

auto_aim::TargetState auto_aim::TargetState::predict(double dt) const {
  TargetState state = *this;
  state.center_position = this->center_position + this->center_velocity * dt;
  state.center_yaw = this->center_yaw + this->center_vyaw * dt;
  return state;
}

auto_aim::RobotTargetState
auto_aim::RobotTargetState::predict(double dt) const {
  RobotTargetState state = *this;
  state.center_position = this->center_position + this->center_velocity * dt;
  state.center_yaw = this->center_yaw + this->center_vyaw * dt;
  return state;
}

std::vector<auto_aim::ArmorPositionYaw> auto_aim::TargetState::armors() const {
  return get_armors_(*this);
}
