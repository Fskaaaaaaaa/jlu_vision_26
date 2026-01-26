// Copyright (c) 2026 tomorin. All Rights Reserved.
#pragma once

#include "iceoryx_posh/popo/sample.hpp"
#include "msgs/Header.hpp"
#include "msgs/Target.hpp"

#include <Eigen/Dense>

#include <chrono>
#include <functional>
#include <vector>
namespace types {
enum class TargetType {
  BigArmorRobot,
  SmallArmorRobot,
  Outpost,
  Base,
  SmallBuff,
  BigBuff,
};
struct Target {
  Target(
      const iox::popo::Sample<const msgs::Target, const msgs::Header> &sample);
  std::string frame_id;
  std::chrono::system_clock::time_point stamp;
  TargetType type;
  Eigen::Isometry3d center_pose;
  std::function<std::vector<Eigen::Isometry3d>(double dt)> kinematic_function;
};
} // namespace types
