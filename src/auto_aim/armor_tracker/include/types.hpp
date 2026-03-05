// Copyright (c) 2026 GuGuGaAaaaa. All Rights Reserved.
#pragma once

#include "types/Armor.hpp"
#include "types/ArmorType.hpp"

#include <Eigen/Core>
#include <gtsam/geometry/Rot2.h>

#include <functional>
#include <vector>

namespace auto_aim {

// NOTE:
// 四个装甲板时的索引
//            ^x
//            |              ^
//            2              |
//            |  /->rb       | center_yaw方向
// y<-----3---+---1-----
//            |->ra
//            0
//            |
// 三个装甲板时同理（都是均分圆周后由-x轴开始逆时针旋转）
// Robot在13方向上使用dz，outpost在1使用dz_a，在2使用dz_b
enum class ArmorIndex {
  _0,
  _1,
  _2,
  _3,
};

struct ArmorPositionYaw {
  ArmorPositionYaw() = default;
  ArmorPositionYaw(const types::Armor &armor);
  ArmorPositionYaw(const Eigen::Vector3d &center_position, double center_yaw,
                   double radius, double dz, int armor_numbers,
                   ArmorIndex index);
  Eigen::Quaterniond getRotation(double pitch, double roll) const;
  Eigen::Vector3d position;
  gtsam::Rot2 yaw;
};

struct TargetState {
  types::ArmorType type;
  Eigen::Vector3d center_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_velocity = Eigen::Vector3d::Zero();
  double center_yaw = 0; // NOTE: 为了不泄漏gtsam，先不用Rot2了
  double center_vyaw = 0;

  TargetState() = default;
  TargetState getStateWithArmorsFunc(
      std::function<std::vector<ArmorPositionYaw>(const TargetState &self)>
          &&get_armors_fn) const {
    auto state = *this;
    state.get_armors_ = get_armors_fn;
    return state;
  }
  TargetState predict(double dt) const;
  std::vector<ArmorPositionYaw> armors() const;

private:
  std::function<std::vector<ArmorPositionYaw>(const TargetState &self)>
      get_armors_;
};

struct RobotTargetState : public TargetState {
  RobotTargetState predict(double dt) const;
  double radius_a;
  double radius_b;
  double dz;
};

struct OutpostTargetState : public TargetState {
  double radius;
  double dz_a;
  double dz_b;
};

struct TrackState {
  enum class State {
    LOST,
    TEMPLOST,
    TRACKING,
  } state = State::LOST;
  std::uint64_t k = 0;
  std::chrono::system_clock::time_point stamp_last_update;
  std::chrono::system_clock::time_point stamp_last_tracking;
};

struct [[deprecated]] ArmorMatchError {
  double distance;
  double yaw_diff;
};
struct ArmorMatchResult {
  ArmorIndex index;
  double distance; // m
  double yaw_diff; // rad
};

struct YawPitchFlytime {
  double yaw;
  double pitch;
  double fly_time;
};

} // namespace auto_aim
