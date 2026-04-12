// Copyright (c) 2026 GuGuGaAaaaa. All Rights Reserved.
#pragma once

#include "types/Armor.hpp"
#include "types/ArmorType.hpp"

#include <Eigen/Core>
#include <array>
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
enum class ArmorIndex {
  _0 = 0,
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

struct ArmorPositionRollPitchYawPoints : public ArmorPositionYaw {
  ArmorPositionRollPitchYawPoints() = default;
  ArmorPositionRollPitchYawPoints(const types::Armor &armor);
  Eigen::Quaterniond getRotation() const;
  std::array<Eigen::Vector2d, 4> points;
  double pitch;
  double roll;
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
  OutpostTargetState predict(double dt) const;
  double radius;
  double dz_0;
  double dz_1;
  double dz_2;
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

struct ArmorMatchResult {
  ArmorIndex index;
  double distance; // m
  double yaw_diff; // rad
};

struct YawPitchFlyTimeIndex {
  double yaw;
  double pitch;
  double fly_time;
  ArmorIndex index;
};

struct AimTrajectoryReference {
  Eigen::MatrixXd state_reference;
  double center_yaw = 0.0;
  double center_fly_time = 0;
  ArmorIndex center_selected_index;
  int fallback_sample_count = 0;
};

struct BallisticDispersion { // 散布
  double horizontal;
  double vertical;
};

} // namespace auto_aim
