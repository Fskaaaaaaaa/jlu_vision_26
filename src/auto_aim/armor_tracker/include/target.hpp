// Copyright (c) 2026 Fr. All Rights Reserved.
#pragma once

#include "configs.hpp"
#include "types.hpp"
#include "types/ArmorType.hpp"

#include "quill/Logger.h"
#include <Eigen/Core>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

namespace auto_aim {

class Target {
public:
  virtual TrackState::State
  track(const std::vector<types::Armor> &armors,
        const std::chrono::system_clock::time_point &stamp) = 0;
  // NOTE: 因为涉及到给targetstate注入armors逻辑，需要是虚函数
  virtual std::pair<TargetState, TrackState> getTargetTrackState() const = 0;

  static std::vector<ArmorMatchResult>
  matchArmor(const std::vector<ArmorPositionYaw> &armors,
             const ArmorPositionYaw &obs, double max_match_distance,
             double max_match_yaw_diff);
};

class RobotTarget : public Target {
public:
  RobotTarget(quill::Logger *logger, const RobotConfig &config,
              types::ArmorType type);
  TrackState::State
  track(const std::vector<types::Armor> &armors,
        const std::chrono::system_clock::time_point &stamp) override;

  std::pair<TargetState, TrackState> getTargetTrackState() const override;

  // for debug
  std::array<double, 3> getRadiusARadiusBDZ() const;

private:
  std::pair<RobotTargetState, TrackState::State>
  update(const std::vector<ArmorPositionYaw> &armors, double dt) const;
  std::vector<ArmorPositionYaw>
  getArmorsFromTargetState(const RobotTargetState &state) const;
  static std::vector<ArmorPositionYaw>
  getArmorsFromTargetState(const TargetState &state, double radius_a,
                           double radius_b, double dz);
  std::vector<std::pair<ArmorPositionYaw, ArmorIndex>>
  matchArmorsUnique(const RobotTargetState &state,
                    const std::vector<ArmorPositionYaw> &obs_armors) const;
  RobotTargetState getTargetStateFromArmor(const ArmorPositionYaw &armor) const;
  void addMotionValuesFactors(gtsam::Values &values,
                              gtsam::NonlinearFactorGraph &graph,
                              const TargetState &target_state, std::uint64_t k,
                              double dt) const;
  void addArmorValuesFactors(
      gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
      const std::vector<std::pair<ArmorPositionYaw, ArmorIndex>> &armor_indexs,
      std::uint64_t k) const;

  quill::Logger *logger_;
  RobotConfig config_;
  RobotTargetState target_state_;
  TrackState track_state_;
  mutable std::mutex state_mtx_;
  mutable gtsam::ISAM2 isam2_;
};

class OutpostTarget : public Target {
public:
private:
};

class BaseTarget : public Target {
public:
private:
};

} // namespace auto_aim
