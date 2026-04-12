// Copyright (c) 2026 Fr. All Rights Reserved.
#pragma once

#include "configs.hpp"
#include "types.hpp"
#include "types/ArmorType.hpp"

#include "quill/Logger.h"
#include <Eigen/Core>
#include <gtsam/base/types.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <opencv2/core.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

namespace auto_aim {

class Target {
public:
  virtual TrackState::State
  track(const std::vector<types::Armor> &armors,
        const std::chrono::system_clock::time_point &stamp,
        const Eigen::Isometry3d &T_camera_to_odom) = 0;
  // NOTE: 因为涉及到给targetstate注入armors逻辑，需要是虚函数
  virtual std::pair<TargetState, TrackState> getTargetTrackState() const = 0;
  virtual double get(const std::string &key) const { return 0; };

  static std::vector<ArmorMatchResult>
  matchArmor(const std::vector<ArmorPositionYaw> &armors,
             const ArmorPositionYaw &obs, double max_match_distance,
             double max_match_yaw_diff);
  static gtsam::Key distributeKey();
};

class RobotTarget : public Target {
public:
  RobotTarget(quill::Logger *logger, const RobotConfig &config,
              types::ArmorType type, const cv::Mat &camera_matrix,
              const cv::Mat &distortion_coefficients);
  TrackState::State track(const std::vector<types::Armor> &armors,
                          const std::chrono::system_clock::time_point &stamp,
                          const Eigen::Isometry3d &T_camera_to_odom) override;

  std::pair<TargetState, TrackState> getTargetTrackState() const override;

  // for debug
  double get(const std::string &key) const override;

private:
  // NOTE: 为了添加像素关键点信息将PoseYaw换成添加了Points的子类了
  std::pair<RobotTargetState, TrackState::State>
  update(const std::vector<ArmorPositionRollPitchYawPoints> &armors, double dt,
         const Eigen::Isometry3d &T) const;

  std::vector<ArmorPositionYaw>
  getArmorsFromTargetState(const RobotTargetState &state) const;

  static std::vector<ArmorPositionYaw>
  getArmorsFromTargetState(const TargetState &state, double radius_a,
                           double radius_b, double dz);

  std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
  matchArmors(
      const RobotTargetState &state,
      const std::vector<ArmorPositionRollPitchYawPoints> &obs_armors_camera,
      const std::vector<ArmorPositionRollPitchYawPoints> &obs_armors_odom)
      const;

  RobotTargetState getTargetStateFromArmor(const ArmorPositionYaw &armor) const;

  void addMotionValuesFactors(gtsam::Values &values,
                              gtsam::NonlinearFactorGraph &graph,
                              const TargetState &target_state, std::uint64_t k,
                              double dt) const;
  void addArmorValuesFactors(
      gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
      const std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
          &armors_indexs,
      const Eigen::Isometry3d &T, std::uint64_t k) const;
  void addArmorReprojValuesFactors(gtsam::Values &values,
                                   gtsam::NonlinearFactorGraph &graph,
                                   gtsam::Key armor_pose_key,
                                   const ArmorPositionRollPitchYawPoints &armor,
                                   std::uint64_t k) const;

  quill::Logger *logger_;
  RobotConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;

  RobotTargetState target_state_;
  TrackState track_state_;

  mutable std::mutex state_mtx_;
  mutable gtsam::ISAM2 isam2_;

  mutable gtsam::Values initial_values_;
  mutable gtsam::NonlinearFactorGraph initial_graph_;
};

class OutpostTarget : public Target {
public:
  OutpostTarget(quill::Logger *logger, const OutpostConfig &config,
                const cv::Mat &camera_matrix,
                const cv::Mat &distortion_coefficients);
  TrackState::State track(const std::vector<types::Armor> &armors,
                          const std::chrono::system_clock::time_point &stamp,
                          const Eigen::Isometry3d &T_camera_to_odom) override;

  std::pair<TargetState, TrackState> getTargetTrackState() const override;

  // for debug
  double get(const std::string &key) const override;

private:
  std::pair<OutpostTargetState, TrackState::State>
  update(const std::vector<ArmorPositionRollPitchYawPoints> &armors, double dt,
         const Eigen::Isometry3d &T) const;

  std::vector<ArmorPositionYaw>
  getArmorsFromTargetState(const OutpostTargetState &state) const;

  static std::vector<ArmorPositionYaw>
  getArmorsFromTargetState(const TargetState &state, double radius, double dz_0,
                           double dz_1, double dz_2);

  std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
  matchArmors(
      const OutpostTargetState &state,
      const std::vector<ArmorPositionRollPitchYawPoints> &obs_armors_camera,
      const std::vector<ArmorPositionRollPitchYawPoints> &obs_armors_odom)
      const;

  OutpostTargetState
  getTargetStateFromArmor(const ArmorPositionYaw &armor) const;

  void addMotionValuesFactors(gtsam::Values &values,
                              gtsam::NonlinearFactorGraph &graph,
                              const TargetState &target_state, std::uint64_t k,
                              double dt) const;
  void addArmorValuesFactors(
      gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
      const OutpostTargetState &target_state,
      const std::vector<std::pair<ArmorPositionRollPitchYawPoints, ArmorIndex>>
          &armors_indexs,
      const Eigen::Isometry3d &T, std::uint64_t k) const;
  void addArmorReprojValuesFactors(gtsam::Values &values,
                                   gtsam::NonlinearFactorGraph &graph,
                                   gtsam::Key armor_pose_key,
                                   const ArmorPositionRollPitchYawPoints &armor,
                                   std::uint64_t k) const;

  quill::Logger *logger_;
  OutpostConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;

  OutpostTargetState target_state_;
  TrackState track_state_;

  mutable std::mutex state_mtx_;
  mutable gtsam::ISAM2 isam2_;

  mutable gtsam::Values initial_values_;
  mutable gtsam::NonlinearFactorGraph initial_graph_;
};

class BaseTarget : public Target {
public:
private:
};

} // namespace auto_aim
