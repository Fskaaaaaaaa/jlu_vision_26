#pragma once

#include "configs.hpp"
#include "fire_controller.hpp"
#include "msgs/AimCommand.hpp"
#include "msgs/GimbalInfo.hpp"
#include "trajectory.hpp"
#include "types.hpp"

#include "quill/Logger.h"
#include "tiny_api.hpp"

#include <chrono>
#include <mutex>
#include <tuple>

namespace auto_aim {

class Planner {
public:
  Planner(quill::Logger *logger, const PlannerConfig &config);

  msgs::AimCommand
  plan(const TargetState &target_state,
       const std::chrono::system_clock::time_point &target_stamp,
       const msgs::GimbalInfo &gimbal_info);

  std::tuple<ArmorPositionYaw, ArmorIndex, double, double, double>
  getAimingArmorIndexPredictTimeFireThres(const TargetState &state) const;

private:
  AimTrajectoryReference
  buildReferenceTrajectory(const TargetState &target_state,
                           double bullet_speed_mps);
  // HACK: 引用的出参用来传递迭代求解的飞行时间
  msgs::AimCommand aimMPC(const TargetState &target_state,
                          double bullet_speed_mps, double &fly_time,
                          ArmorIndex &selected_index);
  [[deprecated]]
  bool shouldAimCenter(const TargetState &target_state);
  [[deprecated]]
  msgs::AimCommand aimCenter(const TargetState &target_state,
                             double dt_image_to_now_sec,
                             double bullet_speed_mps);
  quill::Logger *logger_;
  PlannerConfig config_;
  int trajectory_horizon_;
  unsigned int bullet_id_;
  Trajectory trajectory_solver_;
  FireController fire_controller_;

  // debug异步，用于还原现场
  // 调试线程只在有未过时的瞄准目标时访问，故无需opt语义
  mutable std::mutex cache_mtx_;
  double predict_time_cache_{0};
  double fly_time_cache_{0};
  ArmorIndex selected_index_cache_{ArmorIndex::_0};
  double yaw_fire_thres_{0};
  double pitch_fire_thres_{0};

  TinySolver *yaw_solver_;
  TinySolver *pitch_solver_;
};

} // namespace auto_aim
