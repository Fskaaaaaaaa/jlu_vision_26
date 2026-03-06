#pragma once

#include "configs.hpp"
#include "msgs/AimCommand.hpp"
#include "trajectory.hpp"
#include "types.hpp"

#include "quill/Logger.h"
#include "tiny_api.hpp"

#include <atomic>
#include <chrono>

namespace auto_aim {

class Planner {
public:
  Planner(quill::Logger *logger, const PlannerConfig &config);

  msgs::AimCommand
  plan(const TargetState &target_state,
       const std::chrono::system_clock::time_point &target_stamp,
       double bullet_speed_mps);

  // for debug usage
  std::atomic<double> aim0_predict_time_;

private:
  msgs::AimCommand plan(const TargetState &target_state,
                        double dt_image_to_now_sec,
                        double bullet_speed_mps);
  std::pair<Eigen::MatrixXd, double>
  buildTrajectoryReference(const TargetState &target_state,
                           double dt_image_to_now_sec,
                           double bullet_speed_mps);
  quill::Logger *logger_;
  PlannerConfig config_;
  int trajectory_horizon_;
  unsigned int bullet_id_;
  Trajectory trajectory_solver_;

  TinySolver *yaw_solver_;
  TinySolver *pitch_solver_;
};

} // namespace auto_aim
