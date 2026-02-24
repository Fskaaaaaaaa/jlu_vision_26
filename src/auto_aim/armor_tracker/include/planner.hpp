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
  plan(const std::optional<TargetStatus> &target_state,
       const std::chrono::system_clock::time_point &target_stamp,
       double bullet_speed);

  // for debug usage
  std::atomic<double> aim0_predict_time_;

private:
  msgs::AimCommand plan(const TargetStatus &target_state,
                        double dt_image_to_now, double bullet_speed);
  std::pair<Eigen::MatrixXd, double>
  getTrajectoryYaw0(const TargetStatus &target_state, double dt_image_to_now,
                    double bullet_speed);
  quill::Logger *logger_;
  PlannerConfig config_;
  int trajectory_horizon_;
  unsigned int bullet_id_;
  Trajectory traj_solver_;

  TinySolver *yaw_solver_;
  TinySolver *pitch_solver_;
};

} // namespace auto_aim
