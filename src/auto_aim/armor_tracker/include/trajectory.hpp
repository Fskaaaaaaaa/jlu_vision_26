#pragma once

#include "configs.hpp"
#include "math/ballistic_trajectory.hpp"
#include "types.hpp"

#include "quill/Logger.h"

#include <optional>

namespace auto_aim {

class Trajectory {
public:
  Trajectory(quill::Logger *logger, const TrajectoryConfig &config);

  std::optional<YawPitchFlytime>
  resolveTarget(const TargetState &state, double bullet_speed,
                double delay_time_image_to_now_sec, bool use_rk45,
                bool iterative_fly_time = true);
  static std::pair<ArmorPositionYaw, ArmorIndex>
  getClosestArmorIndexFromTarget(const TargetState &state);

private:
  // NOTE: 按xy距离远近排列，取最近的瞄准
  std::optional<YawPitchFlytime>
  resolveYawPitch(double bullet_speed, const Eigen::Vector3d &aim_position,
                  bool use_rk45, double odom_x = 0.0, double odom_y = 0.0);
  double calculateAimError(double yaw, double pitch, double v0, double fly_time,
                           const Eigen::Vector3d &aim_position, bool use_rk45);

  quill::Logger *logger_;
  TrajectoryConfig config_;

  tools::ballistic::BallisticTrajectorySolver solver_;
};

} // namespace auto_aim
