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
  resolveTarget(const TargetState &state, double bullet_speed_mps,
                double delay_time_image_to_now_sec, bool use_rk45,
                bool iterative_fly_time = true);
  static std::pair<ArmorPositionYaw, ArmorIndex>
  getClosestArmorIndexFromTarget(const TargetState &state);

private:
  // 选择平面距离最近的装甲板，降低遮挡和切板导致的瞄准跳变。
  std::optional<YawPitchFlytime>
  solveYawPitchForArmorPosition(double bullet_speed_mps,
                                const Eigen::Vector3d &armor_position_m,
                                bool use_rk45, double odom_x_m = 0.0,
                                double odom_y_m = 0.0);
  double evaluateImpactPositionError(double yaw_rad, double pitch_rad,
                                     double bullet_speed_mps,
                                     double fly_time_sec,
                                     const Eigen::Vector3d &armor_position_m,
                                     bool use_rk45);

  quill::Logger *logger_;
  TrajectoryConfig config_;

  tools::ballistic::BallisticTrajectorySolver solver_;
};

} // namespace auto_aim
