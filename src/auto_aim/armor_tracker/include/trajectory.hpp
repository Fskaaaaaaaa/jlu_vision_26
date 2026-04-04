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

  std::optional<TargetAimSolution>
  resolveTarget(const TargetState &state, double bullet_speed_mps,
                double delay_time_image_to_now_sec, bool use_rk45,
                bool iterative_fly_time = true,
                std::optional<ArmorIndex> preferred_armor_index = std::nullopt);
  std::pair<ArmorPositionYaw, ArmorIndex> selectArmorForAiming(
      const TargetState &state,
      std::optional<ArmorIndex> preferred_armor_index = std::nullopt,
      double odom_x_m = 0.0, double odom_y_m = 0.0) const;

private:
  // 对指定装甲板位置解算弹道，不参与选板策略。
  std::optional<YawPitchFlyTime> solveYawPitchForArmorPosition(
      double bullet_speed_mps, const Eigen::Vector3d &armor_position_m,
      bool use_rk45, double odom_x_m = 0.0, double odom_y_m = 0.0);
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
