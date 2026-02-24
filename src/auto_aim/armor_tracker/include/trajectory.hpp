#pragma once

#include "configs.hpp"
#include "math/ballistic_trajectory.hpp"
#include "math/bisection.hpp"
#include "types.hpp"

#include "quill/Logger.h"

#include <optional>

namespace auto_aim {

struct YawPitchFlytime {
  double yaw;
  double pitch;
  double fly_time;
};

class Trajectory {
public:
  Trajectory(quill::Logger *logger, const TrajectoryConfig &config);

  std::optional<YawPitchFlytime>
  resolveTarget(const TargetStatus &status, double bullet_speed,
                double delay_time_image_to_now_sec = 0,
                bool use_analytical_solution = true,
                bool iterative_fly_time = true, double odom_x = 0.0,
                double odom_y = 0.0);
  static Armor getClosestArmor(const TargetStatus &status, double odom_x,
                               double odom_y);

private:
  std::optional<tools::PitchFlytime> resolvePitch(double bullet_speed,
                                                  double target_distance,
                                                  double target_height,
                                                  bool use_analytical_solution);

  // NOTE: 按xy距离远近排列，取最近的瞄准
  Eigen::Vector3d getSeletedArmorPosition(const TargetStatus &status,
                                          double odom_x, double odom_y);
  std::optional<YawPitchFlytime>
  resolveYawPitch(double bullet_speed, const Eigen::Vector3d &aim_position,
                  bool use_analytical_solution, double odom_x = 0.0,
                  double odom_y = 0.0);

  quill::Logger *logger_;
  TrajectoryConfig config_;

  tools::Bisection bisection_;
  tools::BallisticTrajectorySolver solver_;
};

} // namespace auto_aim
