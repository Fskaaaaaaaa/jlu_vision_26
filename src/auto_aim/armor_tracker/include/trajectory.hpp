#pragma once

#include "configs.hpp"
#include "math/ballistic_trajectory.hpp"
#include "types.hpp"

#include "quill/Logger.h"

#include <optional>

namespace auto_aim {

// NOTE: 这个类负责三维的弹道计算
class Trajectory {
public:
  Trajectory(quill::Logger *logger, const TrajectoryConfig &config);
  // 假设传入的state已经predict过算法延迟和拨盘延迟了，后续计算只新增子弹飞行时间
  std::optional<YawPitchFlyTimeIndex>
  solveTarget(const TargetState &target_state, double bullet_speed,
              bool iterative_fly_time = true, bool use_rk45 = false);

private:
  static double getArmorFacingAngleAbs(const ArmorPositionYaw &armor);
  ArmorIndex selectArmor(const TargetState &state) const;
  std::optional<YawPitchFlyTimeIndex>
  solveArmor(ArmorIndex armor_index, double fly_time,
             const TargetState &target_state, double bullet_speed,
             bool iterative_fly_time, bool use_rk45) const;

  quill::Logger *logger_;
  TrajectoryConfig config_;

  tools::ballistic::BallisticTrajectorySolver solver_;

  std::optional<ArmorIndex> last_armor_index_;
};

} // namespace auto_aim
