#pragma once

#include "configs.hpp"
#include "math/ballistic_trajectory.hpp"
#include "types.hpp"

#include "quill/Logger.h"

#include <mutex>
#include <optional>

namespace auto_aim {

// NOTE: 这个类负责三维的弹道计算
class Trajectory {
public:
  Trajectory(quill::Logger *logger, const TrajectoryConfig &config);
  // 假设传入的state已经predict过算法延迟和拨盘延迟了，后续计算只新增子弹飞行时间
  std::optional<YawPitchFlyTime> solveTarget(const TargetState &target_state,
                                             double bullet_speed,
                                             bool iterative_fly_time = true,
                                             bool use_rk45 = false);
  std::optional<std::pair<ArmorIndex, double>> getAimIndexFlyTime() const;
  void setAimIndexFlyTimeCache(ArmorIndex index, double fly_time);

private:
  static double getArmorFacingAngleAbs(const ArmorPositionYaw &armor);
  ArmorIndex selectArmor(const TargetState &state) const;
  std::optional<YawPitchFlyTime>
  solveArmor(ArmorIndex armor_index, double fly_time,
             const TargetState &target_state, double bullet_speed,
             bool iterative_fly_time, bool use_rk45) const;

  quill::Logger *logger_;
  TrajectoryConfig config_;

  tools::ballistic::BallisticTrajectorySolver solver_;

  // 仅用作不同帧间调用的缓存，同一帧的访问还是走返回值路径
  mutable std::mutex aim_cache_mtx_;
  std::optional<ArmorIndex> last_armor_index_;
  double last_flytime_;
};

} // namespace auto_aim
