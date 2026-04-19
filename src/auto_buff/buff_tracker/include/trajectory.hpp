#pragma once

#include "configs.hpp"
#include "math/ballistic_trajectory.hpp"

#include "quill/Logger.h"
#include "types.hpp"

#include <optional>

namespace auto_buff {

class Trajectory {
public:
  Trajectory(quill::Logger *logger, const TrajectoryConfig &config);
  std::optional<YawPitchFlyTimeIndex> solveBuff(const BuffState &buff_state,
                                                double bullet_speed);

private:
  std::optional<YawPitchFlyTimeIndex> solveBlade(BuffBladeIndex blade_index,
                                                 const BuffState &buff_state,
                                                 double bullet_speed,
                                                 bool iterative_fly_time) const;
  std::optional<BuffBladeIndex> selectBlade(const BuffState &state) const;

  quill::Logger *logger_;
  TrajectoryConfig config_;

  tools::ballistic::BallisticTrajectorySolver solver_;
};

} // namespace auto_buff
