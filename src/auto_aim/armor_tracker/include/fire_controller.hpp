#pragma once

#include "configs.hpp"
#include "msgs/AimCommand.hpp"
#include "types.hpp"
#include "types/ArmorType.hpp"

#include "quill/Logger.h"
#include <optional>

namespace auto_aim {

class FireController {
public:
  FireController(quill::Logger *logger, const FireControllerConfig &config,
                 double g = 9.8);
  // HACK: 散布投影椭圆会随瞄准位置不同而小幅度变化，
  // 这里近似为求解中心瞄准的散布后平移至相切以求得开火阈值。
  // NOTE: 电控应该用“abs(真实-target)”与阈值相比较开决定是否开火，
  // 因为电机实际运动遵循的规划路径可能与target偏离。
  void calculateFireThres(msgs::AimCommand &cmd, double bullet_speed,
                          types::ArmorType armor_type,
                          const ArmorPositionYaw &armor) const;

private:
  quill::Logger *logger_;
  FireControllerConfig config_;
  double g_;

  // HACK: g由planner访问弹道推演器配置得到
  BallisticDispersion calculateDispersion(double target_pitch,
                                          double bullet_speed,
                                          const ArmorPositionYaw &armor) const;
  std::optional<double> getTargetPitch(double bullet_speed, double distance,
                                       double height) const;
};

} // namespace auto_aim
