// Copyright (c) 2025 Feng. All Rights Reserved.
#include "ballistic_trajectory.hpp"
#include "ballistic_models.hpp"
#include <cmath>

namespace rm_ultra_tools {

BallisticTrajectorySolver::BallisticTrajectorySolver(
    const BallisticParams &param) {}

BallisticState2D
BallisticTrajectorySolver::getPos2DByT(const BallisticState2D &pos0,
                                       double time) {
  return {ballistic_models::getPos2DByT(pos0.toTuple(), std::move(time),
                                        params.time_step, params.k, params.g)};
}

BallisticState2D
BallisticTrajectorySolver::rk45SingleStep(const BallisticState2D &pos0) {
  return getPos2DByT(pos0, this->params.time_step);
}

Eigen::Vector3d
BallisticTrajectorySolver::getPosXyzByT(const BallisticState2D &pos0,
                                        double yaw, double time) {
  auto &&[x, y, z] = ballistic_models::getXyzByT(
      pos0.toTuple(), std::move(yaw), std::move(time), params.time_step,
      params.k, params.g);
  return {x, y, z};
}

bool BallisticTrajectorySolver::isHit(
    const rm_ultra_tools::BallisticState2D &bullet_pos2d,
    const Eigen::Vector3d &target_pos3d) {
  Eigen::Vector2d eg_target2d{hypot(target_pos3d.x(), target_pos3d.y()),
                              target_pos3d.z()};
  Eigen::Vector2d eg_ballet_pos2d{bullet_pos2d.distance, bullet_pos2d.height};
  return eg_ballet_pos2d.norm() >= eg_target2d.norm();
}

bool BallisticTrajectorySolver::isHit(const Eigen::Vector3d &bullet_pos2d,
                                      const Eigen::Vector3d &target_pos3d) {
  return isHit(BallisticState2D{std::hypot(bullet_pos2d.x(), bullet_pos2d.y()),
                                bullet_pos2d.z(), 0., 0.},
               target_pos3d);
}

BallisticState2D BallisticTrajectorySolver::transformPos2DGimbalToBarrel(
    const BallisticState2D &pos2d) {
  BallisticState2D tmp = pos2d;
  tmp.distance += params.barrel_length * std::cos(pos2d.pitch);
  tmp.height += params.barrel_length * std::sin(pos2d.pitch);
  return tmp;
}
} // namespace rm_ultra_tools
