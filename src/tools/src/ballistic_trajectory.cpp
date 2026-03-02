// Copyright (c) 2025 Feng. All Rights Reserved.
#include "math/ballistic_trajectory.hpp"
#include "math/ballistic_models.hpp"
#include <cmath>
#include <optional>

namespace tools {

BallisticTrajectorySolver::BallisticTrajectorySolver(
    const BallisticParams &params)
    : params_(params) {}

BallisticState2D
BallisticTrajectorySolver::getPos2DByT(const BallisticState2D &pos0,
                                       double time) const {
  return {ballistic_models::getPos2DByT(pos0.toTuple(), std::move(time),
                                        params_.time_step, params_.k,
                                        params_.g)};
}

BallisticState2D
BallisticTrajectorySolver::rk45SingleStep(const BallisticState2D &pos0) const {
  return getPos2DByT(pos0, this->params_.time_step);
}

Eigen::Vector3d
BallisticTrajectorySolver::getPosXyzByT(const BallisticState2D &pos0,
                                        double yaw, double time,
                                        bool analytic) const {
  if (!analytic) {
    auto &&[x, y, z] = ballistic_models::getXyzByT(
        pos0.toTuple(), std::move(yaw), std::move(time), params_.time_step,
        params_.k, params_.g);
    return {x, y, z};
  }
  auto vx = pos0.velocity * std::cos(pos0.pitch);
  auto vy0 = pos0.velocity * std::sin(pos0.pitch);
  auto distance = pos0.distance + vx * time;
  auto z = pos0.height + vy0 * time - 0.5 * params_.g * time * time;
  auto x = distance * std::cos(yaw);
  auto y = distance * std::sin(yaw);
  return {x, y, z};
}

bool BallisticTrajectorySolver::isExceedTargetRange(
    const tools::BallisticState2D &bullet_pos2d,
    const Eigen::Vector3d &target_pos3d) {
  Eigen::Vector2d eg_target2d{std::hypot(target_pos3d.x(), target_pos3d.y()),
                              target_pos3d.z()};
  Eigen::Vector2d eg_bullet_pos2d{bullet_pos2d.distance, bullet_pos2d.height};
  return eg_bullet_pos2d.norm() >= eg_target2d.norm();
}

bool BallisticTrajectorySolver::isExceedTargetRange(
    const Eigen::Vector3d &bullet_pos2d, const Eigen::Vector3d &target_pos3d) {
  return isExceedTargetRange(
      BallisticState2D{std::hypot(bullet_pos2d.x(), bullet_pos2d.y()),
                       bullet_pos2d.z(), 0., 0.},
      target_pos3d);
}

BallisticState2D BallisticTrajectorySolver::transformPos2DGimbalToBarrel(
    const BallisticState2D &pos2d) const {
  BallisticState2D tmp = pos2d;
  tmp.distance += params_.barrel_length * std::cos(pos2d.pitch);
  tmp.height += params_.barrel_length * std::sin(pos2d.pitch);
  return tmp;
}

std::optional<PitchFlytime>
BallisticTrajectorySolver::getAnalyticalAimingSolution(double v0, double d,
                                                       double h) const {
  auto a = params_.g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;
  if (delta < 0)
    return std::nullopt;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));
  return PitchFlytime{
      .pitch = (t_1 < t_2) ? pitch_1 : pitch_2,
      .fly_time = (t_1 < t_2) ? t_1 : t_2,
  };
}
} // namespace tools
