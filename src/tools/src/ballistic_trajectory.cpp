// Copyright (c) 2025 Feng. All Rights Reserved.
#include "math/ballistic_trajectory.hpp"
#include "math/angle_tools.hpp"
#include "math/ballistic_models.hpp"
#include "math/bisection.hpp"

#include "quill/LogMacros.h"

#include <cmath>
#include <optional>
#include <stdexcept>

tools::ballistic::BallisticState2D
tools::ballistic::BallisticTrajectorySolver::getBarrelStateFromPitch(
    double pitch, double v0) const {
  return BallisticState2D{std::cos(pitch) * config_.barrel_length,
                          std::sin(pitch) * config_.barrel_length, pitch, v0};
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::BallisticTrajectorySolver::resolvePitchFlyTime(
    double distance, double height, double v0, Method method) const {
  if (method == Method::rk45) {
    return resolveRk45(distance, height, v0);
  }
  if (method == Method::parabola) {
    return resolveParabola(distance, height, v0);
  }
  return std::nullopt;
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::BallisticTrajectorySolver::resolveRk45(double distance,
                                                         double height,
                                                         double v0) const {
  double fly_time;
  auto cost_function = [this, v0, distance, height, &fly_time](double pitch) {
    auto state2d = getBarrelStateFromPitch(pitch, v0);
    fly_time = 0;
    while (!isExceedTargetRange(state2d, distance, height)) {
      state2d = rk45::rk45SingleStep(state2d, config_.time_step, config_.k,
                                     config_.g);
      fly_time += config_.time_step;
      if (fly_time >= config_.max_fly_time)
        throw std::runtime_error{"fly_time too long!"};
    }
    return state2d.height - height;
    // 二分法每次迭代损失函数误差小于零区间会向右收缩，反之则会向左收缩
    // 假设收敛(0,30)角度内的云台pitch，打低了error为负，故收敛右半区间，仰角加大
  };
  auto gimbal_pitch_max = tools::angle2Radian(config_.gimbal_pitch_max_degree);
  auto gimbal_pitch_min = tools::angle2Radian(config_.gimbal_pitch_min_degree);
  try {
    if (std::signbit(cost_function(gimbal_pitch_max)) ==
        std::signbit(cost_function(gimbal_pitch_min))) {
      LOG_DEBUG(logger_, "[BallisticTrajectorySolver]: Beyond gimbal range!");
      return std::nullopt;
    } else
      return PitchFlytime{
          .pitch =
              Bisection::find(gimbal_pitch_min, gimbal_pitch_max, cost_function,
                              config_.max_pitch_iterate_count,
                              config_.min_pitch_error_m)
                  .first,
          .fly_time = fly_time,
      };
  } catch (const std::exception &e) {
    LOG_DEBUG(logger_,
              "[BallisticTrajectorySolver]: Fly time too long! fly_time{}",
              fly_time);
    return std::nullopt;
  }
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::BallisticTrajectorySolver::resolveParabola(double distance,
                                                             double height,
                                                             double v0) const {
  auto a = config_.g * distance * distance / (2 * v0 * v0);
  auto b = -distance;
  auto c = a + height;
  auto delta = b * b - 4 * a * c;
  if (delta < 0)
    return std::nullopt;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = distance / (v0 * std::cos(pitch_1));
  auto t_2 = distance / (v0 * std::cos(pitch_2));
  return PitchFlytime{
      .pitch = (t_1 < t_2) ? pitch_1 : pitch_2,
      .fly_time = (t_1 < t_2) ? t_1 : t_2,
  };
}

// namespace tools {
//
// BallisticTrajectorySolver::BallisticTrajectorySolver(
//     const BallisticConfig &config)
//     : config_(config) {}
//
// BallisticState2D
// BallisticTrajectorySolver::getPos2DByT(const BallisticState2D &pos0,
//                                        double time) const {
//   return {ballistic_models::getPos2DByT(pos0.toTuple(), std::move(time),
//                                         params_.time_step, params_.k,
//                                         params_.g)};
// }
//
// BallisticState2D
// BallisticTrajectorySolver::rk45SingleStep(const BallisticState2D &pos0) const
// {
//   return getPos2DByT(pos0, this->config_.time_step);
// }
//
// Eigen::Vector3d
// BallisticTrajectorySolver::getPosXyzByT(const BallisticState2D &pos0,
//                                         double yaw, double time,
//                                         bool analytic) const {
//   if (!analytic) {
//     auto &&[x, y, z] = ballistic_models::getXyzByT(
//         pos0.toTuple(), std::move(yaw), std::move(time), params_.time_step,
//         params_.k, params_.g);
//     return {x, y, z};
//   }
//   auto vx = pos0.velocity * std::cos(pos0.pitch);
//   auto vy0 = pos0.velocity * std::sin(pos0.pitch);
//   auto distance = pos0.distance + vx * time;
//   auto z = pos0.height + vy0 * time - 0.5 * config_.g * time * time;
//   auto x = distance * std::cos(yaw);
//   auto y = distance * std::sin(yaw);
//   return {x, y, z};
// }
//
//
// BallisticState2D BallisticTrajectorySolver::transformPos2DGimbalToBarrel(
//     const BallisticState2D &pos2d) const {
//   BallisticState2D tmp = pos2d;
//   tmp.distance += config_.barrel_length * std::cos(pos2d.pitch);
//   tmp.height += config_.barrel_length * std::sin(pos2d.pitch);
//   return tmp;
// }
//
// std::optional<PitchFlytime>
// BallisticTrajectorySolver::getAnalyticalAimingSolution(double v0, double d,
//                                                        double h) const {
//   auto a = config_.g * d * d / (2 * v0 * v0);
//   auto b = -d;
//   auto c = a + h;
//   auto delta = b * b - 4 * a * c;
//   if (delta < 0)
//     return std::nullopt;
//   auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
//   auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
//   auto pitch_1 = std::atan(tan_pitch_1);
//   auto pitch_2 = std::atan(tan_pitch_2);
//   auto t_1 = d / (v0 * std::cos(pitch_1));
//   auto t_2 = d / (v0 * std::cos(pitch_2));
//   return PitchFlytime{
//       .pitch = (t_1 < t_2) ? pitch_1 : pitch_2,
//       .fly_time = (t_1 < t_2) ? t_1 : t_2,
//   };
// }
// } // namespace tools
