// Copyright (c) 2025 Feng. All Rights Reserved.
#include "math/ballistic_trajectory.hpp"
#include "math/angle_tools.hpp"

#include <array>
#include <cmath>
#include <optional>

tools::ballistic::BallisticState2D tools::ballistic::getBarrelStateFromPitch(
    double pitch_rad, double muzzle_velocity_mps, double barrel_length) {
  return BallisticState2D{
      std::cos(pitch_rad) * barrel_length,
      std::sin(pitch_rad) * barrel_length,
      pitch_rad,
      muzzle_velocity_mps,
  };
}

tools::ballistic::BallisticState2D
tools::ballistic::BallisticTrajectorySolver::getBarrelStateFromPitch(
    double pitch_rad, double muzzle_velocity_mps) const {
  return ::tools::ballistic::getBarrelStateFromPitch(
      pitch_rad, muzzle_velocity_mps, config_.barrel_length);
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::BallisticTrajectorySolver::resolvePitchFlyTime(
    double target_distance_m, double target_height_m,
    double muzzle_velocity_mps, Method method) const {
  if (method == Method::parabola)
    return solveParabola(target_distance_m, target_height_m,
                         muzzle_velocity_mps);
  if (method == Method::rk45)
    return solveRk45(target_distance_m, target_height_m, muzzle_velocity_mps);
  return std::nullopt;
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::solveTrajectoryParabola(double target_distance_m,
                                          double target_height_m,
                                          double muzzle_velocity_mps, double g,
                                          double gimbal_pitch_min_degree,
                                          double gimbal_pitch_max_degree) {
  if (target_distance_m <= 0.0 || muzzle_velocity_mps <= 0.0)
    return std::nullopt;
  auto a = g * target_distance_m * target_distance_m /
           (2.0 * muzzle_velocity_mps * muzzle_velocity_mps);
  auto b = -target_distance_m;
  auto c = a + target_height_m;
  auto delta = b * b - 4.0 * a * c;
  if (delta < 0.0)
    return std::nullopt;
  auto tan_pitch1 = (-b + std::sqrt(delta)) / (2.0 * a);
  auto tan_pitch2 = (-b - std::sqrt(delta)) / (2.0 * a);
  auto pitch1 = std::atan(tan_pitch1);
  auto pitch2 = std::atan(tan_pitch2);
  auto min_pitch_rad = tools::angle2Radian(gimbal_pitch_min_degree);
  auto max_pitch_rad = tools::angle2Radian(gimbal_pitch_max_degree);
  std::optional<PitchFlytime> best_solution;
  for (auto pitch_rad : std::array{pitch1, pitch2}) {
    if (pitch_rad < min_pitch_rad || pitch_rad > max_pitch_rad)
      continue;
    auto cos_pitch = std::cos(pitch_rad);
    auto fly_time_sec = target_distance_m / (muzzle_velocity_mps * cos_pitch);
    if (!best_solution.has_value() || fly_time_sec < best_solution->fly_time) {
      best_solution = PitchFlytime{
          .pitch = pitch_rad,
          .fly_time = fly_time_sec,
      };
    }
  }
  return best_solution;
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::BallisticTrajectorySolver::solveParabola(
    double target_distance_m, double target_height_m,
    double muzzle_velocity_mps) const {
  return ::tools::ballistic::solveTrajectoryParabola(
      target_distance_m, target_height_m, muzzle_velocity_mps, config_.g,
      config_.gimbal_pitch_min_degree, config_.gimbal_pitch_max_degree);
}
