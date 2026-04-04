// Copyright (c) 2025 Feng. All Rights Reserved.
#include "math/ballistic_trajectory.hpp"
#include "math/angle_tools.hpp"

#include "quill/LogMacros.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace {

constexpr int kPitchBracketSampleCount = 24;
constexpr double kPitchBracketMinWidthRad = 1e-6;
constexpr double kSecantDenominatorMin = 1e-9;

inline bool hasDifferentSign(double left_value, double right_value) {
  return (left_value > 0.0 && right_value < 0.0) ||
         (left_value < 0.0 && right_value > 0.0);
}

} // namespace

tools::ballistic::BallisticState2D
tools::ballistic::BallisticTrajectorySolver::getBarrelStateFromPitch(
    double pitch_rad, double muzzle_velocity_mps) const {
  return BallisticState2D{
      std::cos(pitch_rad) * config_.barrel_length,
      std::sin(pitch_rad) * config_.barrel_length,
      pitch_rad,
      muzzle_velocity_mps,
  };
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::BallisticTrajectorySolver::resolvePitchFlyTime(
    double target_distance_m, double target_height_m,
    double muzzle_velocity_mps, Method method) const {
  if (method == Method::rk45)
    return resolveRk45(target_distance_m, target_height_m, muzzle_velocity_mps);
  if (method == Method::parabola)
    return resolveParabola(target_distance_m, target_height_m,
                           muzzle_velocity_mps);
  return std::nullopt;
}

tools::ballistic::BallisticTrajectorySolver::PitchResidual
tools::ballistic::BallisticTrajectorySolver::evaluatePitchByRk45(
    double pitch_rad, double target_distance_m, double target_height_m,
    double muzzle_velocity_mps) const {
  if (target_distance_m <= 0.0 || muzzle_velocity_mps <= 0.0)
    return {};
  auto previous_state = getBarrelStateFromPitch(pitch_rad, muzzle_velocity_mps);
  if (previous_state.distance >= target_distance_m) {
    return PitchResidual{
        .valid = true,
        .height_error_m = previous_state.height - target_height_m,
        .fly_time_sec = 0.0,
    };
  }
  double elapsed_time_sec = 0.0;
  while (elapsed_time_sec < config_.max_fly_time) {
    const double integration_step_sec =
        std::min(config_.time_step, config_.max_fly_time - elapsed_time_sec);
    auto current_state = rk45::rk45SingleStep(
        previous_state, integration_step_sec, config_.k, config_.g);
    elapsed_time_sec += integration_step_sec;
    if (current_state.distance >= target_distance_m) {
      const double distance_delta =
          current_state.distance - previous_state.distance;
      const double interpolation_ratio =
          distance_delta <= std::numeric_limits<double>::epsilon()
              ? 1.0
              : std::clamp((target_distance_m - previous_state.distance) /
                               distance_delta,
                           0.0, 1.0);
      const double impact_height_m =
          previous_state.height +
          interpolation_ratio * (current_state.height - previous_state.height);
      const double impact_time_sec = elapsed_time_sec - integration_step_sec +
                                     interpolation_ratio * integration_step_sec;
      return PitchResidual{
          .valid = true,
          .height_error_m = impact_height_m - target_height_m,
          .fly_time_sec = impact_time_sec,
      };
    }
    if (current_state.velocity <= 1e-3)
      break;
    previous_state = current_state;
  }
  return {};
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::BallisticTrajectorySolver::solvePitchByHybridMethod(
    double pitch_left_rad, double pitch_right_rad, double target_distance_m,
    double target_height_m, double muzzle_velocity_mps) const {
  auto left_residual = evaluatePitchByRk45(
      pitch_left_rad, target_distance_m, target_height_m, muzzle_velocity_mps);
  auto right_residual = evaluatePitchByRk45(
      pitch_right_rad, target_distance_m, target_height_m, muzzle_velocity_mps);
  if (!left_residual.valid || !right_residual.valid ||
      !hasDifferentSign(left_residual.height_error_m,
                        right_residual.height_error_m)) {
    return std::nullopt;
  }

  double best_pitch_rad = std::abs(left_residual.height_error_m) <
                                  std::abs(right_residual.height_error_m)
                              ? pitch_left_rad
                              : pitch_right_rad;
  PitchResidual best_residual = std::abs(left_residual.height_error_m) <
                                        std::abs(right_residual.height_error_m)
                                    ? left_residual
                                    : right_residual;

  for (int iteration_index = 0;
       iteration_index < config_.max_pitch_iterate_count; ++iteration_index) {
    const double residual_denominator =
        right_residual.height_error_m - left_residual.height_error_m;
    double candidate_pitch_rad =
        std::abs(residual_denominator) < kSecantDenominatorMin
            ? (pitch_left_rad + pitch_right_rad) * 0.5
            : pitch_right_rad - right_residual.height_error_m *
                                    (pitch_right_rad - pitch_left_rad) /
                                    residual_denominator;
    if (!std::isfinite(candidate_pitch_rad) ||
        candidate_pitch_rad <= pitch_left_rad + kPitchBracketMinWidthRad ||
        candidate_pitch_rad >= pitch_right_rad - kPitchBracketMinWidthRad) {
      candidate_pitch_rad = (pitch_left_rad + pitch_right_rad) * 0.5;
    }

    auto candidate_residual =
        evaluatePitchByRk45(candidate_pitch_rad, target_distance_m,
                            target_height_m, muzzle_velocity_mps);
    if (!candidate_residual.valid)
      return std::nullopt;
    if (std::abs(candidate_residual.height_error_m) <
        std::abs(best_residual.height_error_m)) {
      best_residual = candidate_residual;
      best_pitch_rad = candidate_pitch_rad;
    }

    if (std::abs(candidate_residual.height_error_m) <=
            config_.min_pitch_error_m ||
        (pitch_right_rad - pitch_left_rad) <= kPitchBracketMinWidthRad) {
      return PitchFlytime{
          .pitch = best_pitch_rad,
          .fly_time = best_residual.fly_time_sec,
      };
    }

    if (hasDifferentSign(left_residual.height_error_m,
                         candidate_residual.height_error_m)) {
      pitch_right_rad = candidate_pitch_rad;
      right_residual = candidate_residual;
    } else {
      pitch_left_rad = candidate_pitch_rad;
      left_residual = candidate_residual;
    }
  }

  return PitchFlytime{
      .pitch = best_pitch_rad,
      .fly_time = best_residual.fly_time_sec,
  };
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::BallisticTrajectorySolver::resolveRk45(
    double target_distance_m, double target_height_m,
    double muzzle_velocity_mps) const {
  if (target_distance_m <= 0.0 || muzzle_velocity_mps <= 0.0)
    return std::nullopt;

  const double min_pitch_rad =
      tools::angle2Radian(config_.gimbal_pitch_min_degree);
  const double max_pitch_rad =
      tools::angle2Radian(config_.gimbal_pitch_max_degree);

  auto left_residual = evaluatePitchByRk45(
      min_pitch_rad, target_distance_m, target_height_m, muzzle_velocity_mps);
  auto right_residual = evaluatePitchByRk45(
      max_pitch_rad, target_distance_m, target_height_m, muzzle_velocity_mps);
  if (left_residual.valid && right_residual.valid &&
      hasDifferentSign(left_residual.height_error_m,
                       right_residual.height_error_m)) {
    auto solved = solvePitchByHybridMethod(min_pitch_rad, max_pitch_rad,
                                           target_distance_m, target_height_m,
                                           muzzle_velocity_mps);
    if (solved.has_value())
      return solved;
  }

  double last_pitch_rad = min_pitch_rad;
  auto last_residual = left_residual;
  const double pitch_step_rad =
      (max_pitch_rad - min_pitch_rad) / kPitchBracketSampleCount;
  for (int sample_index = 1; sample_index <= kPitchBracketSampleCount;
       ++sample_index) {
    const double pitch_rad = min_pitch_rad + pitch_step_rad * sample_index;
    auto residual = evaluatePitchByRk45(pitch_rad, target_distance_m,
                                        target_height_m, muzzle_velocity_mps);
    if (residual.valid && last_residual.valid &&
        hasDifferentSign(last_residual.height_error_m,
                         residual.height_error_m)) {
      auto solved =
          solvePitchByHybridMethod(last_pitch_rad, pitch_rad, target_distance_m,
                                   target_height_m, muzzle_velocity_mps);
      if (solved.has_value())
        return solved;
    }
    last_pitch_rad = pitch_rad;
    last_residual = residual;
  }

  auto analytic_solution =
      resolveParabola(target_distance_m, target_height_m, muzzle_velocity_mps);
  if (analytic_solution.has_value()) {
    auto rk45_at_analytic_pitch =
        evaluatePitchByRk45(analytic_solution->pitch, target_distance_m,
                            target_height_m, muzzle_velocity_mps);
    if (rk45_at_analytic_pitch.valid) {
      LOG_DEBUG(
          logger_,
          "[BallisticTrajectorySolver]: RK45 fallback to parabola pitch.");
      return PitchFlytime{
          .pitch = analytic_solution->pitch,
          .fly_time = rk45_at_analytic_pitch.fly_time_sec,
      };
    }
    return analytic_solution;
  }

  LOG_DEBUG(logger_, "[BallisticTrajectorySolver]: RK45 solve failed.");
  return std::nullopt;
}

std::optional<tools::ballistic::PitchFlytime>
tools::ballistic::BallisticTrajectorySolver::resolveParabola(
    double target_distance_m, double target_height_m,
    double muzzle_velocity_mps, double g, double gimbal_pitch_min_degree,
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
tools::ballistic::BallisticTrajectorySolver::resolveParabola(
    double target_distance_m, double target_height_m,
    double muzzle_velocity_mps) const {
  return resolveParabola(
      target_distance_m, target_height_m, muzzle_velocity_mps, config_.g,
      config_.gimbal_pitch_min_degree, config_.gimbal_pitch_max_degree);
}
