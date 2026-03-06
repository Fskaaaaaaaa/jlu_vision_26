#include "trajectory.hpp"
#include "configs.hpp"
#include "math/ballistic_models.hpp"
#include "math/ballistic_trajectory.hpp"
#include "types.hpp"

#include "quill/LogMacros.h"

#include <cfloat>
#include <cmath>
#include <optional>

auto_aim::Trajectory::Trajectory(quill::Logger *logger,
                                 const TrajectoryConfig &config)
    : logger_(logger), config_(config),
      solver_(logger_, config_.ballistic_conf) {}

std::optional<auto_aim::YawPitchFlytime>
auto_aim::Trajectory::solveYawPitchForArmorPosition(
    double bullet_speed_mps, const Eigen::Vector3d &armor_position_m,
    bool use_rk45, double odom_x_m, double odom_y_m) {
  const double target_x_m = armor_position_m.x() - odom_x_m;
  const double target_y_m = armor_position_m.y() - odom_y_m;
  const double target_distance_m = std::hypot(target_x_m, target_y_m);
  const double target_height_m = armor_position_m.z();
  const double target_yaw_rad = std::atan2(target_y_m, target_x_m);
  auto pitch_and_flytime = solver_.resolvePitchFlyTime(
      target_distance_m, target_height_m, bullet_speed_mps,
      use_rk45 ? tools::ballistic::Method::rk45
               : tools::ballistic::Method::parabola);
  if (!pitch_and_flytime.has_value())
    return std::nullopt;
  return YawPitchFlytime{
      .yaw = target_yaw_rad,
      .pitch = pitch_and_flytime->pitch,
      .fly_time = pitch_and_flytime->fly_time,
  };
}

std::pair<auto_aim::ArmorPositionYaw, auto_aim::ArmorIndex>
auto_aim::Trajectory::getClosestArmorIndexFromTarget(const TargetState &state) {
  const auto armors = state.armors();
  double closest_distance_m = DBL_MAX;
  ArmorPositionYaw closest_armor;
  ArmorIndex closest_armor_index = ArmorIndex::_0;
  for (std::size_t index = 0; index < armors.size(); ++index) {
    const auto &armor = armors.at(index);
    const double distance_to_gimbal_m =
        std::hypot(armor.position.x(), armor.position.y());
    if (distance_to_gimbal_m < closest_distance_m) {
      closest_distance_m = distance_to_gimbal_m;
      closest_armor = armor;
      closest_armor_index = static_cast<ArmorIndex>(index);
    }
  }
  return {closest_armor, closest_armor_index};
}

double auto_aim::Trajectory::evaluateImpactPositionError(
    double yaw_rad, double pitch_rad, double bullet_speed_mps,
    double fly_time_sec, const Eigen::Vector3d &armor_position_m,
    bool use_rk45) {
  auto bullet_position =
      use_rk45
          ? tools::ballistic::rk45::getState3DByT(
                solver_.getBarrelStateFromPitch(pitch_rad, bullet_speed_mps),
                yaw_rad, fly_time_sec,
                config_.ballistic_conf.time_step, config_.ballistic_conf.k,
                config_.ballistic_conf.g)
          : tools::ballistic::parabola::getState3DByT(
                {0, 0, pitch_rad, bullet_speed_mps}, yaw_rad, fly_time_sec,
                config_.ballistic_conf.g);
  return (bullet_position.position - armor_position_m).norm();
}

std::optional<auto_aim::YawPitchFlytime>
auto_aim::Trajectory::resolveTarget(const TargetState &state,
                                    double bullet_speed_mps,
                                    double delay_time_image_to_now_sec,
                                    bool use_rk45, bool iterative_fly_time) {
  auto predicted_target_state = state.predict(delay_time_image_to_now_sec);
  auto [selected_armor, selected_armor_index] =
      getClosestArmorIndexFromTarget(predicted_target_state);
  if (!iterative_fly_time)
    return solveYawPitchForArmorPosition(bullet_speed_mps, selected_armor.position,
                                         use_rk45);

  double aiming_error_m = DBL_MAX;
  int iteration_index = 0;
  int switched_armor_count = 0;
  std::optional<YawPitchFlytime> solved_yaw_pitch;
  while (aiming_error_m >= config_.aim_ok_error_m) {
    auto current_solution =
        solveYawPitchForArmorPosition(bullet_speed_mps, selected_armor.position,
                                      use_rk45);
    if (!current_solution.has_value() ||
        iteration_index++ > config_.max_aim_iterate_count) {
      LOG_WARNING(logger_,
                  "[Trajectory]: Fail to solve YawPitchFlytime in {} "
                  "iterations, error {}.",
                  iteration_index, aiming_error_m);
      return std::nullopt;
    }
    solved_yaw_pitch = current_solution;
    predicted_target_state =
        state.predict(delay_time_image_to_now_sec + current_solution->fly_time);
    auto [next_armor, next_armor_index] =
        getClosestArmorIndexFromTarget(predicted_target_state);
    if (next_armor_index != selected_armor_index &&
        ++switched_armor_count >= config_.max_aim_switch_armor_count) {
      LOG_WARNING(logger_, "[Trajectory]: Armor select unstable!");
      return current_solution;
    }
    selected_armor = next_armor;
    selected_armor_index = next_armor_index;
    aiming_error_m =
        evaluateImpactPositionError(current_solution->yaw,
                                    current_solution->pitch, bullet_speed_mps,
                                    current_solution->fly_time,
                                    selected_armor.position, use_rk45);
  }
  return solved_yaw_pitch;
}
