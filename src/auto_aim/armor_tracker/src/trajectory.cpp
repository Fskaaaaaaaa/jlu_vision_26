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

std::optional<auto_aim::YawPitchFlytime> auto_aim::Trajectory::resolveYawPitch(
    double bullet_speed, const Eigen::Vector3d &aim_position, bool use_rk45,
    double odom_x, double odom_y) {
  auto distance_x = aim_position.x() - odom_x;
  auto distance_y = aim_position.y() - odom_y;
  auto distance = std::hypot(distance_x, distance_y);
  auto yaw = std::atan2(distance_y, distance_x);
  auto pitch_flytime_opt = solver_.resolvePitchFlyTime(
      bullet_speed, distance, aim_position.z(),
      use_rk45 ? tools::ballistic::Method::rk45
               : tools::ballistic::Method::parabola);
  if (!pitch_flytime_opt.has_value())
    return std::nullopt;
  return YawPitchFlytime{
      .yaw = yaw,
      .pitch = pitch_flytime_opt.value().pitch,
      .fly_time = pitch_flytime_opt.value().fly_time,
  };
}

std::pair<auto_aim::ArmorPositionYaw, auto_aim::ArmorIndex>
auto_aim::Trajectory::getClosestArmorIndexFromTarget(const TargetState &state) {
  auto armors = state.armors();
  double min_distance = DBL_MAX;
  ArmorPositionYaw selected_armor;
  ArmorIndex selected_index;
  for (int i = 0; i < armors.size(); i++) {
    const auto &armor = armors.at(i);
    auto distance = std::hypot(armor.position.x(), armor.position.y());
    if (distance < min_distance) {
      selected_armor = armor;
      selected_index = static_cast<ArmorIndex>(i);
    }
  }
  return {selected_armor, selected_index};
}

double auto_aim::Trajectory::calculateAimError(
    double yaw, double pitch, double v0, double fly_time,
    const Eigen::Vector3d &aim_position, bool use_rk45) {
  auto bullet_position =
      use_rk45
          ? tools::ballistic::rk45::getState3DByT(
                solver_.getBarrelStateFromPitch(pitch, v0), yaw, fly_time,
                config_.ballistic_conf.time_step, config_.ballistic_conf.k,
                config_.ballistic_conf.g)
          : tools::ballistic::parabola::getState3DByT(
                {0, 0, pitch, v0}, yaw, fly_time, config_.ballistic_conf.g);
  return (bullet_position.position - aim_position).norm();
}

std::optional<auto_aim::YawPitchFlytime>
auto_aim::Trajectory::resolveTarget(const TargetState &state,
                                    double bullet_speed,
                                    double delay_time_image_to_now_sec,
                                    bool use_rk45, bool iterative_fly_time) {
  auto state_d = state.predict(delay_time_image_to_now_sec);
  auto [armor, index] = getClosestArmorIndexFromTarget(state_d);
  if (!iterative_fly_time)
    return resolveYawPitch(bullet_speed, armor.position, use_rk45);
  double error = DBL_MAX;
  int iteration_count = 0;
  int switch_armor_count = 0;
  YawPitchFlytime result;
  ArmorIndex last_index;
  while (error >= config_.aim_ok_error_m) {
    auto result_opt = resolveYawPitch(bullet_speed, armor.position, use_rk45);
    if (!result_opt.has_value() ||
        iteration_count++ > config_.max_aim_iterate_count) {
      // 无解情况直接返回
      LOG_WARNING(logger_,
                  "[Trajectory]: Fail to solve YawPitchFlytime in {} "
                  "iterations, error {}.",
                  iteration_count, error);
      return std::nullopt;
    }
    result = result_opt.value();
    state_d = state.predict(delay_time_image_to_now_sec + result.fly_time);
    std::tie(armor, index) = getClosestArmorIndexFromTarget(state_d);
    last_index = index;
    if (last_index != index)
      if (++switch_armor_count >= config_.max_aim_switch_armor_count) {
        LOG_WARNING(logger_, "[Trajectory]: Armor select unstable!");
        return result;
      }
    error = calculateAimError(result.yaw, result.pitch, bullet_speed,
                              result.fly_time, armor.position, use_rk45);
  }
  return result;
}
