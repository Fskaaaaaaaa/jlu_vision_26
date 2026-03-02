#include "trajectory.hpp"
#include "configs.hpp"
#include "math/angle_tools.hpp"
#include "math/ballistic_trajectory.hpp"
#include "types.hpp"

#include "quill/LogMacros.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <csetjmp>
#include <optional>
#include <vector>

auto_aim::Trajectory::Trajectory(quill::Logger *logger,
                                 const TrajectoryConfig &config)
    : logger_(logger), config_(config),
      solver_(tools::BallisticParams{
          .g = config_.g,
          .k = config_.k,
          .time_step = config_.time_step,
          .barrel_length = config_.length_gimbal_to_barrel,
      }) {}

std::optional<tools::PitchFlytime>
auto_aim::Trajectory::resolvePitch(double bullet_speed, double target_distance,
                                   double target_height,
                                   bool use_analytical_solution) {
  auto result_opt = solver_.getAnalyticalAimingSolution(
      bullet_speed, target_distance, target_height);
  if (!result_opt.has_value())
    return std::nullopt;
  if (use_analytical_solution)
    return result_opt;
  tools::PitchFlytime result = result_opt.value();
  struct {
    double min;
    double max;
  } search_range = {
      std::max(result.pitch -
                   tools::angle2Radian(config_.half_search_range_degree),
               tools::angle2Radian(config_.gimbal_pitch_min_degree)),
      std::min(result.pitch +
                   tools::angle2Radian(config_.half_search_range_degree),
               tools::angle2Radian(config_.gimbal_pitch_max_degree)),
  };
  auto cost_function = [&, this](const double &pitch_theta) {
    result.fly_time = 0;
    tools::BallisticState2D pos_d{.0, .0, pitch_theta, bullet_speed};
    pos_d = solver_.transformPos2DGimbalToBarrel(pos_d);
    while (pos_d.distance < target_distance) {
      pos_d = solver_.rk45SingleStep(pos_d);
      result.fly_time += solver_.params_.time_step;
      if (result.fly_time > config_.max_fly_time)
        // 这里对异常迭代的处理不够完善，但反正二分法也发散不了，用不着反三角兜底
        throw std::runtime_error("fly time too long!");
    }
    return pos_d.height - target_height;
    // 二分法每次迭代损失函数误差小于零区间会向右收缩，反之则会向左收缩
    // 假设收敛(0,30)角度内的云台pitch，打低了error为负，故收敛右半区间，仰角加大
  };
  try {
    if (std::signbit(cost_function(search_range.min)) ==
        std::signbit(cost_function(search_range.max))) {
      LOG_WARNING(logger_,
                  "aiming angle exceeding the maximum range of the gimbal");
      return std::nullopt;
    } else {
      result.pitch =
          this->bisection_
              .find(search_range.min, search_range.max, cost_function,
                    config_.max_pitch_iterate_count, config_.min_pitch_error)
              .first;
    }
  } catch (const std::exception &e) {
    LOG_WARNING(logger_, "{}", e.what());
    return std::nullopt;
  }
  return result;
}

std::optional<auto_aim::YawPitchFlytime> auto_aim::Trajectory::resolveYawPitch(
    double bullet_speed, const Eigen::Vector3d &aim_position,
    bool use_analytical_solution, double odom_x, double odom_y) {
  auto distance_x = aim_position.x() - odom_x;
  auto distance_y = aim_position.y() - odom_y;
  auto distance = std::hypot(distance_x, distance_y);
  auto yaw = std::atan2(distance_y, distance_x);
  auto pitch_flytime_opt = resolvePitch(
      bullet_speed, distance, aim_position.z(), use_analytical_solution);
  if (!pitch_flytime_opt.has_value())
    return std::nullopt;
  return YawPitchFlytime{
      .yaw = yaw,
      .pitch = pitch_flytime_opt.value().pitch,
      .fly_time = pitch_flytime_opt.value().fly_time,
  };
}

std::pair<auto_aim::Armor, auto_aim::ArmorIndex>
auto_aim::Trajectory::getClosestArmorIndex(const TargetStatus &status,
                                           double odom_x, double odom_y) {
  auto armors = status.armors();
  double min_distance = DBL_MAX;
  Armor selected_armor;
  ArmorIndex selected_index;
  for (int i = 0; i < armors.size(); i++) {
    const auto &armor = armors.at(i);
    auto distance =
        std::hypot(armor.position.x() - odom_x, armor.position.y() - odom_y);
    if (distance < min_distance) {
      selected_armor = armor;
      selected_index = static_cast<ArmorIndex>(i);
    }
  }
  return {selected_armor, selected_index};
}

std::optional<auto_aim::YawPitchFlytime> auto_aim::Trajectory::resolveTarget(
    const TargetStatus &status, double bullet_speed,
    double delay_time_image_to_now_sec, bool use_analytical_solution,
    bool iterative_fly_time, double odom_x, double odom_y) {
  if (!iterative_fly_time) {
    auto [armor, index] = getClosestArmorIndex(
        status.predict(delay_time_image_to_now_sec), odom_x, odom_y);
    return resolveYawPitch(bullet_speed, armor.position,
                           use_analytical_solution);
  }
  // NOTE:
  // 斜观测spin有时选板会反复在两个armor间横跳，无法收敛，此时应该避免打弹
  TargetStatus status_d = status;
  double error = DBL_MAX;
  int iteration_count = 0;
  int switch_armor_count = 0;
  YawPitchFlytime result;
  status_d = status.predict(delay_time_image_to_now_sec);
  auto [armor, index] = getClosestArmorIndex(status_d, odom_x, odom_y);
  ArmorIndex last_index;
  while (error >= config_.aim_ok_error_m) {
    if (switch_armor_count > config_.max_aim_switch_armor_count) {
      LOG_WARNING(logger_, "[Trajectory]: Aim switch armor {} times! abandon.",
                  switch_armor_count);
      return std::nullopt;
    }
    if (auto result_opt = resolveYawPitch(bullet_speed, armor.position,
                                          use_analytical_solution);
        !result_opt.has_value() ||
        iteration_count++ > config_.max_aim_iterate_count) {
      // 无解情况直接返回
      LOG_WARNING(logger_,
                  "[Trajectory]: fail to solve YawPitchFlytime in {} "
                  "iterations, error {}.",
                  iteration_count, error);
      return std::nullopt;
    } else {
      result = result_opt.value();
    }
    // 使用fly_time更新目标预测位置
    status_d = status.predict(delay_time_image_to_now_sec + result.fly_time);
    last_index = index;
    std::tie(armor, index) = getClosestArmorIndex(status_d, odom_x, odom_y);
    if (last_index != index) {
      switch_armor_count++;
    }
    auto state = tools::BallisticState2D{
        0,
        0,
        result.pitch,
        bullet_speed,
    };
    auto shoot_position =
        use_analytical_solution
            ? solver_.getPosXyzByT(state, result.yaw, result.fly_time, true)
            : solver_.getPosXyzByT(
                  solver_.transformPos2DGimbalToBarrel(tools::BallisticState2D{
                      0, 0, result.pitch, bullet_speed}),
                  result.yaw, result.fly_time);
    error = (shoot_position - armor.position).norm();
    LOG_TRACE_L2(logger_,
                 "iteration_count {}, error {}, positoin: ({}, "
                 "{}, {}), pitch "
                 "{}, yaw {}, fly_time {}.",
                 iteration_count, error, armor.position.x(), armor.position.y(),
                 armor.position.z(), result.pitch, result.yaw, result.fly_time);
  }
  LOG_DEBUG(logger_,
            "success solve YawPitchFlytime in {} iterations, error {}.",
            iteration_count, error);
  return result;
}
