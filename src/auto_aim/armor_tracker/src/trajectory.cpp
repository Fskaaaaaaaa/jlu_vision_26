#include "trajectory.hpp"
#include "configs.hpp"
#include "math/angle_tools.hpp"
#include "math/ballistic_models.hpp"
#include "math/ballistic_trajectory.hpp"
#include "types.hpp"

#include "quill/LogMacros.h"

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <vector>

auto_aim::Trajectory::Trajectory(quill::Logger *logger,
                                 const TrajectoryConfig &config)
    : logger_(logger), config_(config),
      solver_(logger_, config_.ballistic_conf) {}

double
auto_aim::Trajectory::getArmorFacingAngleAbs(const ArmorPositionYaw &armor) {
  return std::abs(tools::radian2Angle(
      armor.yaw.theta() - std::atan2(armor.position.y(), armor.position.x())));
}

auto_aim::ArmorIndex
auto_aim::Trajectory::selectArmor(const TargetState &state) const {
  auto armors = state.armors();
  auto min_facing_angle_abs = DBL_MAX;
  ArmorIndex selected_index;
  for (auto i = 0; i < armors.size(); i++) {
    auto armor = armors.at(i);
    auto index = static_cast<ArmorIndex>(i);
    auto facing_angle_abs = getArmorFacingAngleAbs(armor);
    if (facing_angle_abs < min_facing_angle_abs) {
      min_facing_angle_abs = facing_angle_abs;
      selected_index = index;
    }
  }
  // 更换目标时重置index由solveTarget方法完成
  if (last_armor_index_.has_value()) {
    auto last_select_armor =
        armors.at(static_cast<int>(last_armor_index_.value()));
    auto facing_angle_diff =
        getArmorFacingAngleAbs(last_select_armor) - min_facing_angle_abs;
    // 新选板明显更正对，切板
    if (facing_angle_diff > config_.armor_switch_facing_degree_diff_thres) {
      return selected_index;
      LOG_TRACE_L2(logger_, "[Trajectory]: Select index {}",
                   static_cast<int>(selected_index));
    }
    return last_armor_index_.value();
  } else { // 第一帧锁定这个目标
    LOG_TRACE_L2(logger_, "[Trajectory]: Select index {}",
                 static_cast<int>(selected_index));
    return selected_index;
  }
}

std::optional<auto_aim::YawPitchFlyTimeIndex>
auto_aim::Trajectory::solveTarget(const TargetState &target_state,
                                  double bullet_speed, bool iterative_fly_time,
                                  bool use_rk45) {
  // XXX: 绝大多数情况没有并发。有点开销但不多
  static auto last_target_type{target_state.type};
  if (last_target_type != target_state.type)
    last_armor_index_ = std::nullopt;
  auto distance = std::hypot(target_state.center_position.x(),
                             target_state.center_position.y()) +
                  config_.flytime0_distance_offset;
  if (distance < 0) {
    LOG_WARNING(logger_, "[Trajectory]: Negative distance!");
    return std::nullopt;
  }
  auto pitch_flytime_opt = this->solver_.resolvePitchFlyTime(
      distance, target_state.center_position.z(), bullet_speed);
  if (!pitch_flytime_opt.has_value())
    return std::nullopt;
  auto selected_index =
      selectArmor(target_state.predict(pitch_flytime_opt->fly_time));
  last_armor_index_ = selected_index;
  auto solution =
      solveArmor(selected_index, pitch_flytime_opt->fly_time, target_state,
                 bullet_speed, iterative_fly_time, use_rk45);
  return solution;
}

std::optional<auto_aim::YawPitchFlyTimeIndex> auto_aim::Trajectory::solveArmor(
    ArmorIndex armor_index, double fly_time, const TargetState &target_state,
    double bullet_speed, bool iterative_fly_time, bool use_rk45) const {
  auto method = use_rk45 ? tools::ballistic::Method::rk45
                         : tools::ballistic::Method::parabola;
  double error = DBL_MAX;
  int iterative_count{0};
  YawPitchFlyTimeIndex result;
  do {
    if (++iterative_count > config_.max_aim_iterate_count) {
      LOG_WARNING(logger_, "[Trajectory]: Reach max iterate number!");
      return std::nullopt;
    }
    auto armor = target_state.predict(fly_time).armors().at(
        static_cast<int>(armor_index));
    if (auto facing_angle_abs = getArmorFacingAngleAbs(armor);
        facing_angle_abs > config_.iterative_max_facing_angle) {
      LOG_WARNING(logger_, "[Trajectory]: facing_angle_abs {}. Abandon!",
                  facing_angle_abs);
      return std::nullopt;
    }
    auto result_opt = solver_.resolvePitchFlyTime(
        std::hypot(armor.position.x(), armor.position.y()), armor.position.z(),
        bullet_speed, method);
    if (!result_opt.has_value())
      return std::nullopt;
    auto yaw = std::atan2(armor.position.y(), armor.position.x());
    auto pitch = result_opt->pitch;
    fly_time = result_opt->fly_time;
    auto hit_position =
        use_rk45 ? tools::ballistic::rk45::getState3DByT(
                       solver_.getBarrelStateFromPitch(pitch, bullet_speed),
                       yaw, fly_time, config_.ballistic_conf.time_step,
                       config_.ballistic_conf.k, config_.ballistic_conf.g)
                 : tools::ballistic::parabola::getState3DByT(
                       {0, 0, pitch, bullet_speed}, yaw, fly_time,
                       config_.ballistic_conf.g);
    Eigen::Vector3d target_position = target_state.predict(fly_time)
                                          .armors()
                                          .at(static_cast<int>(armor_index))
                                          .position;
    error = (target_position - hit_position.position).norm();
    result.yaw = yaw;
    result.pitch = pitch;
    result.fly_time = fly_time;
    result.index = armor_index;
  } while (iterative_fly_time && error >= config_.aim_ok_error_m);
  return result;
}
