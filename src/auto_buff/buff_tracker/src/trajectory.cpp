#include "trajectory.hpp"
#include "configs.hpp"
#include "math/ballistic_trajectory.hpp"
#include "types.hpp"

#include "quill/LogMacros.h"

#include <optional>
#include <vector>

auto_buff::Trajectory::Trajectory(quill::Logger *logger,
                                  const TrajectoryConfig &config)
    : logger_(logger), config_(config),
      solver_(logger_, config_.ballistic_conf) {}

std::optional<auto_buff::BuffBladeIndex>
auto_buff::Trajectory::selectBlade(const BuffState &state) const {
  std::vector<BuffBladeIndex> inactive_indices;
  for (int i = 0; i < state.inactivated_flag.size(); i++)
    if (state.inactivated_flag.at(i))
      inactive_indices.emplace_back(static_cast<BuffBladeIndex>(i));

  static auto change_count{0};
  static std::optional<BuffBladeIndex> last_selected{std::nullopt};
  if (inactive_indices.empty()) {
    if (!last_selected.has_value())
      return std::nullopt;
    // 存在上次选中的叶片，但当前无可用叶片
    if (change_count < config_.blade_select_change_count_thres) {
      ++change_count;
      return last_selected.value();
    } else {
      // 超过阈值，清空记忆并返回空
      last_selected = std::nullopt;
      change_count = 0;
      return std::nullopt;
    }
  }

  auto index = inactive_indices.front();

  if (!last_selected.has_value()) {
    last_selected = index;
    change_count = 0;
    return index;
  }
  if (index == last_selected.value())
    return index;
  if (change_count >= config_.blade_select_change_count_thres) {
    // 切换到新叶片并重置计数
    last_selected = index;
    change_count = 0;
    return index;
  } else {
    // 增加计数并继续使用上次叶片
    ++change_count;
    return last_selected.value();
  }
}

std::optional<auto_buff::YawPitchFlyTimeIndex>
auto_buff::Trajectory::solveBlade(BuffBladeIndex blade_index,
                                  const BuffState &buff_state,
                                  double bullet_speed,
                                  bool iterative_fly_time) const {
  double error = DBL_MAX;
  double fly_time{0};
  int iterative_count{0};
  YawPitchFlyTimeIndex result;
  do {
    if (++iterative_count > config_.max_aim_iterate_count) {
      LOG_WARNING(logger_, "[Trajectory]: Reach max iterate number!");
      return std::nullopt;
    }
    auto blade =
        buff_state.predict(fly_time).blades().at(static_cast<int>(blade_index));
    Eigen::Vector3d blade_hit_position = blade.getHitPosition();
    auto result_opt = solver_.resolvePitchFlyTime(
        std::hypot(blade_hit_position.x(), blade_hit_position.y()),
        blade_hit_position.z(),
        bullet_speed); // 默认抛物线
    if (!result_opt.has_value())
      return std::nullopt;
    auto yaw = std::atan2(blade_hit_position.y(), blade_hit_position.x());
    auto pitch = result_opt->pitch;
    fly_time = result_opt->fly_time;
    auto bullet_hit_position = tools::ballistic::parabola::getState3DByT(
        {0, 0, pitch, bullet_speed}, yaw, fly_time, config_.ballistic_conf.g);
    Eigen::Vector3d target_position = buff_state.predict(fly_time)
                                          .blades()
                                          .at(static_cast<int>(blade_index))
                                          .getHitPosition();
    error = (target_position - blade_hit_position).norm();
    result.yaw = yaw;
    result.pitch = pitch;
    result.fly_time = fly_time;
  } while (iterative_fly_time && error >= config_.aim_ok_error_m);
  result.index = blade_index;
  return result;
}

std::optional<auto_buff::YawPitchFlyTimeIndex>
auto_buff::Trajectory::solveBuff(const BuffState &buff_state,
                                 double bullet_speed) {
  if (bullet_speed < config_.min_bullet_speed ||
      bullet_speed > config_.max_bullet_speed) {
    LOG_DEBUG(logger_,
              "[Trajectory]: Abnormal bullet speed {}, use default {}.",
              bullet_speed, config_.default_bullet_speed);
    bullet_speed = config_.default_bullet_speed;
  }
  auto selected_index_opt = selectBlade(buff_state);
  if (!selected_index_opt.has_value())
    return std::nullopt;
  return solveBlade(selected_index_opt.value(), buff_state, bullet_speed,
                    config_.iterative_fly_time);
}
