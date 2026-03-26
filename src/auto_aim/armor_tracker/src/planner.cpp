#include "planner.hpp"
#include "configs.hpp"
#include "math/angle_tools.hpp"
#include "math/threshold_tools.hpp"
#include "msgs/AimCommand.hpp"
#include "trajectory.hpp"
#include "types.hpp"
#include "types/ArmorType.hpp"

#include "quill/LogMacros.h"

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <utility>

auto_aim::Planner::Planner(quill::Logger *logger, const PlannerConfig &config)
    : logger_(logger), config_(config),
      trajectory_solver_(logger_, config_.trajectory_conf),
      aim0_predict_time_(0) {
  trajectory_horizon_ = config_.trajectory_half_horizon * 2;
  bullet_id_ = 0;
  {
    Eigen::MatrixXd A{{1, config_.dt_sec}, {0, 1}};
    Eigen::MatrixXd B{{0}, {config_.dt_sec}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(config_.Q_yaw.data());
    Eigen::Matrix<double, 1, 1> R(config_.R_yaw);
    tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1,
               trajectory_horizon_, 0);

    Eigen::MatrixXd x_min =
        Eigen::MatrixXd::Constant(2, trajectory_horizon_, -1e17);
    Eigen::MatrixXd x_max =
        Eigen::MatrixXd::Constant(2, trajectory_horizon_, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(
        1, trajectory_horizon_ - 1, -config_.max_yaw_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(
        1, trajectory_horizon_ - 1, config_.max_yaw_acc);
    tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);
    yaw_solver_->settings->max_iter = 10;
  }
  {
    Eigen::MatrixXd A{{1, config_.dt_sec}, {0, 1}};
    Eigen::MatrixXd B{{0}, {config_.dt_sec}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(config_.Q_pitch.data());
    Eigen::Matrix<double, 1, 1> R(config_.R_pitch);
    tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2,
               1, trajectory_horizon_, 0);

    Eigen::MatrixXd x_min =
        Eigen::MatrixXd::Constant(2, trajectory_horizon_, -1e17);
    Eigen::MatrixXd x_max =
        Eigen::MatrixXd::Constant(2, trajectory_horizon_, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(
        1, trajectory_horizon_ - 1, -config_.max_pitch_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(
        1, trajectory_horizon_ - 1, config_.max_pitch_acc);
    tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);
    pitch_solver_->settings->max_iter = 10;
  }
}

msgs::AimCommand auto_aim::Planner::plan(
    const TargetState &target_state,
    const std::chrono::system_clock::time_point &target_stamp,
    const msgs::GimbalInfo &gimbal_info) {
  const double dt_image_to_now_sec =
      config_.no_predict
          ? 0.0
          : std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::system_clock::now() - target_stamp +
                std::chrono::milliseconds{config_.predict_offset_ms})
                .count();
  // XXX
  // 当切换目标时清空历史轨迹缓存
  // 不太合理，不适用于团战反复切换目标或一个目标反复丢失重置的情况
  // 但最多也就影响0.5s，先这样试一下
  if (config_.use_history_traj_cache)
    updateHistoryTrajectory(gimbal_info.yaw, gimbal_info.yaw_vel,
                            gimbal_info.pitch, gimbal_info.pitch_vel,
                            target_state.type);
  auto cmd =
      shouldAimCenter(target_state)
          ? aimCenter(target_state, dt_image_to_now_sec,
                      gimbal_info.bullet_speed)
          : aimMPC(target_state, dt_image_to_now_sec, gimbal_info.bullet_speed);
  if (config_.consider_gimbal_response && cmd.control) {
    cmd.fire = (std::hypot(cmd.target_yaw - gimbal_info.yaw,
                           cmd.target_pitch - gimbal_info.pitch) <
                config_.fire_thresh);
  }
  return cmd;
}

bool auto_aim::Planner::shouldAimCenter(const TargetState &target_state) {
  if (!config_.enable_aim_center)
    return false;
  static types::ArmorType last_target_type{target_state.type};
  bool reset{false};
  if (last_target_type != target_state.type) {
    last_target_type = target_state.type;
    reset = true;
  }
  static tools::HysteresisComparator comp_vyaw{
      config_.aim_center_vyaw_thres_high, config_.aim_center_vyaw_thres_low,
      false};
  static tools::HysteresisComparator comp_distance{
      config_.aim_center_distance_high, config_.aim_center_distance_low, true};
  return comp_vyaw(target_state.center_vyaw, reset) &&
         comp_distance(target_state.center_position.norm(), reset);
}

std::pair<auto_aim::ArmorPositionYaw, auto_aim::ArmorIndex>
auto_aim::Planner::selectAimingArmor(const TargetState &target_state) const {
  return trajectory_solver_.selectArmorForAiming(target_state);
}

msgs::AimCommand auto_aim::Planner::aimMPC(const TargetState &target_state,
                                           double dt_image_to_now_sec,
                                           double bullet_speed_mps) {
  if (bullet_speed_mps < config_.min_bullet_speed ||
      bullet_speed_mps > config_.max_bullet_speed) {
    LOG_DEBUG(logger_, "[Planner]: Abnormal bullet speed {}, use default {}.",
              bullet_speed_mps, config_.default_bullet_speed);
    bullet_speed_mps = config_.default_bullet_speed;
  }

  // 构建参考轨迹
  AimTrajectoryReference reference;
  try {
    reference = buildReferenceTrajectory(target_state, dt_image_to_now_sec,
                                         bullet_speed_mps);
  } catch (const std::exception &e) {
    LOG_WARNING(logger_, "{}, bullet_speed: {}.", e.what(), bullet_speed_mps);
    return {.control = false};
  }

  // 依据参考轨迹优化云台轨迹
  Eigen::VectorXd initial_state(2);
  initial_state << reference.state_reference(0, 0),
      reference.state_reference(1, 0);
  tiny_set_x0(yaw_solver_, initial_state);
  yaw_solver_->work->Xref =
      reference.state_reference.block(0, 0, 2, trajectory_horizon_);
  tiny_solve(yaw_solver_);

  initial_state << reference.state_reference(2, 0),
      reference.state_reference(3, 0);
  tiny_set_x0(pitch_solver_, initial_state);
  pitch_solver_->work->Xref =
      reference.state_reference.block(2, 0, 2, trajectory_horizon_);
  tiny_solve(pitch_solver_);

  msgs::AimCommand cmd;
  cmd.control = true;
  cmd.target_yaw = tools::limitRadian(
      reference.state_reference(0, config_.trajectory_half_horizon) +
      reference.center_reference_yaw_rad + config_.yaw_offset);
  cmd.target_pitch =
      reference.state_reference(2, config_.trajectory_half_horizon) +
      config_.pitch_offset;
  cmd.yaw = tools::limitRadian(
      yaw_solver_->work->x(0, config_.trajectory_half_horizon) +
      reference.center_reference_yaw_rad + config_.yaw_offset);
  cmd.yaw_vel = yaw_solver_->work->x(1, config_.trajectory_half_horizon);
  cmd.yaw_acc = yaw_solver_->work->u(0, config_.trajectory_half_horizon);
  cmd.pitch = pitch_solver_->work->x(0, config_.trajectory_half_horizon) +
              config_.pitch_offset;
  cmd.pitch_vel = pitch_solver_->work->x(1, config_.trajectory_half_horizon);
  cmd.pitch_acc = pitch_solver_->work->u(0, config_.trajectory_half_horizon);
  cmd.fire =
      std::hypot(reference.state_reference(0, config_.trajectory_half_horizon +
                                                  config_.shoot_offset) -
                     yaw_solver_->work->x(0, config_.trajectory_half_horizon +
                                                 config_.shoot_offset),
                 reference.state_reference(2, config_.trajectory_half_horizon +
                                                  config_.shoot_offset) -
                     pitch_solver_->work->x(0, config_.trajectory_half_horizon +
                                                   config_.shoot_offset)) <
      config_.fire_thresh;
  cmd.bullet_id = bullet_id_++;
  return cmd;
}

msgs::AimCommand auto_aim::Planner::aimCenter(const TargetState &target_state,
                                              double dt_image_to_now_sec,
                                              double bullet_speed_mps) {
  auto aim_opt = trajectory_solver_.resolveTarget(
      target_state, bullet_speed_mps, dt_image_to_now_sec, config_.rk45_yaw0,
      config_.iterative_yaw0);
  if (!aim_opt.has_value())
    return {.control = false};
  auto aim = aim_opt.value().yaw_pitch_fly_time;
  msgs::AimCommand cmd;
  cmd.control = true;
  cmd.target_yaw = tools::limitRadian(aim.yaw + config_.yaw_offset);
  cmd.target_pitch = aim.pitch + config_.pitch_offset;
  cmd.yaw = tools::limitRadian(std::atan2(target_state.center_position.y(),
                                          target_state.center_position.x()) +
                               config_.yaw_offset);
  cmd.yaw_vel = 0;
  cmd.yaw_acc = 0;
  cmd.pitch = aim.pitch + config_.pitch_offset;
  cmd.pitch_vel = 0;
  cmd.pitch_acc = 0;
  cmd.fire = (std::abs(cmd.yaw - cmd.target_yaw) < config_.fire_thresh);
  cmd.bullet_id = bullet_id_++;
  return cmd;
}

std::optional<auto_aim::TargetAimSolution>
auto_aim::Planner::solveAim(const TargetState &target_state,
                            double dt_image_to_now_sec, double bullet_speed_mps,
                            bool use_rk45, bool iterative_fly_time,
                            std::optional<ArmorIndex> &preferred_armor_index) {
  auto solution = trajectory_solver_.resolveTarget(
      target_state, bullet_speed_mps, dt_image_to_now_sec, use_rk45,
      iterative_fly_time, preferred_armor_index);
  if (!solution.has_value() && use_rk45) {
    solution = trajectory_solver_.resolveTarget(
        target_state, bullet_speed_mps, dt_image_to_now_sec, false,
        iterative_fly_time, preferred_armor_index);
  }
  if (solution.has_value()) {
    preferred_armor_index = solution->selected_armor_index;
  }
  return solution;
}

void auto_aim::Planner::updateHistoryTrajectory(double yaw, double yaw_vel,
                                                double pitch, double pitch_vel,
                                                types::ArmorType type) {
  static auto last_type{type};
  if (last_type != type) {
    history_traj_cache_.clear();
    last_type = type;
  }
  history_traj_cache_.push_back({yaw, yaw_vel, pitch, pitch_vel});
  if (history_traj_cache_.size() > config_.trajectory_half_horizon)
    history_traj_cache_.pop_front();
}

auto_aim::AimTrajectoryReference
auto_aim::Planner::buildReferenceTrajectory(const TargetState &target_state,
                                            double dt_image_to_now_sec,
                                            double bullet_speed_mps) {
  std::optional<ArmorIndex> preferred_armor_index;
  // 注意这个是中心时间(瞄准时刻)的aim(不是指瞄中心)
  auto center_time_aim_opt = solveAim(
      target_state, dt_image_to_now_sec, bullet_speed_mps, config_.rk45_yaw0,
      config_.iterative_yaw0, preferred_armor_index);
  if (!center_time_aim_opt.has_value())
    throw std::runtime_error("Unsolvable bullet trajectory!");
  const auto center_aim = center_time_aim_opt.value();
  AimTrajectoryReference reference;
  reference.center_reference_yaw_rad = center_aim.yaw_pitch_fly_time.yaw;
  reference.state_reference = Eigen::MatrixXd(4, trajectory_horizon_);
  aim0_predict_time_.store(center_aim.yaw_pitch_fly_time.fly_time +
                           dt_image_to_now_sec);

  // 将目标反向推算到轨迹起始时刻的状态
  auto target_state_d = target_state.predict(
      -config_.dt_sec * (config_.trajectory_half_horizon + 1));
  auto previous_aim_opt = solveAim(
      target_state_d, dt_image_to_now_sec, bullet_speed_mps, config_.rk45_traj,
      config_.iterative_traj, preferred_armor_index);
  if (!previous_aim_opt.has_value()) {
    ++reference.fallback_sample_count;
    LOG_TRACE_L1(
        logger_,
        "[Planner]: use center-aim fallback for first trajectory sample.");
  }
  auto previous_solution = previous_aim_opt.has_value()
                               ? previous_aim_opt->yaw_pitch_fly_time
                               : center_aim.yaw_pitch_fly_time;
  target_state_d = target_state_d.predict(config_.dt_sec);
  auto current_solution_optional = solveAim(
      target_state_d, dt_image_to_now_sec, bullet_speed_mps, config_.rk45_traj,
      config_.iterative_traj, preferred_armor_index);
  if (!current_solution_optional.has_value()) {
    ++reference.fallback_sample_count;
    LOG_TRACE_L1(
        logger_,
        "[Planner]: use previous-sample fallback for second trajectory "
        "sample.");
  }
  auto current_solution = current_solution_optional.has_value()
                              ? current_solution_optional->yaw_pitch_fly_time
                              : previous_solution;

  auto use_history_cache =
      config_.use_history_traj_cache &&
      (history_traj_cache_.size() == config_.trajectory_half_horizon);

  // 从-half到half构建整条轨迹
  for (int frame_index = 0; frame_index < trajectory_horizon_; ++frame_index) {
    target_state_d = target_state_d.predict(config_.dt_sec);
    YawPitchFlyTime next_solution;
    double yaw_vel, pitch_vel;
    if (use_history_cache && frame_index < config_.trajectory_half_horizon) {
      auto [yaw_cache, yaw_vel_cache, pitch_cache, pitch_vel_cache] =
          history_traj_cache_.at(frame_index);
      next_solution.yaw = yaw_cache;
      next_solution.pitch = pitch_cache;
      yaw_vel = yaw_vel_cache;
      pitch_vel = pitch_vel_cache;
      LOG_TRACE_L1(logger_,
                   "[Planner]: use_history_cache, yaw{}, yaw_vel{}, pitch{}, "
                   "pitch_vel{}, frame_index{}",
                   yaw_cache, yaw_vel_cache, pitch_cache, pitch_vel_cache,
                   frame_index);
    } else {
      auto next_solution_opt = solveAim(
          target_state_d, dt_image_to_now_sec, bullet_speed_mps,
          config_.rk45_traj, config_.iterative_traj, preferred_armor_index);
      next_solution = next_solution_opt.has_value()
                          ? next_solution_opt->yaw_pitch_fly_time
                          : current_solution;
      if (!next_solution_opt.has_value()) {
        ++reference.fallback_sample_count;
        LOG_TRACE_L1(logger_,
                     "[Planner]: use trajectory hold fallback at frame {} when "
                     "aim fails.",
                     frame_index);
      }
      yaw_vel = tools::limitRadian(next_solution.yaw - previous_solution.yaw) /
                (2 * config_.dt_sec);
      pitch_vel = (next_solution.pitch - previous_solution.pitch) /
                  (2 * config_.dt_sec);
    }
    // 将当前帧保存到轨迹中
    reference.state_reference.col(frame_index) << tools::limitRadian(
        current_solution.yaw - reference.center_reference_yaw_rad),
        yaw_vel, current_solution.pitch, pitch_vel;
    previous_solution = current_solution;
    current_solution = next_solution;
  }

  if (reference.fallback_sample_count > 0) {
    LOG_TRACE_L1(logger_, "[Planner]: fallback solve used {}/{} samples.",
                 reference.fallback_sample_count, trajectory_horizon_ + 2);
  }
  return reference;
}
