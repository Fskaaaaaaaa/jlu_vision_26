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
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

auto_aim::Planner::Planner(quill::Logger *logger, const PlannerConfig &config)
    : logger_(logger), config_(config),
      trajectory_solver_(logger_, config_.trajectory_conf) {
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

// HACK: 应该设置图像和target的缓冲区来可视化瞄准时刻，但开销太大且我是懒狗
std::pair<auto_aim::ArmorPositionYaw, auto_aim::ArmorIndex>
auto_aim::Planner::getAimingArmorIndex(const TargetState &state) const {
  std::scoped_lock lk{cache_mtx_};
  return {state.predict(predict_time_cache_)
              .armors()
              .at(static_cast<int>(selected_index_cache_)),
          selected_index_cache_};
}

msgs::AimCommand auto_aim::Planner::plan(
    const TargetState &target_state,
    const std::chrono::system_clock::time_point &target_stamp,
    const msgs::GimbalInfo &gimbal_info) {
  std::scoped_lock lk{cache_mtx_};
  auto dt_image_to_now_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::system_clock::now() - target_stamp)
          .count();
  predict_time_cache_ =
      dt_image_to_now_sec +
      std::chrono::milliseconds{config_.predict_offset_ms}.count();
  auto cmd =
      aimMPC(target_state.predict(predict_time_cache_),
             gimbal_info.bullet_speed, fly_time_cache_, selected_index_cache_);
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
      config_.aim_center_vyaw_thres_high, config_.aim_center_vyaw_thres_low};
  return comp_vyaw(target_state.center_vyaw, reset);
}

msgs::AimCommand auto_aim::Planner::aimMPC(const TargetState &target_state,
                                           double bullet_speed_mps,
                                           double &fly_time,
                                           ArmorIndex &selected_index) {
  if (bullet_speed_mps < config_.min_bullet_speed ||
      bullet_speed_mps > config_.max_bullet_speed) {
    LOG_DEBUG(logger_, "[Planner]: Abnormal bullet speed {}, use default {}.",
              bullet_speed_mps, config_.default_bullet_speed);
    bullet_speed_mps = config_.default_bullet_speed;
  }

  // 在不撞到加速度限制的情况下，参考轨迹中心点就是规划的瞄准角
  AimTrajectoryReference reference;
  try {
    reference = buildReferenceTrajectory(target_state, bullet_speed_mps);
  } catch (const std::exception &e) {
    LOG_WARNING(logger_, "{}, bullet_speed: {}.", e.what(), bullet_speed_mps);
    return {.control = false};
  }

  // 更新调试用缓存
  fly_time = reference.center_fly_time;
  selected_index = reference.center_selected_index;

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
      reference.center_yaw + config_.yaw_offset);
  cmd.target_pitch =
      reference.state_reference(2, config_.trajectory_half_horizon) +
      config_.pitch_offset;
  cmd.yaw = tools::limitRadian(
      yaw_solver_->work->x(0, config_.trajectory_half_horizon) +
      reference.center_yaw + config_.yaw_offset);
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
  auto aim_opt = trajectory_solver_.solveTarget(
      target_state.predict(dt_image_to_now_sec), bullet_speed_mps,
      config_.rk45_yaw0, config_.iterative_yaw0);
  if (!aim_opt.has_value())
    return {.control = false};
  auto aim = aim_opt.value();
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

auto_aim::AimTrajectoryReference
auto_aim::Planner::buildReferenceTrajectory(const TargetState &target_state,
                                            double bullet_speed_mps) {
  std::optional<ArmorIndex> preferred_armor_index;
  // 注意这个是中心时间(子弹发射时刻)的aim(不是指瞄中心)
  auto center_time_aim_opt = this->trajectory_solver_.solveTarget(
      target_state, bullet_speed_mps, config_.iterative_yaw0,
      config_.rk45_yaw0);
  if (!center_time_aim_opt.has_value())
    throw std::runtime_error("Unsolvable bullet trajectory!");
  auto center_aim = center_time_aim_opt.value();
  AimTrajectoryReference reference;
  reference.center_yaw = center_aim.yaw;
  reference.center_fly_time = center_aim.fly_time;
  reference.center_selected_index = center_aim.index;
  reference.state_reference = Eigen::MatrixXd(4, trajectory_horizon_);

  // 将目标反向推算到轨迹起始时刻的状态
  auto target_state_d = target_state.predict(
      -config_.dt_sec * (config_.trajectory_half_horizon + 1));
  auto previous_aim_opt =
      trajectory_solver_.solveTarget(target_state_d, bullet_speed_mps,
                                     config_.iterative_traj, config_.rk45_traj);
  if (!previous_aim_opt.has_value()) {
    reference.fallback_sample_count++;
    LOG_TRACE_L1(
        logger_,
        "[Planner]: use center-aim fallback for first trajectory sample.");
  }
  auto previous_solution =
      previous_aim_opt.has_value() ? previous_aim_opt.value() : center_aim;
  target_state_d = target_state_d.predict(config_.dt_sec);
  auto current_solution_optional =
      trajectory_solver_.solveTarget(target_state_d, bullet_speed_mps,
                                     config_.rk45_traj, config_.iterative_traj);
  if (!current_solution_optional.has_value()) {
    reference.fallback_sample_count++;
    LOG_TRACE_L1(
        logger_,
        "[Planner]: use previous-sample fallback for second trajectory "
        "sample.");
  }
  auto current_solution = current_solution_optional.has_value()
                              ? current_solution_optional.value()
                              : previous_solution;

  // 从-half到half构建整条轨迹
  for (int frame_index = 0; frame_index < trajectory_horizon_; ++frame_index) {
    target_state_d = target_state_d.predict(config_.dt_sec);
    YawPitchFlyTimeIndex next_solution;
    double yaw_vel, pitch_vel;
    auto next_solution_opt = trajectory_solver_.solveTarget(
        target_state_d, bullet_speed_mps, config_.rk45_traj,
        config_.iterative_traj);
    next_solution = next_solution_opt.has_value() ? next_solution_opt.value()
                                                  : current_solution;
    if (!next_solution_opt.has_value()) {
      reference.fallback_sample_count++;
      LOG_TRACE_L1(logger_,
                   "[Planner]: use trajectory hold fallback at frame {} when "
                   "aim fails.",
                   frame_index);
    }
    yaw_vel = tools::limitRadian(next_solution.yaw - previous_solution.yaw) /
              (2 * config_.dt_sec);
    pitch_vel =
        (next_solution.pitch - previous_solution.pitch) / (2 * config_.dt_sec);
    // 将当前帧保存到轨迹中
    reference.state_reference.col(frame_index)
        << tools::limitRadian(current_solution.yaw - reference.center_yaw),
        yaw_vel, current_solution.pitch, pitch_vel;
    previous_solution = current_solution;
    current_solution = next_solution;
  }
  if (reference.fallback_sample_count > 0) {
    LOG_TRACE_L1(logger_, "[Planner]: fallback solve used {}/{} samples.",
                 reference.fallback_sample_count, trajectory_horizon_ + 2);
  }
  // HACK: 锁的颗粒度太粗了
  return reference;
}
