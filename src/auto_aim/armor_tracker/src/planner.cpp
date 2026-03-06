#include "planner.hpp"
#include "configs.hpp"
#include "math/angle_tools.hpp"
#include "msgs/AimCommand.hpp"
#include "trajectory.hpp"

#include "quill/LogMacros.h"
#include "types.hpp"

#include <chrono>
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
    // 注意Eigen对于指针构造默认深拷贝，不用担心生命周期
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
    double bullet_speed_mps) {
  const double dt_image_to_now_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::system_clock::now() - target_stamp)
          .count();
  return plan(target_state, dt_image_to_now_sec, bullet_speed_mps);
}

msgs::AimCommand auto_aim::Planner::plan(const TargetState &target_state,
                                         double dt_image_to_now_sec,
                                         double bullet_speed_mps) {
  if (bullet_speed_mps < config_.min_bullet_speed ||
      bullet_speed_mps > config_.max_bullet_speed) {
    LOG_WARNING(logger_, "[Planner]: Abnormal bullet speed {}, use default {}.",
                bullet_speed_mps, config_.default_bullet_speed);
    bullet_speed_mps = config_.default_bullet_speed;
  }
  Eigen::MatrixXd trajectory_reference;
  double center_reference_yaw_rad = 0.0;
  try {
    std::tie(trajectory_reference, center_reference_yaw_rad) =
        buildTrajectoryReference(target_state, dt_image_to_now_sec,
                                 bullet_speed_mps);
  } catch (const std::exception &e) {
    LOG_WARNING(logger_, "{}, bullet_speed: {}.", e.what(), bullet_speed_mps);
    return {.control = false};
  }
  Eigen::VectorXd initial_state(2);
  initial_state << trajectory_reference(0, 0), trajectory_reference(1, 0);
  tiny_set_x0(yaw_solver_, initial_state);
  yaw_solver_->work->Xref =
      trajectory_reference.block(0, 0, 2, trajectory_horizon_);
  tiny_solve(yaw_solver_);
  initial_state << trajectory_reference(2, 0), trajectory_reference(3, 0);
  tiny_set_x0(pitch_solver_, initial_state);
  pitch_solver_->work->Xref =
      trajectory_reference.block(2, 0, 2, trajectory_horizon_);
  tiny_solve(pitch_solver_);

  msgs::AimCommand cmd;
  cmd.control = true;
  cmd.target_yaw =
      tools::limitRadian(trajectory_reference(0, config_.trajectory_half_horizon) +
                         center_reference_yaw_rad);
  cmd.target_pitch =
      trajectory_reference(2, config_.trajectory_half_horizon);
  cmd.yaw = tools::limitRadian(
      yaw_solver_->work->x(0, config_.trajectory_half_horizon) +
      center_reference_yaw_rad);
  cmd.yaw_vel = yaw_solver_->work->x(1, config_.trajectory_half_horizon);
  cmd.yaw_acc = yaw_solver_->work->u(0, config_.trajectory_half_horizon);
  cmd.pitch = pitch_solver_->work->x(0, config_.trajectory_half_horizon);
  cmd.pitch_vel = pitch_solver_->work->x(1, config_.trajectory_half_horizon);
  cmd.pitch_acc = pitch_solver_->work->u(0, config_.trajectory_half_horizon);
  cmd.fire =
      std::hypot(
          trajectory_reference(
              0, config_.trajectory_half_horizon + config_.shoot_offset) -
              yaw_solver_->work->x(0, config_.trajectory_half_horizon +
                                          config_.shoot_offset),
          trajectory_reference(
              2, config_.trajectory_half_horizon + config_.shoot_offset) -
              pitch_solver_->work->x(0, config_.trajectory_half_horizon +
                                            config_.shoot_offset)) <
      config_.fire_thresh;
  cmd.bullet_id = bullet_id_++; // 发送自增的子弹ID，用于弹道闭环
  return cmd;
}

std::pair<Eigen::MatrixXd, double>
auto_aim::Planner::buildTrajectoryReference(const TargetState &target_state,
                                            double dt_image_to_now_sec,
                                            double bullet_speed_mps) {
  auto solveAimWithMethodFallback = [&](const TargetState &predicted_target_state,
                                        bool use_rk45,
                                        bool iterative_fly_time) {
    auto solved_yaw_pitch = trajectory_solver_.resolveTarget(
        predicted_target_state, bullet_speed_mps, dt_image_to_now_sec, use_rk45,
        iterative_fly_time);
    if (!solved_yaw_pitch.has_value() && use_rk45) {
      solved_yaw_pitch = trajectory_solver_.resolveTarget(
          predicted_target_state, bullet_speed_mps, dt_image_to_now_sec, false,
          iterative_fly_time);
    }
    return solved_yaw_pitch;
  };

  auto requireAimSolutionOrThrow = [&](const TargetState &predicted_target_state,
                                       bool use_rk45,
                                       bool iterative_fly_time) {
    auto solved_yaw_pitch =
        solveAimWithMethodFallback(predicted_target_state, use_rk45,
                                   iterative_fly_time);
    if (!solved_yaw_pitch.has_value())
      throw std::runtime_error("Unsolvable bullet trajectory!");
    return solved_yaw_pitch.value();
  };

  auto center_aim_solution = requireAimSolutionOrThrow(
      target_state, config_.rk45_yaw0, config_.iterative_yaw0);
  const double center_reference_yaw_rad = center_aim_solution.yaw;
  aim0_predict_time_.store(center_aim_solution.fly_time + dt_image_to_now_sec);
  Eigen::MatrixXd trajectory_reference(4, trajectory_horizon_);

  TargetState target_state_for_current_frame = target_state.predict(
      -config_.dt_sec * (config_.trajectory_half_horizon + 1));
  auto previous_solution_optional =
      solveAimWithMethodFallback(target_state_for_current_frame,
                                 config_.rk45_traj, config_.iterative_traj);
  if (!previous_solution_optional.has_value()) {
    LOG_TRACE_L1(logger_,
                 "[Planner]: use center-aim fallback for first trajectory "
                 "sample.");
  }
  auto previous_solution =
      previous_solution_optional.value_or(center_aim_solution);
  target_state_for_current_frame =
      target_state_for_current_frame.predict(config_.dt_sec);
  auto current_solution_optional =
      solveAimWithMethodFallback(target_state_for_current_frame,
                                 config_.rk45_traj, config_.iterative_traj);
  if (!current_solution_optional.has_value()) {
    LOG_TRACE_L1(
        logger_,
        "[Planner]: use previous-sample fallback for second trajectory sample.");
  }
  auto current_solution =
      current_solution_optional.value_or(previous_solution);

  for (int frame_index = 0; frame_index < trajectory_horizon_; ++frame_index) {
    target_state_for_current_frame =
        target_state_for_current_frame.predict(config_.dt_sec);
    auto next_solution = solveAimWithMethodFallback(
        target_state_for_current_frame, config_.rk45_traj,
        config_.iterative_traj);
    if (!next_solution.has_value()) {
      next_solution = current_solution;
      LOG_TRACE_L1(
          logger_,
          "[Planner]: use trajectory hold fallback at frame {} when aim fails.",
          frame_index);
    }

    const double yaw_velocity =
        tools::limitRadian(next_solution->yaw - previous_solution.yaw) /
        (2 * config_.dt_sec);
    const double pitch_velocity =
        (next_solution->pitch - previous_solution.pitch) /
        (2 * config_.dt_sec);
    trajectory_reference.col(frame_index)
        << tools::limitRadian(current_solution.yaw - center_reference_yaw_rad),
        yaw_velocity,
        current_solution.pitch, pitch_velocity;
    previous_solution = current_solution;
    current_solution = next_solution.value();
  }

  return {trajectory_reference, center_reference_yaw_rad};
}
