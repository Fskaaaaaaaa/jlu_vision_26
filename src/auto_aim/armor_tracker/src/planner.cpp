#include "planner.hpp"
#include "configs.hpp"
#include "math/angle_tools.hpp"
#include "msgs/AimCommand.hpp"
#include "trajectory.hpp"

#include "quill/LogMacros.h"
#include "types.hpp"

#include <chrono>
#include <optional>
#include <utility>

auto_aim::Planner::Planner(quill::Logger *logger, const PlannerConfig &config)
    : logger_(logger), config_(config),
      traj_solver_(logger_, config_.trajectory_conf), aim0_predict_time_(0) {
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
    const TargetStatus &target_state,
    const std::chrono::system_clock::time_point &target_stamp,
    double bullet_speed) {
  if (target_state.track_status == TrackStatus::Lost)
    return {.control = false};
  double dt_sec = std::chrono::duration_cast<std::chrono::duration<double>>(
                      std::chrono::system_clock::now() - target_stamp)
                      .count();
  return plan(target_state, dt_sec, bullet_speed);
}

msgs::AimCommand auto_aim::Planner::plan(const TargetStatus &target_state,
                                         double dt, double bullet_speed) {
  // 0. Check bullet speed
  if (bullet_speed < config_.min_bullet_speed ||
      bullet_speed > config_.max_bullet_speed) {
    LOG_WARNING(logger_, "abnormal bullet speed {}, use default {}.",
                bullet_speed, config_.default_bullet_speed);
    bullet_speed = config_.default_bullet_speed;
  }
  // 1. Get trajectory
  Eigen::MatrixXd traj;
  double yaw0;
  try {
    std::tie(traj, yaw0) = getTrajectoryYaw0(target_state, dt, bullet_speed);
  } catch (const std::exception &e) {
    LOG_WARNING(logger_, "{}, bullet_speed: {}.", e.what(), bullet_speed);
    return {false};
  }
  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);
  tiny_set_x0(yaw_solver_, x0);
  yaw_solver_->work->Xref = traj.block(0, 0, 2, trajectory_horizon_);
  tiny_solve(yaw_solver_);
  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);
  pitch_solver_->work->Xref = traj.block(2, 0, 2, trajectory_horizon_);
  tiny_solve(pitch_solver_);

  msgs::AimCommand cmd;
  cmd.control = true;
  cmd.target_yaw =
      tools::limitRadian(traj(0, config_.trajectory_half_horizon) + yaw0);
  cmd.target_pitch = traj(2, config_.trajectory_half_horizon);
  cmd.yaw = tools::limitRadian(
      yaw_solver_->work->x(0, config_.trajectory_half_horizon) + yaw0);
  cmd.yaw_vel = yaw_solver_->work->x(1, config_.trajectory_half_horizon);
  cmd.yaw_acc = yaw_solver_->work->u(0, config_.trajectory_half_horizon);
  cmd.pitch = pitch_solver_->work->x(0, config_.trajectory_half_horizon);
  cmd.pitch_vel = pitch_solver_->work->x(1, config_.trajectory_half_horizon);
  cmd.pitch_acc = pitch_solver_->work->u(0, config_.trajectory_half_horizon);
  cmd.fire =
      std::hypot(
          traj(0, config_.trajectory_half_horizon + config_.shoot_offset) -
              yaw_solver_->work->x(0, config_.trajectory_half_horizon +
                                          config_.shoot_offset),
          traj(2, config_.trajectory_half_horizon + config_.shoot_offset) -
              pitch_solver_->work->x(0, config_.trajectory_half_horizon +
                                            config_.shoot_offset)) <
      config_.fire_thresh;
  cmd.bullet_id = bullet_id_++; // 发送自增的子弹ID，用于弹道闭环
  return cmd;
}

std::pair<Eigen::MatrixXd, double>
auto_aim::Planner::getTrajectoryYaw0(const TargetStatus &target_state,
                                     double dt_image_to_now,
                                     double bullet_speed) {
  auto aim = [&](const TargetStatus &state, bool high_precision = false) {
    auto yaw_pitch_flytime_opt = traj_solver_.resolveTarget(
        state, bullet_speed, dt_image_to_now,
        high_precision ? false : config_.use_analytical_solution,
        high_precision ? true : config_.iterative_fly_time);
    if (!yaw_pitch_flytime_opt.has_value())
      throw std::runtime_error("Unsolvable bullet trajectory!");

    return yaw_pitch_flytime_opt.value();
  };
  auto aim0 = aim(target_state, true);
  double yaw0 = aim0.yaw; // mid
  aim0_predict_time_.store(aim0.fly_time + dt_image_to_now);
  Eigen::MatrixXd traj(4, trajectory_horizon_);
  TargetStatus status = target_state.predict(
      -config_.dt_sec * (config_.trajectory_half_horizon + 1));
  auto yaw_pitch_last = aim(status);
  status = status.predict(config_.dt_sec);
  auto yaw_pitch = aim(status);                   // left
  for (int i = 0; i < trajectory_horizon_; i++) { // until right
    status = status.predict(config_.dt_sec);
    auto yaw_pitch_next = aim(status);
    auto yaw_vel = tools::limitRadian(yaw_pitch_next.yaw - yaw_pitch_last.yaw) /
                   (2 * config_.dt_sec);
    auto pitch_vel =
        (yaw_pitch_next.pitch - yaw_pitch_last.pitch) / (2 * config_.dt_sec);
    // 这里将yaw映射到以yaw0为中心的相对角度
    traj.col(i) << tools::limitRadian(yaw_pitch.yaw - yaw0), yaw_vel,
        yaw_pitch.pitch, pitch_vel;
    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }
  return {traj, yaw0};
}
