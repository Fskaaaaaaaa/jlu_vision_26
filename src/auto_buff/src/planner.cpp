#include "planner.hpp"

#include "math/angle_tools.hpp"
#include "parameter.hpp"

#include "quill/LogMacros.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <unordered_map>

namespace {

tools::ballistic::BallisticConfig makeBallisticConfig() {
    return tools::ballistic::BallisticConfig{
        .g = par::TRACKER_BALLISTIC_G,
        .k = par::TRACKER_BALLISTIC_K,
        .barrel_length = par::TRACKER_BALLISTIC_BARREL_LENGTH,
        .time_step = par::TRACKER_BALLISTIC_TIME_STEP,
        .max_fly_time = par::TRACKER_BALLISTIC_MAX_FLY_TIME,
        .max_pitch_iterate_count = par::TRACKER_BALLISTIC_MAX_PITCH_ITERATE_COUNT,
        .min_pitch_error_m = par::TRACKER_BALLISTIC_MIN_PITCH_ERROR_M,
        .gimbal_pitch_min_degree = par::TRACKER_BALLISTIC_PITCH_MIN_DEG,
        .gimbal_pitch_max_degree = par::TRACKER_BALLISTIC_PITCH_MAX_DEG,
    };
}

double targetScore(const BuffAimTrack& track) {
    double score = track.current.position_gimbal.norm();
    if (track.predicted.has_value()) {
        score -= 0.5;
    }
    return score;
}

double resolveReferenceDt(const BuffAimTrack& track,
                          const BuffAimSolution& current_solution) {
    if (track.predicted.has_value()) {
        if (track.predicted->fly_time > par::TRACKER_PLANNER_DT_SEC) {
            return track.predicted->fly_time;
        }
    }
    if (current_solution.fly_time > par::TRACKER_PLANNER_DT_SEC) {
        return current_solution.fly_time;
    }
    return par::TRACKER_PLANNER_DT_SEC;
}

} // namespace

BuffPlanner::BuffPlanner(quill::Logger* logger)
    : logger_(logger),
      solver_(logger_, makeBallisticConfig()),
      trajectory_horizon_(par::TRACKER_TRAJECTORY_HALF_HORIZON * 2) {
    Eigen::MatrixXd A{{1, par::TRACKER_PLANNER_DT_SEC}, {0, 1}};
    Eigen::MatrixXd B{{0}, {par::TRACKER_PLANNER_DT_SEC}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q_yaw(par::TRACKER_Q_YAW.data());
    Eigen::Matrix<double, 1, 1> R_yaw;
    R_yaw << par::TRACKER_R_YAW;
    tiny_setup(&yaw_solver_, A, B, f, Q_yaw.asDiagonal(), R_yaw.asDiagonal(), 1.0,
               2, 1, trajectory_horizon_, 0);

    Eigen::MatrixXd x_min =
        Eigen::MatrixXd::Constant(2, trajectory_horizon_, -1e17);
    Eigen::MatrixXd x_max =
        Eigen::MatrixXd::Constant(2, trajectory_horizon_, 1e17);
    Eigen::MatrixXd u_min =
        Eigen::MatrixXd::Constant(1, trajectory_horizon_ - 1,
                                  -par::TRACKER_MAX_YAW_ACC);
    Eigen::MatrixXd u_max =
        Eigen::MatrixXd::Constant(1, trajectory_horizon_ - 1,
                                  par::TRACKER_MAX_YAW_ACC);
    tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);
    yaw_solver_->settings->max_iter = 10;

    Eigen::Matrix<double, 2, 1> Q_pitch(par::TRACKER_Q_PITCH.data());
    Eigen::Matrix<double, 1, 1> R_pitch;
    R_pitch << par::TRACKER_R_PITCH;
    tiny_setup(&pitch_solver_, A, B, f, Q_pitch.asDiagonal(), R_pitch.asDiagonal(),
               1.0, 2, 1, trajectory_horizon_, 0);
    u_min =
        Eigen::MatrixXd::Constant(1, trajectory_horizon_ - 1,
                                  -par::TRACKER_MAX_PITCH_ACC);
    u_max =
        Eigen::MatrixXd::Constant(1, trajectory_horizon_ - 1,
                                  par::TRACKER_MAX_PITCH_ACC);
    tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);
    pitch_solver_->settings->max_iter = 10;
}

msgs::AimCommand BuffPlanner::plan(const BuffTargetSnapshot& snapshot,
                                   const msgs::GimbalInfo& gimbal_info) {
    const auto track_opt = selectTarget(snapshot);
    if (!track_opt.has_value()) {
        return {.control = false};
    }

    const double bullet_speed = resolveBulletSpeed(gimbal_info.bullet_speed);
    BuffTrajectoryReference reference;
    try {
        reference =
            buildTrajectoryReference(track_opt.value(), gimbal_info, bullet_speed);
    } catch (const std::exception& e) {
        LOG_WARNING(logger_, "buff planner failed to build reference: {}", e.what());
        return {.control = false};
    }

    Eigen::VectorXd initial_state(2);
    initial_state << reference.state_reference(0, 0), reference.state_reference(1, 0);
    tiny_set_x0(yaw_solver_, initial_state);
    yaw_solver_->work->Xref = reference.state_reference.block(0, 0, 2, trajectory_horizon_);
    tiny_solve(yaw_solver_);

    initial_state << reference.state_reference(2, 0), reference.state_reference(3, 0);
    tiny_set_x0(pitch_solver_, initial_state);
    pitch_solver_->work->Xref =
        reference.state_reference.block(2, 0, 2, trajectory_horizon_);
    tiny_solve(pitch_solver_);

    const int output_index = par::TRACKER_TRAJECTORY_HALF_HORIZON;
    const int fire_index =
        std::clamp(output_index + par::TRACKER_SHOOT_OFFSET, 0,
                   trajectory_horizon_ - 1);
    const int acc_index = std::min(output_index, trajectory_horizon_ - 2);
    const double fire_error = std::hypot(
        tools::shortestRadianDistance(
            yaw_solver_->work->x(0, fire_index), reference.state_reference(0, fire_index)),
        pitch_solver_->work->x(0, fire_index) - reference.state_reference(2, fire_index));

    msgs::AimCommand cmd{};
    cmd.control = true;
    cmd.fire = fire_error < par::TRACKER_FIRE_THRESH_RAD;
    cmd.target_yaw =
        static_cast<float>(tools::limitRadian(reference.state_reference(0, output_index)));
    cmd.target_pitch = static_cast<float>(reference.state_reference(2, output_index));
    cmd.yaw = static_cast<float>(tools::limitRadian(yaw_solver_->work->x(0, output_index)));
    cmd.yaw_vel = static_cast<float>(yaw_solver_->work->x(1, output_index));
    cmd.yaw_acc = static_cast<float>(yaw_solver_->work->u(0, acc_index));
    cmd.pitch = static_cast<float>(pitch_solver_->work->x(0, output_index));
    cmd.pitch_vel = static_cast<float>(pitch_solver_->work->x(1, output_index));
    cmd.pitch_acc = static_cast<float>(pitch_solver_->work->u(0, acc_index));
    cmd.bullet_id = bullet_id_++;
    return cmd;
}

std::optional<BuffAimTrack> BuffPlanner::selectTarget(
    const BuffTargetSnapshot& snapshot) {
    if (snapshot.targets.empty()) {
        return std::nullopt;
    }

    struct TrackGroup {
        std::optional<BuffTrackedTarget> observed;
        std::optional<BuffTrackedTarget> predicted;
    };
    std::unordered_map<int, TrackGroup> groups;
    for (const auto& target : snapshot.targets) {
        if (target.track_index < 0) {
            continue;
        }
        auto& group = groups[target.track_index];
        auto& slot = target.predicted ? group.predicted : group.observed;
        if (!slot.has_value() ||
            target.position_gimbal.norm() < slot->position_gimbal.norm()) {
            slot = target;
        }
    }

    std::vector<BuffAimTrack> candidates;
    candidates.reserve(groups.size());
    for (const auto& [track_index, group] : groups) {
        const auto current = group.observed.has_value() ? group.observed : group.predicted;
        if (!current.has_value()) {
            continue;
        }
        candidates.push_back(BuffAimTrack{
            .current = current.value(),
            .predicted = group.predicted,
        });
    }
    if (candidates.empty()) {
        return std::nullopt;
    }

    if (par::TRACKER_TARGET_SELECTION_POLICY ==
        par::BuffTargetSelectionPolicy::PreferTrack0FallbackTrack1) {
        const auto track0_it =
            std::find_if(candidates.begin(), candidates.end(),
                         [](const BuffAimTrack& candidate) {
                             return candidate.current.track_index == 0;
                         });
        if (track0_it != candidates.end()) {
            preferred_track_index_ = 0;
            return *track0_it;
        }
        const auto track1_it =
            std::find_if(candidates.begin(), candidates.end(),
                         [](const BuffAimTrack& candidate) {
                             return candidate.current.track_index == 1;
                         });
        if (track1_it != candidates.end()) {
            preferred_track_index_ = 1;
            return *track1_it;
        }
        return std::nullopt;
    }

    const auto best_it =
        std::min_element(candidates.begin(), candidates.end(),
                         [](const BuffAimTrack& lhs, const BuffAimTrack& rhs) {
                               return targetScore(lhs) < targetScore(rhs);
                           });
    auto selected = *best_it;

    if (preferred_track_index_.has_value()) {
        const auto preferred_it =
            std::find_if(candidates.begin(), candidates.end(),
                         [&](const BuffAimTrack& candidate) {
                             return candidate.current.track_index ==
                                    preferred_track_index_.value();
                          });
        if (preferred_it != candidates.end() &&
            targetScore(*preferred_it) <=
                targetScore(selected) + par::TRACKER_TARGET_SWITCH_HYSTERESIS_M) {
            selected = *preferred_it;
        }
    }
    preferred_track_index_ = selected.current.track_index;
    return selected;
}

std::optional<BuffAimSolution>
BuffPlanner::solveAim(const Eigen::Vector3d& position_gimbal,
                      double bullet_speed_mps, bool use_rk45) const {
    const double target_distance_m =
        std::hypot(position_gimbal.x(), position_gimbal.y());
    if (target_distance_m <= std::numeric_limits<double>::epsilon()) {
        return std::nullopt;
    }

    auto pitch_and_fly_time = solver_.resolvePitchFlyTime(
        target_distance_m, position_gimbal.z(), bullet_speed_mps,
        use_rk45 ? tools::ballistic::Method::rk45
                 : tools::ballistic::Method::parabola);
    if (!pitch_and_fly_time.has_value() && use_rk45) {
        pitch_and_fly_time = solver_.resolvePitchFlyTime(
            target_distance_m, position_gimbal.z(), bullet_speed_mps,
            tools::ballistic::Method::parabola);
    }
    if (!pitch_and_fly_time.has_value()) {
        return std::nullopt;
    }

    return BuffAimSolution{
        .yaw = std::atan2(position_gimbal.y(), position_gimbal.x()),
        .pitch = pitch_and_fly_time->pitch,
        .fly_time = pitch_and_fly_time->fly_time,
    };
}

BuffTrajectoryReference BuffPlanner::buildTrajectoryReference(
    const BuffAimTrack& track, const msgs::GimbalInfo& gimbal_info,
    double bullet_speed_mps) const {
    const auto current_solution =
        solveAim(track.current.position_gimbal, bullet_speed_mps,
                 par::TRACKER_BALLISTIC_USE_RK45);
    if (!current_solution.has_value()) {
        throw std::runtime_error("current target ballistic solve failed");
    }

    const double current_abs_yaw =
        tools::limitRadian(gimbal_info.yaw + current_solution->yaw);
    const double current_abs_pitch = gimbal_info.pitch + current_solution->pitch;

    double predicted_abs_yaw = current_abs_yaw;
    double predicted_abs_pitch = current_abs_pitch;
    BuffTrajectoryReference reference;

    if (track.predicted.has_value()) {
        const auto predicted_solution =
            solveAim(track.predicted->position_gimbal, bullet_speed_mps,
                     par::TRACKER_BALLISTIC_USE_RK45);
        if (predicted_solution.has_value()) {
            predicted_abs_yaw =
                tools::limitRadian(gimbal_info.yaw + predicted_solution->yaw);
            predicted_abs_pitch = gimbal_info.pitch + predicted_solution->pitch;
        } else {
            ++reference.fallback_sample_count;
        }
    }

    const double reference_dt = resolveReferenceDt(track, current_solution.value());
    const double yaw_velocity =
        tools::shortestRadianDistance(current_abs_yaw, predicted_abs_yaw) /
        reference_dt;
    const double pitch_velocity =
        (predicted_abs_pitch - current_abs_pitch) / reference_dt;

    reference.state_reference = Eigen::MatrixXd(4, trajectory_horizon_);
    for (int frame_index = 0; frame_index < trajectory_horizon_; ++frame_index) {
        const double relative_time =
            (frame_index - par::TRACKER_TRAJECTORY_HALF_HORIZON) *
            par::TRACKER_PLANNER_DT_SEC;
        reference.state_reference.col(frame_index)
            << tools::limitRadian(current_abs_yaw + yaw_velocity * relative_time),
            yaw_velocity, current_abs_pitch + pitch_velocity * relative_time,
            pitch_velocity;
    }
    return reference;
}

double BuffPlanner::wrapAngle(double angle_rad) {
    while (angle_rad > std::numbers::pi) {
        angle_rad -= 2.0 * std::numbers::pi;
    }
    while (angle_rad < -std::numbers::pi) {
        angle_rad += 2.0 * std::numbers::pi;
    }
    return angle_rad;
}

double BuffPlanner::resolveBulletSpeed(double measured_speed) {
    if (!std::isfinite(measured_speed) ||
        measured_speed < par::TRACKER_MIN_BULLET_SPEED ||
        measured_speed > par::TRACKER_MAX_BULLET_SPEED) {
        return par::TRACKER_DEFAULT_BULLET_SPEED;
    }
    return measured_speed;
}
