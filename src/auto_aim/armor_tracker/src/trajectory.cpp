#include "trajectory.hpp"
#include "configs.hpp"
#include "math/ballistic_models.hpp"
#include "math/ballistic_trajectory.hpp"
#include "types.hpp"

#include "quill/LogMacros.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

// XXX: 狗操的codex在我的代码里四处拉屎
namespace {

struct ArmorAimCandidate {
  auto_aim::ArmorPositionYaw armor;
  auto_aim::ArmorIndex index;
  double distance_to_gimbal_m;
  bool is_front_facing;
};

struct SolvedArmorAim {
  ArmorAimCandidate candidate;
  auto_aim::YawPitchFlyTime yaw_pitch_fly_time;
};

bool isArmorFrontFacing(const auto_aim::TargetState &target_state,
                        const auto_aim::ArmorPositionYaw &armor,
                        double odom_x_m, double odom_y_m,
                        double min_dot_product) {
  const Eigen::Vector2d center_to_armor_vector_m{
      armor.position.x() - target_state.center_position.x(),
      armor.position.y() - target_state.center_position.y(),
  };
  const Eigen::Vector2d armor_to_shooter_vector_m{
      odom_x_m - armor.position.x(),
      odom_y_m - armor.position.y(),
  };
  if (center_to_armor_vector_m.norm() <=
          std::numeric_limits<double>::epsilon() ||
      armor_to_shooter_vector_m.norm() <=
          std::numeric_limits<double>::epsilon()) {
    return false;
  }
  auto front_facing_score = center_to_armor_vector_m.normalized().dot(
      armor_to_shooter_vector_m.normalized());
  return front_facing_score > min_dot_product;
}

std::vector<ArmorAimCandidate>
collectArmorAimCandidates(const auto_aim::TargetState &target_state,
                          double odom_x_m, double odom_y_m,
                          double front_facing_min_dot_product) {
  const auto armors = target_state.armors();
  std::vector<ArmorAimCandidate> candidates;
  candidates.reserve(armors.size());
  for (std::size_t index = 0; index < armors.size(); ++index) {
    const auto &armor = armors.at(index);
    candidates.emplace_back(ArmorAimCandidate{
        .armor = armor,
        .index = static_cast<auto_aim::ArmorIndex>(index),
        .distance_to_gimbal_m = std::hypot(armor.position.x() - odom_x_m,
                                           armor.position.y() - odom_y_m),
        .is_front_facing =
            isArmorFrontFacing(target_state, armor, odom_x_m, odom_y_m,
                               front_facing_min_dot_product),
    });
  }
  return candidates;
}

void sortCandidatesByDistance(std::vector<ArmorAimCandidate> &candidates) {
  std::sort(candidates.begin(), candidates.end(),
            [](const ArmorAimCandidate &left, const ArmorAimCandidate &right) {
              return left.distance_to_gimbal_m < right.distance_to_gimbal_m;
            });
}

void prioritizePreferredCandidate(
    std::vector<ArmorAimCandidate> &candidates,
    std::optional<auto_aim::ArmorIndex> preferred_armor_index,
    double switch_distance_hysteresis_m) {
  if (candidates.empty() || !preferred_armor_index.has_value()) {
    return;
  }
  auto preferred_iterator =
      std::find_if(candidates.begin(), candidates.end(),
                   [&](const ArmorAimCandidate &candidate) {
                     return candidate.index == preferred_armor_index.value();
                   });
  if (preferred_iterator == candidates.end()) {
    return;
  }
  if (preferred_iterator->distance_to_gimbal_m >
      candidates.front().distance_to_gimbal_m + switch_distance_hysteresis_m) {
    return;
  }
  std::rotate(candidates.begin(), preferred_iterator, preferred_iterator + 1);
}

} // namespace

auto_aim::Trajectory::Trajectory(quill::Logger *logger,
                                 const TrajectoryConfig &config)
    : logger_(logger), config_(config),
      solver_(logger_, config_.ballistic_conf) {}

std::optional<auto_aim::YawPitchFlyTime>
auto_aim::Trajectory::solveYawPitchForArmorPosition(
    double bullet_speed_mps, const Eigen::Vector3d &armor_position_m,
    bool use_rk45, double odom_x_m, double odom_y_m) {
  auto target_x_m = armor_position_m.x() - odom_x_m;
  auto target_y_m = armor_position_m.y() - odom_y_m;
  auto target_distance_m = std::hypot(target_x_m, target_y_m);
  auto target_height_m = armor_position_m.z();
  auto target_yaw_rad = std::atan2(target_y_m, target_x_m);
  auto pitch_and_fly_time = solver_.resolvePitchFlyTime(
      target_distance_m, target_height_m, bullet_speed_mps,
      use_rk45 ? tools::ballistic::Method::rk45
               : tools::ballistic::Method::parabola);
  if (!pitch_and_fly_time.has_value()) {
    return std::nullopt;
  }
  return YawPitchFlyTime{
      .yaw = target_yaw_rad,
      .pitch = pitch_and_fly_time->pitch,
      .fly_time = pitch_and_fly_time->fly_time,
  };
}

std::pair<auto_aim::ArmorPositionYaw, auto_aim::ArmorIndex>
auto_aim::Trajectory::selectArmorForAiming(
    const TargetState &state, std::optional<ArmorIndex> preferred_armor_index,
    double odom_x_m, double odom_y_m) const {
  auto candidates = collectArmorAimCandidates(
      state, odom_x_m, odom_y_m, config_.armor_front_facing_min_dot_product);
  if (candidates.empty()) {
    return {ArmorPositionYaw{}, ArmorIndex::_0};
  }

  auto pickBestCandidate = [&](std::vector<ArmorAimCandidate> candidate_group)
      -> std::optional<ArmorAimCandidate> {
    if (candidate_group.empty()) {
      return std::nullopt;
    }
    sortCandidatesByDistance(candidate_group);
    prioritizePreferredCandidate(candidate_group, preferred_armor_index,
                                 config_.armor_switch_distance_hysteresis_m);
    return candidate_group.front();
  };

  std::vector<ArmorAimCandidate> front_facing_candidates;
  front_facing_candidates.reserve(candidates.size());
  std::copy_if(candidates.begin(), candidates.end(),
               std::back_inserter(front_facing_candidates),
               [](const ArmorAimCandidate &candidate) {
                 return candidate.is_front_facing;
               });
  if (auto front_facing_best =
          pickBestCandidate(std::move(front_facing_candidates));
      front_facing_best.has_value()) {
    return {front_facing_best->armor, front_facing_best->index};
  }

  auto nearest_candidate = pickBestCandidate(std::move(candidates));
  return {nearest_candidate->armor, nearest_candidate->index};
}

double auto_aim::Trajectory::evaluateImpactPositionError(
    double yaw_rad, double pitch_rad, double bullet_speed_mps,
    double fly_time_sec, const Eigen::Vector3d &armor_position_m,
    bool use_rk45) {
  auto bullet_position =
      use_rk45
          ? tools::ballistic::rk45::getState3DByT(
                solver_.getBarrelStateFromPitch(pitch_rad, bullet_speed_mps),
                yaw_rad, fly_time_sec, config_.ballistic_conf.time_step,
                config_.ballistic_conf.k, config_.ballistic_conf.g)
          : tools::ballistic::parabola::getState3DByT(
                {0, 0, pitch_rad, bullet_speed_mps}, yaw_rad, fly_time_sec,
                config_.ballistic_conf.g);
  return (bullet_position.position - armor_position_m).norm();
}

std::optional<auto_aim::TargetAimSolution> auto_aim::Trajectory::resolveTarget(
    const TargetState &state, double bullet_speed_mps,
    double delay_time_image_to_now_sec, bool use_rk45, bool iterative_fly_time,
    std::optional<ArmorIndex> preferred_armor_index) {
  // XXX:  狗屎代码
  auto solve_first_solvable_armor_aim =
      [&](const TargetState &predicted_state,
          std::optional<ArmorIndex> preferred_index)
      -> std::optional<SolvedArmorAim> {
    auto candidates = collectArmorAimCandidates(
        predicted_state, 0.0, 0.0, config_.armor_front_facing_min_dot_product);
    auto try_solve_candidates =
        [&](std::vector<ArmorAimCandidate> candidate_group)
        -> std::optional<SolvedArmorAim> {
      if (candidate_group.empty()) {
        return std::nullopt;
      }
      sortCandidatesByDistance(candidate_group);
      prioritizePreferredCandidate(candidate_group, preferred_index,
                                   config_.armor_switch_distance_hysteresis_m);
      for (const auto &candidate : candidate_group) {
        auto solved = solveYawPitchForArmorPosition(
            bullet_speed_mps, candidate.armor.position, use_rk45, 0.0, 0.0);
        if (!solved.has_value())
          continue;
        return SolvedArmorAim{
            .candidate = candidate,
            .yaw_pitch_fly_time = solved.value(),
        };
      }
      return std::nullopt;
    };
    std::vector<ArmorAimCandidate> front_facing_candidates;
    front_facing_candidates.reserve(candidates.size());
    std::ranges::copy_if(candidates,
                         std::back_inserter(front_facing_candidates),
                         [](const ArmorAimCandidate &candidate) {
                           return candidate.is_front_facing;
                         });
    if (auto solved_front_facing =
            try_solve_candidates(std::move(front_facing_candidates));
        solved_front_facing.has_value()) {
      return solved_front_facing;
    }
    return try_solve_candidates(std::move(candidates));
  };

  auto predicted_target_state = state.predict(delay_time_image_to_now_sec);
  auto solved_aim = solve_first_solvable_armor_aim(predicted_target_state,
                                                   preferred_armor_index);
  if (!solved_aim.has_value())
    return std::nullopt;
  if (preferred_armor_index.has_value() &&
      preferred_armor_index.value() != solved_aim->candidate.index) {
    LOG_TRACE_L2(logger_, "[Trajectory]: aim armor switch {} -> {}.",
                 static_cast<int>(preferred_armor_index.value()),
                 static_cast<int>(solved_aim->candidate.index));
  }

  if (!iterative_fly_time) {
    return TargetAimSolution{
        .yaw_pitch_fly_time = solved_aim->yaw_pitch_fly_time,
        .selected_armor_index = solved_aim->candidate.index,
    };
  }

  double aiming_error_m = std::numeric_limits<double>::infinity();
  int iteration_index = 0;
  int switched_armor_count = 0;
  while (aiming_error_m >= config_.aim_ok_error_m) {
    if (++iteration_index > config_.max_aim_iterate_count) {
      LOG_WARNING(logger_,
                  "[Trajectory]: Fail to solve YawPitchFlyTime in {} "
                  "iterations, error {}.",
                  iteration_index, aiming_error_m);
      return std::nullopt;
    }

    const auto current_solution = solved_aim->yaw_pitch_fly_time;
    predicted_target_state =
        state.predict(delay_time_image_to_now_sec + current_solution.fly_time);
    auto next_solved_aim = solve_first_solvable_armor_aim(
        predicted_target_state, solved_aim->candidate.index);
    if (!next_solved_aim.has_value()) {
      LOG_WARNING(logger_,
                  "[Trajectory]: Fail to solve YawPitchFlyTime in {} "
                  "iterations, error {}.",
                  iteration_index, aiming_error_m);
      return std::nullopt;
    }
    if (next_solved_aim->candidate.index != solved_aim->candidate.index &&
        ++switched_armor_count >= config_.max_aim_switch_armor_count) {
      LOG_WARNING(logger_, "[Trajectory]: Armor select unstable!");
      return TargetAimSolution{
          .yaw_pitch_fly_time = current_solution,
          .selected_armor_index = solved_aim->candidate.index,
      };
    }

    aiming_error_m = evaluateImpactPositionError(
        current_solution.yaw, current_solution.pitch, bullet_speed_mps,
        current_solution.fly_time, next_solved_aim->candidate.armor.position,
        use_rk45);
    if (aiming_error_m < config_.aim_ok_error_m) {
      return TargetAimSolution{
          .yaw_pitch_fly_time = current_solution,
          .selected_armor_index = solved_aim->candidate.index,
      };
    }
    solved_aim = next_solved_aim;
  }
  return std::nullopt;
}
