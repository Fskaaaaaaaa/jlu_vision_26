// Copyright (c) 2026 shuodedaoli. All Rights Reserved.
// TODO 换马氏距离匹配armor，
// 添加自适应噪声
// 添加半径窗口平滑（不推荐，违背因子图初衷）
#include "target.hpp"
#include "configs.hpp"
#include "factors.hpp"
#include "math/angle_tools.hpp"
#include "math/sigmoid_functions.hpp"
#include "types.hpp"
#include "types/ArmorType.hpp"

#include "quill/LogMacros.h"
#include "rfl/enums.hpp"
#include <array>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iterator>
#include <mutex>
#include <utility>
#include <vector>

std::vector<auto_aim::ArmorMatchResult> auto_aim::Target::matchArmor(
    const std::vector<ArmorPositionYaw> &armors, const ArmorPositionYaw &obs,
    double max_match_distance, double max_match_yaw_diff) {
  std::vector<ArmorMatchResult> results;
  std::size_t i = 0;
  for (const auto &armor : armors) {
    auto distance = (armor.position - obs.position).norm();
    auto yaw_diff = std::abs(obs.yaw.localCoordinates(armor.yaw).x());
    if (distance <= max_match_distance && yaw_diff <= max_match_yaw_diff)
      results.emplace_back(static_cast<ArmorIndex>(i), distance, yaw_diff);
    ++i;
  }
  std::ranges::sort(results, std::ranges::less{}, &ArmorMatchResult::distance);
  return results;
}

auto_aim::RobotTarget::RobotTarget(quill::Logger *logger,
                                   const RobotConfig &config,
                                   types::ArmorType type)
    : logger_(logger), config_(config) {
  target_state_.type = type;
  target_state_.radius_a = config_.default_radius;
  target_state_.radius_b = config_.default_radius;
  target_state_.dz = config_.default_dz;
  track_state_.state = TrackState::State::LOST;
  track_state_.stamp_last_update = std::chrono::system_clock::from_time_t(0);
  track_state_.stamp_last_tracking = std::chrono::system_clock::from_time_t(0);
  track_state_.k = 0;
}

std::vector<auto_aim::ArmorPositionYaw>
auto_aim::RobotTarget::getArmorsFromTargetState(const TargetState &state,
                                                double radius_a,
                                                double radius_b, double dz) {
  std::vector<ArmorPositionYaw> armors;
  for (auto i : std::array{
           ArmorIndex::_0,
           ArmorIndex::_1,
           ArmorIndex::_2,
           ArmorIndex::_3,
       }) {
    auto [_r, _dz] = (i == ArmorIndex::_0 || i == ArmorIndex::_2)
                         ? std::make_pair(radius_a, 0.0)
                         : std::make_pair(radius_b, dz);
    armors.emplace_back(state.center_position, state.center_yaw, _r, _dz, 4, i);
  }
  return armors;
}

std::vector<auto_aim::ArmorPositionYaw>
auto_aim::RobotTarget::getArmorsFromTargetState(
    const RobotTargetState &state) const {
  return RobotTarget::getArmorsFromTargetState(state, state.radius_a,
                                               state.radius_b, state.dz);
}

std::vector<std::pair<auto_aim::ArmorPositionYaw, auto_aim::ArmorIndex>>
auto_aim::RobotTarget::matchArmorsUnique(
    const RobotTargetState &state,
    const std::vector<ArmorPositionYaw> &obs_armors) const {
  auto armors_for_association = RobotTarget::getArmorsFromTargetState(state);
  std::vector<MatchCandidate> candidates;
  for (std::size_t obs_i = 0; obs_i < obs_armors.size(); ++obs_i) {
    auto match_result = Target::matchArmor(
        armors_for_association, obs_armors[obs_i], config_.max_match_distance_m,
        tools::angle2Radian(config_.max_match_yaw_diff_degree));
    for (const auto &match : match_result)
      candidates.emplace_back(obs_i, match.index, match.distance,
                              match.yaw_diff);
  }
  std::ranges::sort(
      candidates, [](const MatchCandidate &a, const MatchCandidate &b) -> bool {
        if (a.distance != b.distance)
          return a.distance < b.distance;
        return a.yaw_diff < b.yaw_diff;
      });
  std::array<bool, 4> used_index{false, false, false, false};
  std::vector<bool> used_obs(obs_armors.size(), false);
  std::vector<std::pair<ArmorPositionYaw, ArmorIndex>> matched_armors;
  matched_armors.reserve(candidates.size());
  for (const auto &candidate : candidates) {
    auto idx = static_cast<std::size_t>(candidate.index);
    if (used_obs.at(candidate.obs_i) || used_index.at(idx))
      continue;
    used_obs.at(candidate.obs_i) = true;
    used_index.at(idx) = true;
    matched_armors.emplace_back(obs_armors.at(candidate.obs_i),
                                candidate.index);
  }
  return matched_armors;
}

auto_aim::RobotTargetState auto_aim::RobotTarget::getTargetStateFromArmor(
    const ArmorPositionYaw &armor) const {
  auto armor_x = armor.position.x();
  auto armor_y = armor.position.y();
  auto center_x =
      armor_x + config_.default_radius * std::cos(armor.yaw.theta());
  auto center_y =
      armor_y + config_.default_radius * std::sin(armor.yaw.theta());
  auto center_z = armor.position.z();
  RobotTargetState state;
  state.type = target_state_.type;
  state.center_position = Eigen::Vector3d{center_x, center_y, center_z};
  state.center_yaw = armor.yaw.theta();
  state.radius_a = config_.default_radius;
  state.radius_b = config_.default_radius;
  state.dz = config_.default_dz;
  return state;
}

std::pair<auto_aim::TargetState, auto_aim::TrackState>
auto_aim::RobotTarget::getTargetTrackState() const {
  std::scoped_lock lk{state_mtx_};
  return {
      target_state_.getStateWithArmorsFunc(
          [ra = target_state_.radius_a, rb = target_state_.radius_b,
           dz = target_state_.dz](const TargetState &state) {
            return getArmorsFromTargetState(state, ra, rb, dz);
          }),
      track_state_,
  };
}

auto_aim::TrackState::State auto_aim::RobotTarget::track(
    const std::vector<types::Armor> &armors,
    const std::chrono::system_clock::time_point &stamp) {
  std::vector<ArmorPositionYaw> selected_armors;
  std::copy_if(armors.begin(), armors.end(),
               std::back_inserter(selected_armors),
               [this](const types::Armor &armor) -> bool {
                 return armor.type == target_state_.type;
               });
  double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                  stamp - track_state_.stamp_last_update)
                  .count();
  auto [estimated_target_state, updated_track_state] =
      update(selected_armors, dt);
  if (updated_track_state == TrackState::State::TRACKING) {
    if (track_state_.state != TrackState::State::TRACKING) {
      LOG_INFO(logger_, "[Target {}]: {} -> TRACKING. dt{}, k{}.",
               rfl::enum_to_string(target_state_.type),
               rfl::enum_to_string(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_tracking = stamp;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::TEMPLOST) {
    if (track_state_.state != TrackState::State::TEMPLOST) {
      LOG_INFO(logger_, "[Target {}]: TRACKING -> TEMPLOST. dt{}, k{}.",
               rfl::enum_to_string(target_state_.type), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    target_state_ = estimated_target_state;
    track_state_.state = updated_track_state;
    track_state_.stamp_last_update = stamp;
    track_state_.k += 1;
  } else if (updated_track_state == TrackState::State::LOST) {
    if (track_state_.state != TrackState::State::LOST) {
      LOG_INFO(logger_, "[Target {}]: {} -> LOST. dt{}, k{}.",
               rfl::enum_to_string(target_state_.type),
               rfl::enum_to_string(track_state_.state), dt, track_state_.k);
    }
    std::scoped_lock lk{state_mtx_};
    track_state_.state = TrackState::State::LOST;
    track_state_.k = 0;
    isam2_ = gtsam::ISAM2{};
  }
  return track_state_.state;
}

double auto_aim::RobotTarget::get(const std::string &key) {
  std::scoped_lock lk{state_mtx_};
  if (key == "ra")
    return target_state_.radius_a;
  if (key == "rb")
    return target_state_.radius_b;
  if (key == "dz")
    return target_state_.dz;
  return 0;
}

using namespace gtsam::symbol_shorthand;

void auto_aim::RobotTarget::addMotionValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    const TargetState &target_state, std::uint64_t k, double dt) const {
  values.insert(X(k), target_state.center_position);
  values.insert(R(k), gtsam::Rot2::fromAngle(target_state.center_yaw));
  values.insert(V(k), target_state.center_velocity);
  values.insert(W(k), target_state.center_vyaw);
  if (k == 0) {
    graph.addPrior(X(0), target_state.center_position,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.translation_prior_noise.x,
                       config_.translation_prior_noise.y,
                       config_.translation_prior_noise.z,
                   }));
    graph.addPrior(
        R(0), gtsam::Rot2::fromAngle(target_state.center_yaw),
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_prior_noise));
    graph.addPrior(V(0), target_state.center_velocity,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.velocity_prior_noise.x,
                       config_.velocity_prior_noise.y,
                       config_.velocity_prior_noise.z,
                   }));
    graph.addPrior(
        W(0), target_state.center_vyaw,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_prior_noise));
  } else {
    graph.add(TranslationFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.translation_factor_noise.x,
            config_.translation_factor_noise.y,
            config_.translation_factor_noise.z,
        }),
        X(k - 1),
        V(k - 1),
        X(k),
        dt,
    });
    graph.add(YawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_factor_noise),
        R(k - 1),
        W(k - 1),
        R(k),
        dt,
    });
    graph.add(VelocityFactor{
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
            config_.velocity_factor_noise.x,
            config_.velocity_factor_noise.y,
            config_.velocity_factor_noise.z,
        }),
        V(k - 1),
        V(k),
    });
    graph.add(VyawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_factor_noise),
        W(k - 1),
        W(k),
    });
  }
  LOG_TRACE_L1(logger_,
               "[Target {}]: add motion values factors. k = {}, dt = {}.",
               rfl::enum_to_string(target_state_.type), k, dt);
}

void auto_aim::RobotTarget::addArmorValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    const std::vector<std::pair<ArmorPositionYaw, ArmorIndex>> &armor_indexs,
    std::uint64_t k) const {
  auto default_radius = tools::logisticInverse(
      config_.default_radius, config_.radius_min, config_.radius_max);
  if (k == 0) {
    values.insert(A(0), default_radius);
    values.insert(B(0), default_radius);
    values.insert(Z(0), config_.default_dz);
    graph.addPrior(
        A(0), default_radius,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.radius_prior_noise));
    graph.addPrior(
        B(0), default_radius,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.radius_prior_noise));
    graph.addPrior(
        Z(0), config_.default_dz,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.dz_prior_noise));
  }
  for (const auto &[armor, index] : armor_indexs) {
    if (index == ArmorIndex::_0 || index == ArmorIndex::_2) {
      graph.add(ArmorRACenterZFactor{
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
              config_.armor_observation_noise.tangential_error_m,
              config_.armor_observation_noise.radial_error_m,
              config_.armor_observation_noise.height_error_m,
              config_.armor_observation_noise.yaw_error_rad,
          }),
          A(0),
          R(k),
          X(k),
          armor.position,
          armor.yaw.theta(),
          index,
          config_.radius_min,
          config_.radius_max,
      });
    } else {
      graph.add(ArmorRBDZFactor{
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
              config_.armor_observation_noise.tangential_error_m,
              config_.armor_observation_noise.radial_error_m,
              config_.armor_observation_noise.height_error_m,
              config_.armor_observation_noise.yaw_error_rad,
          }),
          B(0),
          Z(0),
          R(k),
          X(k),
          armor.position,
          armor.yaw.theta(),
          index,
          config_.radius_min,
          config_.radius_max,
      });
    }
  }
  LOG_TRACE_L1(logger_, "[Target {}]: add {} armor factors. k = {}.",
               rfl::enum_to_string(target_state_.type), armor_indexs.size(), k);
}

std::pair<auto_aim::RobotTargetState, auto_aim::TrackState::State>
auto_aim::RobotTarget::update(const std::vector<ArmorPositionYaw> &armors,
                              double dt) const {
  auto dt_tracking_to_update =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          track_state_.stamp_last_update - track_state_.stamp_last_tracking)
          .count();
  if (track_state_.state != TrackState::State::LOST &&
      dt + dt_tracking_to_update > config_.lost_threshold_sec) {
    LOG_INFO(logger_, "[Target {}]: Time out! dt{}.",
             rfl::enum_to_string(target_state_.type),
             dt + dt_tracking_to_update);
    return {{}, TrackState::State::LOST};
  }
  auto target_state = target_state_.predict(dt);
  if (track_state_.state == TrackState::State::LOST) {
    if (armors.empty()) {
      return {{}, TrackState::State::LOST};
    } else {
      target_state = getTargetStateFromArmor(armors.front());
    }
  }
  auto matched_armors = matchArmorsUnique(target_state, armors);
  if (matched_armors.size() < armors.size()) {
    LOG_DEBUG(logger_, "[Target {}]: Miss match {} armors! k = {}.",
              rfl::enum_to_string(target_state.type),
              armors.size() - matched_armors.size(), track_state_.k);
  }

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  addMotionValuesFactors(values, graph, target_state, track_state_.k, dt);
  addArmorValuesFactors(values, graph, matched_armors, track_state_.k);

  try {
    this->isam2_.update(graph, values);
    target_state.center_position =
        isam2_.calculateEstimate<gtsam::Point3>(X(track_state_.k));
    target_state.center_velocity =
        isam2_.calculateEstimate<gtsam::Vector3>(V(track_state_.k));
    target_state.center_yaw =
        isam2_.calculateEstimate<gtsam::Rot2>(R(track_state_.k)).theta();
    target_state.center_vyaw =
        isam2_.calculateEstimate<double>(W(track_state_.k));
    target_state.radius_a =
        tools::logisticFunction(isam2_.calculateEstimate<double>(A(0)),
                                config_.radius_min, config_.radius_max);
    target_state.radius_b =
        tools::logisticFunction(isam2_.calculateEstimate<double>(B(0)),
                                config_.radius_min, config_.radius_max);
    target_state.dz = isam2_.calculateEstimate<double>(Z(0));
    return {target_state, matched_armors.empty() ? TrackState::State::TEMPLOST
                                                 : TrackState::State::TRACKING};
  } catch (const std::exception &e) {
    LOG_ERROR(logger_, "[Target {}]: {}\ncurrent k: {}.",
              rfl::enum_to_string(target_state.type), e.what(), track_state_.k);
    return {{}, TrackState::State::LOST};
  }
}
