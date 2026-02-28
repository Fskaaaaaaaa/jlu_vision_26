// Copyright (c) 2026 shuodedaoli. All Rights Reserved.
#include "target.hpp"
#include "configs.hpp"
#include "factors.hpp"
#include "math/angle_tools.hpp"
#include "math/sigmoid_functions.hpp"
#include "types.hpp"
#include "types/ArmorType.hpp"

#include "quill/LogMacros.h"
#include "rfl/enums.hpp"
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
#include <iterator>
#include <mutex>
#include <utility>
#include <vector>

auto_aim::Target::Target(quill::Logger *logger, const TargetConfig &config,
                         types::ArmorType type)
    : logger_(logger), config_(config), type_(type),
      stamp_last_tracking_(std::chrono::system_clock::from_time_t(0)) {}

auto_aim::TargetStatus
auto_aim::Target::getStatusFromArmor(const Armor &armor, TrackStatus status,
                                     std::uint64_t k) const {
  bool is_outpost = (type_ == types::ArmorType::Outpost);
  auto r = is_outpost ? config_.outpost.default_radius
                      : config_.robot.default_radius_a;
  auto xa = armor.position.x();
  auto ya = armor.position.y();
  auto xc = xa + r * std::cos(armor.yaw.theta());
  auto yc = ya + r * std::sin(armor.yaw.theta());
  return (is_outpost)
             ? TargetStatus{.type = type_,
                            .radius = config_.outpost.default_radius,
                            .dz_a = config_.outpost.default_dz_a,
                            .dz_b = config_.outpost.default_dz_b,
                            .center_position =
                                Eigen::Vector3d{xc, yc, armor.position.z()},
                            .center_velocity = Eigen::Vector3d::Zero(),
                            .center_yaw = armor.yaw.theta(),
                            .center_vyaw = 0.0,
                            .track_status = status,
                            .k = k,}
             : TargetStatus{
                   .type = type_,
                   .radius_a = config_.robot.default_radius_a,
                   .radius_b = config_.robot.default_radius_b,
                   .dz = config_.robot.default_dz,
                   .center_position =
                       Eigen::Vector3d{xc, yc, armor.position.z()},
                   .center_velocity = Eigen::Vector3d::Zero(),
                   .center_yaw = armor.yaw.theta(),
                   .center_vyaw = 0.0,
                   .track_status = status,
                   .k = k,
               };
}

std::vector<std::pair<auto_aim::ArmorIndex, auto_aim::ArmorMatchError>>
auto_aim::Target::matchArmor(const Armor &armor_obs, double dt_sec) const {
  auto armors = status_.predict(dt_sec).armors();
  std::vector<std::pair<ArmorIndex, ArmorMatchError>> idx_errors;
  for (const auto &armor_pre : armors) {
    auto distance = (armor_obs.position - armor_pre.position).norm();
    auto yaw_diff = std::abs(armor_obs.yaw.localCoordinates(armor_pre.yaw).x());
    if (distance < config_.max_match_distance_m &&
        yaw_diff < tools::angle2Radian(config_.max_match_yaw_diff_degree)) {
      idx_errors.emplace_back(armor_pre.index,
                              ArmorMatchError{distance, yaw_diff});
    }
  }
  // 因为PNPyaw不准确，所以优先距离排序
  std::sort(idx_errors.begin(), idx_errors.end(),
            [this](const std::pair<ArmorIndex, ArmorMatchError> &a,
                   const std::pair<ArmorIndex, ArmorMatchError> &b) -> bool {
              return a.second.distance < b.second.distance;
            });
  // 满足条件的都是匹配到的装甲板（虽然大多数情况只有一块）
  // 怎么处理是后续方法的事情了
  return idx_errors;
}

std::pair<auto_aim::TargetStatus, std::chrono::system_clock::time_point>
auto_aim::Target::getStatusStamp() const {
  std::scoped_lock lk{status_mtx_};
  return {status_, stamp_last_tracking_};
}

auto_aim::TargetStatus
auto_aim::Target::track(const std::vector<types::Armor> &armors,
                        const std::chrono::system_clock::time_point &stamp) {
  std::vector<Armor> selected_armors;
  // 调用auto_aim::Armor对types::Armor的构造函数
  std::copy_if(armors.begin(), armors.end(),
               std::back_inserter(selected_armors),
               [this](const types::Armor &armor) -> bool {
                 return armor.type == type_;
               });
  double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                  stamp - stamp_last_tracking_)
                  .count();
  auto status_updated = (type_ == types::ArmorType::Base)
                            ? updateBase(selected_armors)
                        : (type_ == types::ArmorType::Outpost)
                            ? updateOutpost(selected_armors, dt)
                            : updateRobot(selected_armors, dt);
  if (status_updated.track_status == TrackStatus::Tracking) {
    LOG_DEBUG(logger_, "[Target {}]: Updated. dt{}, k{}.",
              rfl::enum_to_string(type_), dt, status_.k);
    // 更新成功就更新状态
    std::scoped_lock lk{status_mtx_};
    this->stamp_last_tracking_ = stamp; // 只在有观测时更新时间戳
    this->status_ = status_updated;
    status_.k += 1;
  } else if (status_updated.track_status == TrackStatus::TempLost) {
    LOG_INFO(logger_, "[Target {}]: Temp lost! dt{}, k{}.",
             rfl::enum_to_string(type_), dt, status_.k);
    std::scoped_lock lk{status_mtx_};
    this->status_ = status_updated;
    // TODO 再检查一下时间戳相关
    status_.k += 1;
  } else if (status_updated.track_status == TrackStatus::Lost &&
             status_.track_status != TrackStatus::Lost) {
    LOG_INFO(logger_, "[Target {}]: Reset! dt{}, k{}.",
             rfl::enum_to_string(type_), dt, status_.k);
    std::scoped_lock lk{status_mtx_};
    this->isam2_ = gtsam::ISAM2{};
    this->status_.track_status = TrackStatus::Lost;
    status_.k = 0;
  }
  return status_;
}

using namespace gtsam::symbol_shorthand;

void auto_aim::Target::addMotionValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
    TargetStatus status, double dt) const {
  auto status_predict = status.predict(dt);
  Eigen::Vector3d predict_position = status_predict.center_position;
  gtsam::Rot2 predict_yaw = status_predict.center_yaw;
  values.insert(X(status.k), predict_position);
  values.insert(R(status.k), predict_yaw);
  values.insert(V(status.k), status_predict.center_velocity);
  values.insert(W(status.k), status_predict.center_vyaw);
  if (status.k == 0) {
    graph.addPrior(X(0), status_predict.center_position,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.translation_prior_noise.x,
                       config_.translation_prior_noise.y,
                       config_.translation_prior_noise.z,
                   }));
    graph.addPrior(
        R(0), gtsam::Rot2::fromAngle(status_.center_yaw),
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_prior_noise));
    graph.addPrior(V(0), status_predict.center_velocity,
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.velocity_prior_noise.x,
                       config_.velocity_prior_noise.y,
                       config_.velocity_prior_noise.z,
                   }));
    graph.addPrior(
        W(0), status_predict.center_vyaw,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_prior_noise));
  } else {
    graph.add(
        TranslationFactor{gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                              config_.translation_factor_noise.x,
                              config_.translation_factor_noise.y,
                              config_.translation_factor_noise.z,
                          }),
                          X(status.k - 1), V(status.k - 1), X(status.k), dt});
    graph.add(YawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_factor_noise),
        R(status.k - 1), W(status.k - 1), R(status.k), dt});
    graph.add(VelocityFactor{gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                                 config_.velocity_factor_noise.x,
                                 config_.velocity_factor_noise.y,
                                 config_.velocity_factor_noise.z,
                             }),
                             V(status.k - 1), V(status.k)});
    graph.add(VyawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_factor_noise),
        W(status.k - 1), W(status.k)});
  }
}

auto_aim::TargetStatus
auto_aim::Target::updateRobot(const std::vector<Armor> &armors,
                              double dt) const {
  // 首帧观测缺失，为欠约束系统，返回失败
  if (status_.track_status == TrackStatus::Lost && armors.empty())
    return {.track_status = TrackStatus::Lost};

  auto status = status_;
  if (status_.track_status != TrackStatus::Lost &&
      dt > config_.robot.lost_threshold_sec) {
    LOG_INFO(logger_, "[Target {}]: Time out! dt{}.",
             rfl::enum_to_string(type_), dt);
    if (armors.empty())
      return {.track_status = TrackStatus::Lost};
    isam2_ = gtsam::ISAM2{};
    status = getStatusFromArmor(armors.front(), TrackStatus::Lost, 0);
    LOG_DEBUG(logger_,
              "[Target {}]: get status from armor! xa{}, ya{}, xc{}, yc{}",
              rfl::enum_to_string(type_), armors.front().position.x(),
              armors.front().position.y(), status.center_position.x(),
              status.center_position.y());
  }

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  // 首帧观测初始化常量并添加先验
  if (status.k == 0) {
    values.insert(A(0), config_.robot.default_radius_a);
    values.insert(B(0), config_.robot.default_radius_b);
    values.insert(Z(0), config_.robot.default_dz);
    graph.addPrior(
        A(0),
        tools::logisticInverse(config_.robot.default_radius_a,
                               config_.robot.radius_min,
                               config_.robot.radius_max),
        gtsam::noiseModel::Isotropic::Sigma(1, config_.robot.ra_prior_noise));
    graph.addPrior(
        B(0),
        tools::logisticInverse(config_.robot.default_radius_b,
                               config_.robot.radius_min,
                               config_.robot.radius_max),
        gtsam::noiseModel::Isotropic::Sigma(1, config_.robot.rb_prior_noise));
    graph.addPrior(
        Z(0), config_.robot.default_dz,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.robot.dz_prior_noise));
  }
  // 添加运动因子或先验
  addMotionValuesFactors(values, graph, status, dt);

  // 添加装甲板观测因子，非首帧时aromrs可以为空（直到超时）
  bool has_obs_factor{false};
  for (const auto &obs : armors) {
    auto matched_indexs = matchArmor(obs, dt);
    if (matched_indexs.empty()) {
      LOG_WARNING(logger_, "[Target {}]: No armor matched obs:xa{},ya{}!",
                  rfl::enum_to_string(type_), obs.position.x(),
                  obs.position.y());
      continue;
    } else {
      has_obs_factor = true;
    }
    auto index = matched_indexs.front().first;
    if (index == ArmorIndex::_0 || index == ArmorIndex::_2) {
      graph.add(ArmorRadiusAFactor{
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
              config_.robot.ra_factor_noise.x,
              config_.robot.ra_factor_noise.y,
              config_.robot.ra_factor_noise.z,
              config_.robot.ra_factor_noise.yaw,
          }),
          A(0),
          R(status.k),
          X(status.k),
          obs.position,
          obs.yaw.theta(),
          index,
          config_.robot.radius_min,
          config_.robot.radius_max,
      });
    } else {
      graph.add(ArmorRadiusBDZFactor{
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
              config_.robot.rbdz_factor_noise.x,
              config_.robot.rbdz_factor_noise.y,
              config_.robot.rbdz_factor_noise.z,
              config_.robot.rbdz_factor_noise.yaw,
          }),
          B(0),
          Z(0),
          R(status.k),
          X(status.k),
          obs.position,
          obs.yaw.theta(),
          index,
          config_.robot.radius_min,
          config_.robot.radius_max,
      });
    }
  } // end of armors loop

  try {
    // 进行增量优化并返回结果
    this->isam2_.update(graph, values);
    return TargetStatus{
        .type = type_,
        .radius_a = tools::logisticFunction(
            isam2_.calculateEstimate<double>(A(0)), config_.robot.radius_min,
            config_.robot.radius_max),
        .radius_b = tools::logisticFunction(
            isam2_.calculateEstimate<double>(B(0)), config_.robot.radius_min,
            config_.robot.radius_max),
        .dz = isam2_.calculateEstimate<double>(Z(0)),
        .center_position = isam2_.calculateEstimate<gtsam::Point3>(X(status.k)),
        .center_velocity =
            isam2_.calculateEstimate<gtsam::Vector3>(V(status.k)),
        .center_yaw =
            isam2_.calculateEstimate<gtsam::Rot2>(R(status.k)).theta(),
        .center_vyaw = isam2_.calculateEstimate<double>(W(status.k)),
        .track_status =
            has_obs_factor ? TrackStatus::Tracking : TrackStatus::TempLost,
        .k = status.k,
    };
  } catch (const std::exception &e) {
    LOG_ERROR(logger_, "[Target {}]: {}\ncurrent k: {}.",
              rfl::enum_to_string(type_), e.what(), status.k);
    return {.track_status = TrackStatus::Lost};
  }
}

auto_aim::TargetStatus
auto_aim::Target::updateBase(const std::vector<Armor> &armors) const {
  if (armors.empty())
    return {.track_status = TrackStatus::Lost};
  Eigen::Vector3d position = armors.front().position;
  double yaw = armors.front().yaw.theta();
  return TargetStatus{
      .type = type_,
      .radius_a = 0,
      .radius_b = 0,
      .dz = 0,
      .center_position = position,
      .center_velocity = Eigen::Vector3d::Zero(),
      .center_yaw = yaw,
      .center_vyaw = 0,
      .track_status = TrackStatus::Tracking,
  };
}
