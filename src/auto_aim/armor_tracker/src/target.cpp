// Copyright (c) 2026 shuodedaoli. All Rights Reserved.
#include "target.hpp"
#include "configs.hpp"
#include "factors.hpp"
#include "types.hpp"
#include "types/ArmorType.hpp"

#include "quill/LogMacros.h"
#include "rfl/enums.hpp"
#include <exception>
#include <fcntl.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

auto_aim::Target::Target(quill::Logger *logger, const TargetConfig &config,
                         types::ArmorType type)
    : logger_(logger), config_(config), type_(type), inited_(false),
      stamp_last_update_(std::chrono::system_clock::from_time_t(0)), k_(0) {}

void auto_aim::Target::initStatus(const Eigen::Vector3d &armor_pos,
                                  double armor_yaw) {
  bool outpost = type_ == types::ArmorType::Outpost;
  auto r =
      outpost ? config_.outpost.default_radius : config_.robot.default_radius_a;
  auto xa = armor_pos.x();
  auto ya = armor_pos.y();
  auto xc = xa + r * std::cos(armor_yaw);
  auto yc = ya + r * std::sin(armor_yaw);
  if (outpost) {
    this->status_ = {
        .type = type_,
        .radius = config_.outpost.default_radius,
        .dz_a = config_.outpost.default_dz_a,
        .dz_b = config_.outpost.default_dz_b,
        .center_position = {xc, yc, armor_pos.z()},
        .center_velocity = Eigen::Vector3d::Zero(),
        .center_yaw = armor_yaw,
        .center_vyaw = 0.0,
    };
  } else {
    this->status_ = {
        .type = type_,
        .radius_a = config_.robot.default_radius_a,
        .radius_b = config_.robot.default_radius_b,
        .dz = config_.robot.default_dz,
        .center_position = {xc, yc, armor_pos.z()},
        .center_velocity = Eigen::Vector3d::Zero(),
        .center_yaw = armor_yaw,
        .center_vyaw = 0.0,
    };
  }
}

auto_aim::ArmorIndex auto_aim::Target::matchArmor(const Armor &armor,
                                                  double dt_sec) {
  const auto armor_idx_vec =
      type_ == types::ArmorType::Outpost
          ? std::vector{ArmorIndex::_0, ArmorIndex::_1, ArmorIndex::_2}
          : std::vector{
                ArmorIndex::_0,
                ArmorIndex::_1,
                ArmorIndex::_2,
                ArmorIndex::_3,
            };
  std::vector<std::pair<ArmorIndex, double>> idx_errors;
  for (auto armor_index : armor_idx_vec) {
    auto armor_predict = Armor::fromTargetStatus(status_, armor_index, dt_sec);
    Eigen::Vector3d pos_error = armor_predict.position - armor.position;
    // NOTE: 因为单位不一样，yaw的error必须乘个权重
    double yaw_error = config_.yaw_error_weight *
                       armor.yaw.localCoordinates(armor_predict.yaw).x();
    Eigen::Vector4d error{pos_error.x(), pos_error.y(), pos_error.z(),
                          yaw_error};
    idx_errors.emplace_back(armor_index, error.norm());
  }
  std::sort(idx_errors.begin(), idx_errors.end(),
            [](const std::pair<ArmorIndex, double> &a,
               const std::pair<ArmorIndex, double> &b) -> bool {
              return a.second < b.second;
            });
  return idx_errors.at(0).first;
}

std::optional<auto_aim::TargetStatus>
auto_aim::Target::update(const std::vector<types::Armor> &armors,
                         const std::chrono::system_clock::time_point &stamp) {
  try {
    std::vector<auto_aim::Armor> selected_armors;
    // 调用auto_aim::Armor对types::Armor的构造函数
    std::copy_if(armors.begin(), armors.end(),
                 std::back_inserter(selected_armors),
                 [this](const types::Armor &armor) -> bool {
                   return armor.type == type_;
                 });
    double dt_sec = std::chrono::duration_cast<std::chrono::microseconds>(
                        stamp - stamp_last_update_)
                        .count() /
                    1e6;
    return type_ == types::ArmorType::Outpost
               ? updateOutpost(selected_armors, dt_sec)
               : updateRobot(selected_armors, dt_sec);
  } catch (const std::exception &e) {
    LOG_ERROR(logger_, "[Target {}]: {}", rfl::enum_to_string(type_), e.what());
    return std::nullopt;
  }
}

using namespace gtsam::symbol_shorthand;

std::optional<auto_aim::TargetStatus>
auto_aim::Target::updateRobot(const std::vector<Armor> &armors, double dt) {
  // 超时无新观测就返回并重置，否则进行优化并更新时间戳
  if (armors.empty() && dt >= config_.robot.lost_threshold_sec) {
    this->k_ = 0;
    this->inited_ = false;
    this->isam2_.clear();
    return std::nullopt;
  }
  this->stamp_last_update_ =
      stamp_last_update_ +
      std::chrono::duration_cast<std::chrono::system_clock::duration>(
          std::chrono::duration<double>{dt});

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  // 添加运动因子
  if (inited_) {
    graph.add(
        TranslationFactor{gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                              config_.translation_factor_noise.x,
                              config_.translation_factor_noise.y,
                              config_.translation_factor_noise.z,
                          }),
                          X(k_ - 1), V(k_ - 1), X(k_), dt});
    graph.add(VelocityFactor{gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                                 config_.velocity_factor_noise.vx,
                                 config_.velocity_factor_noise.vy,
                                 config_.velocity_factor_noise.vz,
                             }),
                             V(k_ - 1), V(k_)});
    graph.add(YawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_factor_noise),
        R(k_ - 1), W(k_ - 1), R(k_), dt});
    graph.add(VyawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.robot.vyaw_factor_noise),
        W(k_ - 1), W(k_)});
  }
  // 初始化常量和先验
  // NOTE: 车和前哨要初始化的变量不一样，就不往成员里写了
  auto do_init = [&](const Armor &armor) {
    // 初始化ra、rb、dz
    values.insert(A(0), config_.robot.default_radius_a);
    values.insert(B(0), config_.robot.default_radius_b);
    values.insert(Z(0), config_.robot.default_dz);
    graph.addPrior(
        A(0), config_.robot.default_radius_a,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.robot.ra_prior_noise));
    graph.addPrior(
        B(0), config_.robot.default_radius_b,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.robot.rb_prior_noise));
    graph.addPrior(
        Z(0), config_.robot.default_dz,
        gtsam::noiseModel::Isotropic::Sigma(1, config_.robot.dz_prior_noise));
    // 初始化position、yaw
    initStatus(armor.position, armor.yaw.theta());
    graph.addPrior(X(0), gtsam::Point3{status_.center_position},
                   gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                       config_.translation_prior_noise.x,
                       config_.translation_prior_noise.y,
                       config_.translation_prior_noise.z,
                   }));
    graph.addPrior(
        R(0), gtsam::Rot2::fromAngle(status_.center_yaw),
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_prior_noise));
  };
  // 添加装甲板观测因子
  for (const auto &armor : armors) {
    if (!inited_) {
      do_init(armor);
      inited_ = true;
    }
    auto index = matchArmor(armor, dt);
    if (index == ArmorIndex::_0 || index == ArmorIndex::_2) {
      graph.add(ArmorRadiusAFactor{
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4{
              config_.robot.ra_factor_noise.x,
              config_.robot.ra_factor_noise.y,
              config_.robot.ra_factor_noise.z,
              config_.robot.ra_factor_noise.yaw,
          }),
          A(0),
          R(k_),
          X(k_),
          armor.position,
          armor.yaw.theta(),
          index,
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
          R(k_),
          X(k_),
          armor.position,
          armor.yaw.theta(),
          index,
      });
    }
  } // end of armors loop

  // 构建优化变量
  Eigen::Vector3d predict_position =
      status_.center_position + status_.center_velocity * dt;
  double predict_yaw = status_.center_yaw + status_.center_vyaw * dt;
  values.insert(X(k_), predict_position);
  values.insert(R(k_), predict_yaw);
  values.insert(V(k_), status_.center_velocity);
  values.insert(W(k_), status_.center_vyaw);

  // 进行增量优化并保存结果
  this->isam2_.update(graph, values);
  auto optimized_values = this->isam2_.calculateEstimate();
  status_.center_position = optimized_values.at<gtsam::Point3>(X(k_));
  status_.center_velocity = optimized_values.at<gtsam::Vector3>(V(k_));
  status_.center_yaw = optimized_values.at<gtsam::Rot2>(R(k_)).theta();
  status_.center_vyaw = optimized_values.at<double>(W(k_));
  status_.radius_a = optimized_values.at<double>(A(0));
  status_.radius_b = optimized_values.at<double>(B(0));
  status_.dz = optimized_values.at<double>(Z(0));
  return status_;
}
