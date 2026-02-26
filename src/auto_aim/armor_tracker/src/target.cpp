// Copyright (c) 2026 shuodedaoli. All Rights Reserved.
#include "target.hpp"
#include "configs.hpp"
#include "factors.hpp"
#include "math/sigmoid_functions.hpp"
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
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

auto_aim::Target::Target(quill::Logger *logger, const TargetConfig &config,
                         types::ArmorType type)
    : logger_(logger), config_(config), type_(type),
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
  {
    std::scoped_lock lk{status_mtx_};
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
}

std::pair<auto_aim::ArmorIndex, double>
auto_aim::Target::matchArmor(const Armor &armor, double dt_sec) const {
  const auto armor_idx_vec =
      (type_ == types::ArmorType::Outpost)
          ? std::vector{ArmorIndex::_0, ArmorIndex::_1, ArmorIndex::_2}
      : (type_ == types::ArmorType::Base) ? std::vector{ArmorIndex::_0}
                                          : std::vector{
                                                ArmorIndex::_0,
                                                ArmorIndex::_1,
                                                ArmorIndex::_2,
                                                ArmorIndex::_3,
                                            };
  std::vector<std::pair<ArmorIndex, double>> idx_errors;
  for (auto armor_index : armor_idx_vec) {
    auto armor_predict = Armor::fromTargetStatus(status_, armor_index, dt_sec);
    double pos_error = (armor_predict.position - armor.position).norm();
    // NOTE: 因为单位不一样，yaw的error必须乘个权重
    double yaw_error =
        config_.yaw_error_weight *
        std::abs(armor.yaw.localCoordinates(armor_predict.yaw).x());
    idx_errors.emplace_back(
        armor_index,
        // 将误差映射到0~1区间
        tools::logisticFunction(
            (pos_error + yaw_error) * config_.match_error_scale, -1, 1));
  }
  std::sort(idx_errors.begin(), idx_errors.end(),
            [](const std::pair<ArmorIndex, double> &a,
               const std::pair<ArmorIndex, double> &b) -> bool {
              return a.second < b.second;
            });
  return idx_errors.front();
}

std::pair<std::optional<auto_aim::TargetStatus>,
          std::chrono::system_clock::time_point>
auto_aim::Target::getStatusStamp() const {
  std::scoped_lock lk{status_mtx_};
  if (k_ == 0)
    return {std::nullopt, {}};
  return {status_, stamp_last_update_};
}

std::optional<auto_aim::TargetStatus>
auto_aim::Target::track(const std::vector<types::Armor> &armors,
                        const std::chrono::system_clock::time_point &stamp) {
  try {
    std::vector<auto_aim::Armor> selected_armors;
    // 调用auto_aim::Armor对types::Armor的构造函数
    std::copy_if(armors.begin(), armors.end(),
                 std::back_inserter(selected_armors),
                 [this](const types::Armor &armor) -> bool {
                   return armor.type == type_;
                 });
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                    stamp - stamp_last_update_)
                    .count();
    // 首帧观测时初始化status_
    if (!selected_armors.empty() && k_ == 0) {
      this->initStatus(selected_armors.front().position,
                       selected_armors.front().yaw.theta());
      LOG_INFO(logger_, "[Target {}]: status init.",
               rfl::enum_to_string(type_));
    }
    if (auto status_opt = (type_ == types::ArmorType::Base)
                              ? updateBase(selected_armors)
                          : (type_ == types::ArmorType::Outpost)
                              ? updateOutpost(selected_armors, dt)
                              : updateRobot(selected_armors, dt);
        status_opt.has_value()) {
      // 追踪成功就更新时间戳和索引
      std::scoped_lock lk{status_mtx_};
      this->k_ = k_ + 1;
      this->status_ = status_opt.value();
      this->stamp_last_update_ = stamp;
      return {status_};
    } else if (k_ > 0) {
      // NOTE:追踪失败就且非首帧就重置isam和k
      // 传入armors为空时，update方法只在超时后才会失败
      // 超时后传入空armors，k_一定被转入超时状态时的失败重置为0，
      // 这里判断k_是为了避免重复重置
      std::scoped_lock lk{status_mtx_};
      // NOTE: 注意clear方法不会清空values，所以需要重新构造整个isam对象
      this->isam2_ = gtsam::ISAM2{};
      this->k_ = 0;
      LOG_INFO(logger_, "[Target {}]: reset!", rfl::enum_to_string(type_));
      return std::nullopt;
    } else { // 首帧失败的情况，应该不会遇到
      return std::nullopt;
    }
  } catch (const std::exception &e) {
    // 异常时也重置状态
    std::scoped_lock lk{status_mtx_};
    LOG_ERROR(logger_, "[Target {}]: {}\ncurrent k: {}. reset Target!",
              rfl::enum_to_string(type_), e.what(), k_);
    this->isam2_ = gtsam::ISAM2{};
    this->k_ = 0;
    return std::nullopt;
  }
}

using namespace gtsam::symbol_shorthand;

void auto_aim::Target::addMotionValuesFactors(
    gtsam::Values &values, gtsam::NonlinearFactorGraph &graph, double dt,
    std::uint64_t k) const {
  auto status_predict = status_.predict(dt);
  Eigen::Vector3d predict_position = status_predict.center_position;
  gtsam::Rot2 predict_yaw = status_predict.center_yaw;
  values.insert(X(k_), predict_position);
  values.insert(R(k_), predict_yaw);
  values.insert(V(k_), status_predict.center_velocity);
  values.insert(W(k_), status_predict.center_vyaw);
  if (k == 0) {
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
                          X(k_ - 1), V(k_ - 1), X(k_), dt});
    graph.add(YawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.yaw_factor_noise),
        R(k_ - 1), W(k_ - 1), R(k_), dt});
    graph.add(VelocityFactor{gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{
                                 config_.velocity_factor_noise.x,
                                 config_.velocity_factor_noise.y,
                                 config_.velocity_factor_noise.z,
                             }),
                             V(k_ - 1), V(k_)});
    graph.add(VyawFactor{
        gtsam::noiseModel::Isotropic::Sigma(1, config_.vyaw_factor_noise),
        W(k_ - 1), W(k_)});
  }
}

std::optional<auto_aim::TargetStatus>
auto_aim::Target::updateRobot(const std::vector<Armor> &armors,
                              double dt) const {
  // 超时，返回失败
  if (k_ > 0 && dt > config_.robot.lost_threshold_sec) {
    LOG_INFO(logger_, "[Target {}]: Time out!", rfl::enum_to_string(type_));
    return std::nullopt;
  }
  // 首帧观测缺失，为欠约束系统，返回失败
  if (k_ == 0 && armors.empty())
    return std::nullopt;

  if (!armors.empty()) {
    LOG_DEBUG(logger_, "[Target {}]: receive {} armor(s).",
              rfl::enum_to_string(type_), armors.size());
  }

  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph;
  // 首帧观测初始化常量并添加先验
  if (k_ == 0) {
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
  addMotionValuesFactors(values, graph, dt, k_);

  // TODO
  // FIXME:
  // x1 = x0 + v0
  // v1 = v0
  // 如果没有观测
  // 整个系统可以整体平移，缺少约束
  // 只有一个装甲板观测时也一样（会连着R一起飘走）
  // 而ekf不会飘R纯粹是因为有clamp
  // 另外因为缺少约束，isam时不时会将半径优化成负数，需要一个映射（比如sigmod）
  // 26.2.26: 添加了sigmoid归一半径。依然会飘或反复横跳。
  // 虽然反复reset且状态抖动不可用，但符号和趋势是没问题的

  // 添加装甲板观测因子，非首帧时aromrs可以为空（直到超时）
  for (const auto &armor : armors) {
    auto [index, error] = matchArmor(armor, dt);
    if (error > config_.max_match_error) {
      LOG_WARNING(
          logger_,
          "[Target {}]: Huge armor matching error! index: {}, error: {}!",
          rfl::enum_to_string(type_), static_cast<int>(index), error);
      return std::nullopt;
    }
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
          R(k_),
          X(k_),
          armor.position,
          armor.yaw.theta(),
          index,
          config_.robot.radius_min,
          config_.robot.radius_max,
      });
    }
  } // end of armors loop

  // 进行增量优化并返回结果
  this->isam2_.update(graph, values);

  // 使用模版重载的constcalculateEstimate接口来减小开销
  return TargetStatus{
      .type = type_,
      .radius_a = tools::logisticFunction(
          isam2_.calculateEstimate<double>(A(0)), config_.robot.radius_min,
          config_.robot.radius_max),
      .radius_b = tools::logisticFunction(
          isam2_.calculateEstimate<double>(B(0)), config_.robot.radius_min,
          config_.robot.radius_max),
      .dz = isam2_.calculateEstimate<double>(Z(0)),
      .center_position = isam2_.calculateEstimate<gtsam::Point3>(X(k_)),
      .center_velocity = isam2_.calculateEstimate<gtsam::Vector3>(V(k_)),
      .center_yaw = isam2_.calculateEstimate<gtsam::Rot2>(R(k_)).theta(),
      .center_vyaw = isam2_.calculateEstimate<double>(W(k_)),
  };
}

std::optional<auto_aim::TargetStatus>
auto_aim::Target::updateBase(const std::vector<Armor> &armors) const {
  if (armors.empty())
    return std::nullopt;
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
  };
}
