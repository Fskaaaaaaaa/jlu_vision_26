// Copyright (c) 2026 F. All Rights Reserved.
#include "imu_integrator.hpp"
#include "quill/LogMacros.h"

#include <cmath>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <chrono>
#include <optional>

tf::ImuIntegrator::ImuIntegrator(quill::Logger *logger,
                                 const ImuIntegratorConfig &config)
    : logger_(logger), config_(config) {
  accum_params_ = gtsam::PreintegrationParams::MakeSharedD(config_.gravity);
  accum_params_->setAccelerometerCovariance(
      gtsam::Vector3{config_.accel_noise.at(0), config_.accel_noise.at(1),
                     config_.accel_noise.at(2)}
          .array()
          .square()
          .matrix()
          .asDiagonal());
  accum_params_->setGyroscopeCovariance(gtsam::Vector3{config_.gyro_noise.at(0),
                                                       config_.gyro_noise.at(1),
                                                       config_.gyro_noise.at(2)}
                                            .array()
                                            .square()
                                            .matrix()
                                            .asDiagonal());
  accum_params_->setIntegrationCovariance(gtsam::I_3x3 *
                                          config_.integration_noise);
  accum_params_->setUse2ndOrderCoriolis(true);
  accum_params_->setOmegaCoriolis(gtsam::Vector3(0, 0, 0));
}

using namespace gtsam::symbol_shorthand;

void tf::ImuIntegrator::init(
    const Eigen::Quaterniond &quaternion0, const Eigen::Vector3d &translation0,
    const Eigen::Vector3d &v0,
    const std::chrono::system_clock::time_point &stamp) {
  this->isam_.clear();
  this->accum_ = gtsam::PreintegratedImuMeasurements(accum_params_);
  this->time_index_ = 0;
  this->last_update_stamp_ = stamp;
  this->last_optimize_stamp_ = stamp;
  this->init_graph_ = gtsam::NonlinearFactorGraph{};
  this->optimized_values_.clear();
  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(config_.prior_noise_rpy),
       gtsam::Vector3::Constant(config_.prior_noise_xyz))
          .finished());
  auto velnoise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(config_.vel_noise.at(0), config_.vel_noise.at(1),
                     config_.vel_noise.at(2)));
  auto biasnoise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector6::Constant(config_.bias_noise));
  gtsam::Pose3 pose_0{
      gtsam::Rot3::Quaternion(quaternion0.w(), quaternion0.x(), quaternion0.y(),
                              quaternion0.z()),
      gtsam::Point3{translation0.x(), translation0.y(), translation0.z()}};
  gtsam::Vector3 vel_0{v0.x(), v0.y(), v0.z()};
  realtime_state_ = gtsam::NavState{pose_0, vel_0};
  init_graph_->addPrior(X(0), pose_0, prior_noise);
  init_graph_->addPrior(V(time_index_), vel_0, velnoise);
  init_graph_->addPrior(B(time_index_), gtsam::imuBias::ConstantBias(),
                        biasnoise);
  optimized_values_.insert(X(time_index_), pose_0);
  optimized_values_.insert(V(time_index_), vel_0);
  optimized_values_.insert(B(time_index_), gtsam::imuBias::ConstantBias());
  LOG_INFO(logger_, "imu accum reset!");
  return;
}

tf::IsometryVel tf::ImuIntegrator::update(
    const Eigen::Quaterniond &measured_ori, const Eigen::Vector3d &measured_acc,
    const Eigen::Vector3d &measured_omega,
    const std::chrono::system_clock::time_point &time_point) {
  auto update_dt_sec =
      std::chrono::duration<double>(time_point - last_update_stamp_).count();
  LOG_DEBUG(logger_, "update dt: {}ms", update_dt_sec * 1000);
  last_update_stamp_ = time_point;
  accum_.integrateMeasurement(measured_acc, measured_omega, update_dt_sec);
  auto optimize_dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            time_point - last_optimize_stamp_)
                            .count();
  if (optimize_dt_ms <= config_.update_interval_ms) {
    realtime_state_ = accum_.predict(
        {optimized_values_.at<gtsam::Pose3>(X(time_index_)),
         optimized_values_.at<gtsam::Vector3>(V(time_index_))},
        optimized_values_.at<gtsam::imuBias::ConstantBias>(B(time_index_)));
    return {Eigen::Isometry3d{realtime_state_.pose().matrix()},
            realtime_state_.v()};
  } // 非关键帧 early return

  // 进入关键帧
  this->time_index_++;
  this->last_optimize_stamp_ = time_point;
  LOG_INFO(logger_, "keyframe! current accum_time_index: {}, dt: {}ms",
           time_index_, optimize_dt_ms);
  // 构造因子图
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  if (init_graph_.has_value()) { // 首帧特判
    graph = init_graph_.value();
    init_graph_ = std::nullopt;
    values = optimized_values_; // 避免新增图和isam内部重复
    LOG_DEBUG(logger_, "init_graph reset.");
  }
  gtsam::ImuFactor imu_fator{X(time_index_ - 1), V(time_index_ - 1),
                             X(time_index_),     V(time_index_),
                             B(time_index_ - 1), accum_};
  graph.add(imu_fator);
  gtsam::imuBias::ConstantBias zero_bias(gtsam::Vector3(0, 0, 0),
                                         gtsam::Vector3(0, 0, 0));
  graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(time_index_ - 1), B(time_index_), zero_bias,
      gtsam::noiseModel::Isotropic::Sigma(6, config_.bias_noise)));
  LOG_DEBUG(logger_, "add imu factor and bias factor.");
  auto prev_state =
      config_.use_imu_rpy
          ? gtsam::Pose3{gtsam::Rot3{measured_ori.w(), measured_ori.x(),
                                     measured_ori.y(), measured_ori.z()},
                         realtime_state_.pose().translation()}
          : realtime_state_.pose();
  auto prev_bias =
      optimized_values_.at<gtsam::imuBias::ConstantBias>(B(time_index_ - 1));
  auto prop_state =
      accum_.predict({prev_state, realtime_state_.v()}, prev_bias);
  // 既用init_values放初值，又用里面缓存的上一次优化的结果
  // gtsam::Values values = optimized_values_; // 初始化优化初值
  values.insert(X(time_index_), prop_state.pose());
  values.insert(V(time_index_), prop_state.v());
  values.insert(B(time_index_), prev_bias);
  isam_.update(graph, values);
  optimized_values_ = isam_.calculateEstimate(); // 完成本次关键帧优化并缓存
  accum_.resetIntegrationAndSetBias(             // 重置积分器
      optimized_values_.at<gtsam::imuBias::ConstantBias>(B(time_index_)));
  realtime_state_ = // 将关键帧慢路径的优化结果更新到实时路径上
      gtsam::NavState{optimized_values_.at<gtsam::Pose3>(X(time_index_)),
                      optimized_values_.at<gtsam::Vector3>(V(time_index_))};
  return {Eigen::Isometry3d{realtime_state_.pose().matrix()},
          realtime_state_.v()};
}
