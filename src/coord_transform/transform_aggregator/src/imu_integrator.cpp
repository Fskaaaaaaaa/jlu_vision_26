// Copyright (c) 2026 F. All Rights Reserved.
#include "imu_integrator.hpp"
#include "quill/LogMacros.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <chrono>

tf::ImuIntegrator::ImuIntegrator(quill::Logger *logger,
                                 const ImuIntegratorConfig &config)
    : logger_(logger), config_(config) {
  accum_params_ = gtsam::PreintegrationParams::MakeSharedU(config_.gravity);
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
  accum_params_->setUse2ndOrderCoriolis(false);
  accum_params_->setOmegaCoriolis(gtsam::Vector3(0, 0, 0));
}

using namespace gtsam::symbol_shorthand;

void tf::ImuIntegrator::init(const Eigen::Quaterniond &quaternion0,
                             const Eigen::Vector3d &translation0,
                             const Eigen::Vector3d &v0) {
  this->isam_.clear();
  this->accum_ = gtsam::PreintegratedImuMeasurements(accum_params_);
  this->graph_ = gtsam::NonlinearFactorGraph{};
  this->estimate_.clear();
  this->time_index_ = 0;
  this->bias_time_index_ = 0;
  this->last_pose_ = gtsam::Pose3{};
  gtsam::Pose3 pose_0{
      gtsam::Rot3::Quaternion(quaternion0.w(), quaternion0.x(), quaternion0.y(),
                              quaternion0.z()),
      gtsam::Point3{translation0.x(), translation0.y(), translation0.z()}};
  auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(config_.prior_noise_rpy),
       gtsam::Vector3::Constant(config_.prior_noise_xyz))
          .finished());
  graph_.addPrior(X(time_index_), pose_0, prior_noise);
  auto biasnoise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector6::Constant(config_.bias_noise));
  graph_.addPrior(B(time_index_), gtsam::imuBias::ConstantBias(), biasnoise);
  estimate_.insert(B(time_index_), gtsam::imuBias::ConstantBias());
  auto velnoise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(config_.vel_noise.at(0), config_.vel_noise.at(1),
                     config_.vel_noise.at(2)));
  gtsam::Vector3 vel_0{v0.x(), v0.y(), v0.z()};
  graph_.addPrior(V(time_index_), vel_0, velnoise);
  estimate_.insert(V(time_index_), vel_0);
  LOG_INFO(logger_, "imu accum reset!");
  return;
}

tf::IsometryVel tf::ImuIntegrator::update(
    const Eigen::Quaterniond &measured_ori, const Eigen::Vector3d &measured_acc,
    const Eigen::Vector3d &measured_omega,
    const std::chrono::system_clock::time_point &time_point) {
  if (this->time_index_ == 0) {
    gtsam::Pose3 pose_guess{{measured_ori.w(), measured_ori.x(),
                             measured_ori.y(), measured_ori.z()},
                            {0, 0, 0}};
    estimate_.insert(X(time_index_), pose_guess);
  } else { // i > 0
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            time_point - last_bias_update_stamp_)
            .count() >= config_.bias_factor_interval_ms) {
      this->bias_time_index_++;
      this->last_bias_update_stamp_ = time_point;
      gtsam::Symbol b1 = B(bias_time_index_ - 1);
      gtsam::Symbol b2 = B(bias_time_index_);
      gtsam::Vector6 covvec{
          config_.accel_bias_rw_sigma.at(0), config_.accel_bias_rw_sigma.at(1),
          config_.accel_bias_rw_sigma.at(2), config_.gyro_bias_rw_sigma.at(0),
          config_.gyro_bias_rw_sigma.at(1),  config_.gyro_bias_rw_sigma.at(2)};
      auto cov = gtsam::noiseModel::Diagonal::Variances(covvec);
      auto f =
          std::make_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
              b1, b2, gtsam::imuBias::ConstantBias(), cov);
      graph_.add(f);
      estimate_.insert(b2, gtsam::imuBias::ConstantBias());
    }
    auto dt = time_point - last_update_stamp_;
    gtsam::Pose3 pose_guess{{measured_ori.w(), measured_ori.x(),
                             measured_ori.y(), measured_ori.z()},
                            last_pose_.translation()};
    estimate_.insert(X(time_index_), pose_guess);
    // Predict acceleration and gyro measurements in (actual) body frame
    accum_.integrateMeasurement(
        measured_acc, measured_omega,
        std::chrono::duration_cast<std::chrono::duration<double>>(dt).count());
    // Add Imu Factor
    gtsam::ImuFactor imufac(X(time_index_ - 1), V(time_index_ - 1),
                            X(time_index_), V(time_index_), B(bias_time_index_),
                            accum_);
    graph_.add(imufac);
  }
  this->isam_.update(graph_, estimate_);
  auto result = isam_.calculateEstimate();
  this->graph_ = gtsam::NonlinearFactorGraph{};
  this->estimate_.clear();
  time_index_++;
  this->last_update_stamp_ = time_point;
  last_pose_ = result.at<gtsam::Pose3>(X(time_index_));
  auto vel = result.at<gtsam::Vector3>(V(time_index_));
  return {Eigen::Isometry3d{last_pose_.matrix()}, vel};
}
