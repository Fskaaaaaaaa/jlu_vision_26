// Copyright (c) 2026 Fenghr. All Rights Reserved.
#include "factors.hpp"
#include "math/sigmoid_functions.hpp"
#include "types.hpp"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>

auto_aim::TranslationFactor::TranslationFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key x_pre, gtsam::Key v_pre,
    gtsam::Key x_cur, double dt)
    : Base(model, x_pre, v_pre, x_cur), dt_(dt) {}

gtsam::Vector auto_aim::TranslationFactor::evaluateError(
    const gtsam::Point3 &x_pre, const gtsam::Vector3 &v_pre,
    const gtsam::Point3 &x_cur, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const {
  gtsam::Vector3 error = x_cur - (x_pre + v_pre * dt_);
  if (H1)
    *H1 = -gtsam::Matrix3::Identity();
  if (H2)
    *H2 = -dt_ * gtsam::Matrix3::Identity();
  if (H3)
    *H3 = gtsam::Matrix3::Identity();
  return error;
}

auto_aim::YawFactor::YawFactor(const gtsam::SharedNoiseModel &model,
                               gtsam::Key r_pre, gtsam::Key w_pre,
                               gtsam::Key r_cur, double dt)
    : Base(model, r_pre, w_pre, r_cur), dt_(dt) {}

gtsam::Vector auto_aim::YawFactor::evaluateError(
    const gtsam::Rot2 &r_pre, const double &w_pre, const gtsam::Rot2 &r_cur,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3) const {
  gtsam::Vector1 error =
      (r_pre * gtsam::Rot2::fromAngle(w_pre * dt_)).localCoordinates(r_cur);
  if (H1)
    *H1 = gtsam::Matrix::Constant(1, 1, -1.0);
  if (H2)
    *H2 = gtsam::Matrix::Constant(1, 1, -dt_);
  if (H3)
    *H3 = gtsam::Matrix::Identity(1, 1);
  return error;
}

auto_aim::VelocityFactor::VelocityFactor(const gtsam::SharedNoiseModel &model,
                                         gtsam::Key v_pre, gtsam::Key v_cur)
    : Base(model, v_pre, v_cur) {}

gtsam::Vector auto_aim::VelocityFactor::evaluateError(
    const gtsam::Vector3 &r_pre, const gtsam::Vector3 &r_cur,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const {
  gtsam::Vector3 error = r_cur - r_pre;
  if (H1)
    *H1 = -gtsam::Matrix3::Identity();
  if (H2)
    *H2 = gtsam::Matrix3::Identity();
  return error;
}

auto_aim::VyawFactor::VyawFactor(const gtsam::SharedNoiseModel &model,
                                 gtsam::Key w_pre, gtsam::Key w_cur)
    : Base(model, w_pre, w_cur) {}

gtsam::Vector
auto_aim::VyawFactor::evaluateError(const double &w_pre, const double &w_cur,
                                    gtsam::OptionalMatrixType H1,
                                    gtsam::OptionalMatrixType H2) const {
  gtsam::Vector1 error{w_cur - w_pre};
  if (H1)
    *H1 = -gtsam::Matrix1::Identity();
  if (H2)
    *H2 = gtsam::Matrix1::Identity();
  return error;
}

auto_aim::ArmorRadiusAFactor::ArmorRadiusAFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key rad_a, gtsam::Key rot_cur,
    gtsam::Key x_cur, const Eigen::Vector3d &obs_armor_position,
    double obs_armor_yaw, ArmorIndex armor_index, double radius_min,
    double radius_max)
    : Base(model, rad_a, rot_cur, x_cur),
      obs_armor_position_(obs_armor_position),
      obs_armor_yaw_(gtsam::Rot2::fromAngle(obs_armor_yaw)),
      armor_index_(armor_index), min_(radius_min), max_(radius_max) {}

gtsam::Vector auto_aim::ArmorRadiusAFactor::evaluateError(
    const double &ra, const gtsam::Rot2 &rotation, const gtsam::Point3 &center,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3) const {
  // NOTE: 因子图不直接优化半径
  auto radius_a = tools::logisticFunction(ra, min_, max_);
  Armor armor{center, rotation.theta(), radius_a, armor_index_};
  gtsam::Vector3 pos_err = armor.position - obs_armor_position_;
  auto yaw_err = obs_armor_yaw_.localCoordinates(armor.yaw).x();
  gtsam::Vector4 error{pos_err.x(), pos_err.y(), pos_err.z(), yaw_err};
  // HACK: 注意armor.yaw是直接由rotation计算来的，所以算雅可比直接用了
  // NOTE: 注意error减去的obs相当于常数，故相当于求armor(xyzyaw)对输入的雅可比
  if (H1) {
    auto d_radius_d_ra = tools::logisticDerivative(radius_a, min_, max_);
    (*H1) =
        (gtsam::Matrix(4, 1) << -std::cos(armor.yaw.theta()) * d_radius_d_ra,
         -std::sin(armor.yaw.theta()) * d_radius_d_ra, 0.0, 0.0)
            .finished();
  }
  if (H2) {
    (*H2) = (gtsam::Matrix(4, 1) << radius_a * std::sin(armor.yaw.theta()),
             -radius_a * std::cos(armor.yaw.theta()), 0.0, 1.0)
                .finished();
  }
  if (H3) {
    (*H3) =
        (gtsam::Matrix(4, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0).finished();
  }
  return error;
}

auto_aim::ArmorRadiusBDZFactor::ArmorRadiusBDZFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key rad_b, gtsam::Key dz,
    gtsam::Key rot_cur, gtsam::Key x_cur,
    const Eigen::Vector3d &obs_armor_position, double obs_armor_yaw,
    ArmorIndex armor_index, double radius_min, double radius_max)
    : Base(model, rad_b, dz, rot_cur, x_cur),
      obs_armor_position_(obs_armor_position),
      obs_armor_yaw_(gtsam::Rot2::fromAngle(obs_armor_yaw)),
      armor_index_(armor_index), min_(radius_min), max_(radius_max) {}

gtsam::Vector auto_aim::ArmorRadiusBDZFactor::evaluateError(
    const double &rb, const double &dz, const gtsam::Rot2 &rotation,
    const gtsam::Point3 &center, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4) const {
  auto radius_b = tools::logisticFunction(rb, min_, max_);
  Armor armor{center, rotation.theta(), radius_b, dz, armor_index_};
  gtsam::Vector3 pos_err = armor.position - obs_armor_position_;
  auto yaw_err = obs_armor_yaw_.localCoordinates(armor.yaw).x();
  gtsam::Vector4 error{pos_err.x(), pos_err.y(), pos_err.z(), yaw_err};
  if (H1) {
    auto d_radius_d_rb = tools::logisticDerivative(radius_b, min_, max_);
    (*H1) =
        (gtsam::Matrix(4, 1) << -std::cos(armor.yaw.theta()) * d_radius_d_rb,
         -std::sin(armor.yaw.theta()) * d_radius_d_rb, 0.0, 0.0)
            .finished();
  }
  if (H2) {
    (*H2) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 1.0, 0.0).finished();
  }
  if (H3) {
    (*H3) = (gtsam::Matrix(4, 1) << radius_b * std::sin(armor.yaw.theta()),
             -radius_b * std::cos(armor.yaw.theta()), 0.0, 1.0)
                .finished();
  }
  if (H4) {
    (*H4) =
        (gtsam::Matrix(4, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0).finished();
  }
  return error;
}
