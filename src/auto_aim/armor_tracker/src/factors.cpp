// Copyright (c) 2026 Fenghr. All Rights Reserved.
#include "factors.hpp"
#include "math/angle_tools.hpp"
#include "math/sigmoid_functions.hpp"
#include "types.hpp"
#include "types/ArmorPoints.hpp"

#include <gtsam/base/Vector.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <numbers>

inline double getArmorBetweenYawFromIndex(auto_aim::ArmorIndex index) {
  return static_cast<int>(index) * std::numbers::pi / 2.0;
}

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

auto_aim::ArmorRACenterZFactor::ArmorRACenterZFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key rad_a, gtsam::Key rot_cur,
    gtsam::Key x_cur, const Eigen::Vector3d &obs_armor_position,
    double obs_armor_yaw, ArmorIndex armor_index, double radius_min,
    double radius_max)
    : Base(model, rad_a, rot_cur, x_cur),
      obs_armor_position_(obs_armor_position),
      obs_armor_yaw_(gtsam::Rot2::fromAngle(obs_armor_yaw)),
      armor_index_(armor_index), min_(radius_min), max_(radius_max) {}

gtsam::Vector auto_aim::ArmorRACenterZFactor::evaluateError(
    const double &ra, const gtsam::Rot2 &rotation, const gtsam::Point3 &center,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3) const {
  auto radius_a = tools::logisticFunction(ra, min_, max_);
  auto obs_theta = obs_armor_yaw_.theta();
  auto nx = std::cos(obs_theta);
  auto ny = std::sin(obs_theta);
  auto tx = -ny;
  auto ty = nx;
  auto dx = center.x() - obs_armor_position_.x();
  auto dy = center.y() - obs_armor_position_.y();
  auto tangential_err = tx * dx + ty * dy;
  auto radial_err = nx * dx + ny * dy - radius_a;
  auto z_err = center.z() - obs_armor_position_.z();
  auto pred_armor_yaw = gtsam::Rot2::fromAngle(
      rotation.theta() + getArmorBetweenYawFromIndex(armor_index_));
  auto yaw_err = obs_armor_yaw_.localCoordinates(pred_armor_yaw).x();
  gtsam::Vector4 error{tangential_err, radial_err, z_err, yaw_err};
  if (H1) {
    auto d_radius_d_ra = tools::logisticDerivative(radius_a, min_, max_);
    (*H1) = (gtsam::Matrix(4, 1) << 0.0, -d_radius_d_ra, 0.0, 0.0).finished();
  }
  if (H2) {
    (*H2) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 0.0, 1.0).finished();
  }
  if (H3) {
    (*H3) = (gtsam::Matrix(4, 3) << tx, ty, 0.0, nx, ny, 0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0)
                .finished();
  }
  return error;
}

auto_aim::ArmorRBDZFactor::ArmorRBDZFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key rad_b, gtsam::Key dz,
    gtsam::Key rot_cur, gtsam::Key x_cur,
    const Eigen::Vector3d &obs_armor_position, double obs_armor_yaw,
    ArmorIndex armor_index, double radius_min, double radius_max)
    : Base(model, rad_b, dz, rot_cur, x_cur),
      obs_armor_position_(obs_armor_position),
      obs_armor_yaw_(gtsam::Rot2::fromAngle(obs_armor_yaw)),
      armor_index_(armor_index), min_(radius_min), max_(radius_max) {}

gtsam::Vector auto_aim::ArmorRBDZFactor::evaluateError(
    const double &rb, const double &dz, const gtsam::Rot2 &rotation,
    const gtsam::Point3 &center, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4) const {
  auto radius_b = tools::logisticFunction(rb, min_, max_);
  auto obs_theta = obs_armor_yaw_.theta();
  auto nx = std::cos(obs_theta);
  auto ny = std::sin(obs_theta);
  auto tx = -ny;
  auto ty = nx;
  auto dx = center.x() - obs_armor_position_.x();
  auto dy = center.y() - obs_armor_position_.y();
  auto tangential_err = tx * dx + ty * dy;
  auto radial_err = nx * dx + ny * dy - radius_b;
  auto z_err = center.z() + dz - obs_armor_position_.z();
  auto pred_armor_yaw = gtsam::Rot2::fromAngle(
      rotation.theta() + getArmorBetweenYawFromIndex(armor_index_));
  auto yaw_err = obs_armor_yaw_.localCoordinates(pred_armor_yaw).x();
  gtsam::Vector4 error{tangential_err, radial_err, z_err, yaw_err};
  if (H1) {
    auto d_radius_d_rb = tools::logisticDerivative(radius_b, min_, max_);
    (*H1) = (gtsam::Matrix(4, 1) << 0.0, -d_radius_d_rb, 0.0, 0.0).finished();
  }
  if (H2) {
    (*H2) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 1.0, 0.0).finished();
  }
  if (H3) {
    (*H3) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 0.0, 1.0).finished();
  }
  if (H4) {
    (*H4) = (gtsam::Matrix(4, 3) << tx, ty, 0.0, nx, ny, 0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0)
                .finished();
  }
  return error;
}

auto_aim::ArmorRadiusCenterZFactor::ArmorRadiusCenterZFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key armor_pose_key,
    gtsam::Key radius_key, gtsam::Key center_yaw_key,
    gtsam::Key center_point_key, const Eigen::Isometry3d &T_camera_to_odom,
    ArmorIndex armor_index, double radius_min, double radius_max)
    : Base(model, armor_pose_key, radius_key, center_yaw_key, center_point_key),
      T_camera_to_odom_(T_camera_to_odom), armor_index_(armor_index),
      radius_min_(radius_min), radius_max_(radius_max) {}

gtsam::Vector auto_aim::ArmorRadiusCenterZFactor::evaluateError(
    const gtsam::Pose3 &armor_pose_camera, const double &radius,
    const gtsam::Rot2 &center_yaw, const gtsam::Point3 &center_point,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
    gtsam::OptionalMatrixType H3, gtsam::OptionalMatrixType H4) const {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  pose.pretranslate(armor_pose_camera.translation());
  pose.rotate(armor_pose_camera.rotation().matrix());
  Eigen::Isometry3d armor_pose_odom = T_camera_to_odom_ * pose;
  auto armor_yaw = gtsam::Rot2::fromAngle(
      tools::rotationMatrixToRPY(armor_pose_odom.rotation().matrix()).z());
  Eigen::Vector3d armor_position = armor_pose_odom.translation();
  auto radius_b = tools::logisticFunction(radius, radius_min_, radius_max_);
  auto nx = std::cos(armor_yaw.theta());
  auto ny = std::sin(armor_yaw.theta());
  auto tx = -ny;
  auto ty = nx;
  auto dx = center_point.x() - armor_position.x();
  auto dy = center_point.y() - armor_position.y();
  auto tangential_err = tx * dx + ty * dy;
  auto radial_err = nx * dx + ny * dy - radius_b;
  auto z_err = center_point.z() - armor_position.z();
  auto pred_armor_yaw = gtsam::Rot2::fromAngle(
      center_yaw.theta() + getArmorBetweenYawFromIndex(armor_index_));
  auto yaw_err = armor_yaw.localCoordinates(pred_armor_yaw).x();
  gtsam::Vector4 error{tangential_err, radial_err, z_err, yaw_err};
  if (H1) {
    // auto error_func = [this, &radius, &center_yaw, &center_point](
    //                       const gtsam::Pose3 &pose) -> gtsam::Vector {
    //   return this->evaluateError(pose, radius, center_yaw, center_point,
    //                              nullptr, nullptr, nullptr, nullptr);
    // };
    // (*H1) = numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    //     error_func, armor_pose_camera, 1e-6);
    Eigen::Matrix<double, 4, 3> J_p;
    J_p << -tx, -ty, 0.0, -nx, -ny, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0;
    auto radial_proj = nx * dx + ny * dy;
    Eigen::Matrix<double, 4, 1> d_e_d_psi;
    d_e_d_psi << -radial_proj, tangential_err, 0.0, -1.0;
    Eigen::Vector3d rpy =
        tools::rotationMatrixToRPY(armor_pose_odom.rotation().matrix());
    auto roll = rpy.x();
    auto pitch = rpy.y();
    auto cos_pitch = std::cos(pitch);
    Eigen::Matrix<double, 1, 3> d_psi_d_omega;
    // Pose3 的局部扰动使用右乘重traction，旋转分量是 body-frame omega
    d_psi_d_omega << 0.0, std::sin(roll) / cos_pitch,
        std::cos(roll) / cos_pitch;
    Eigen::Matrix<double, 4, 3> J_rot = d_e_d_psi * d_psi_d_omega;
    Eigen::Matrix<double, 4, 3> J_trans =
        J_p * armor_pose_odom.rotation().matrix();
    Eigen::Matrix<double, 4, 6> H1_mat;
    // 注意gtsam的pose3旋转扰动在前，平移扰动在后
    H1_mat.leftCols<3>() = J_rot;
    H1_mat.rightCols<3>() = J_trans;
    (*H1) = H1_mat;
  }
  if (H2) {
    double d_radius_d_radius =
        tools::logisticDerivative(radius_b, radius_min_, radius_max_);
    (*H2) =
        (gtsam::Matrix(4, 1) << 0.0, -d_radius_d_radius, 0.0, 0.0).finished();
  }
  if (H3) {
    (*H3) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 0.0, -1.0).finished();
  }
  if (H4) {
    (*H4) = (gtsam::Matrix(4, 3) << tx, ty, 0.0, nx, ny, 0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0)
                .finished();
  }
  return error;
}

auto_aim::ArmorRadiusDZFactor::ArmorRadiusDZFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key armor_pose_key,
    gtsam::Key radius_key, gtsam::Key dz_key, gtsam::Key center_yaw_key,
    gtsam::Key center_point_key, const Eigen::Isometry3d &T_camera_to_odom,
    ArmorIndex armor_index, double radius_min, double radius_max)
    : Base(model, armor_pose_key, radius_key, dz_key, center_yaw_key,
           center_point_key),
      T_camera_to_odom_(T_camera_to_odom), armor_index_(armor_index),
      radius_min_(radius_min), radius_max_(radius_max) {}

gtsam::Vector auto_aim::ArmorRadiusDZFactor::evaluateError(
    const gtsam::Pose3 &armor_pose_camera, const double &radius,
    const double &dz, const gtsam::Rot2 &center_yaw,
    const gtsam::Point3 &center_point, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4, gtsam::OptionalMatrixType H5) const {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  pose.pretranslate(armor_pose_camera.translation());
  pose.rotate(armor_pose_camera.rotation().matrix());
  Eigen::Isometry3d armor_pose_odom = T_camera_to_odom_ * pose;
  auto armor_yaw = gtsam::Rot2::fromAngle(
      tools::rotationMatrixToRPY(armor_pose_odom.rotation().matrix()).z());
  Eigen::Vector3d armor_position = armor_pose_odom.translation();
  auto radius_b = tools::logisticFunction(radius, radius_min_, radius_max_);
  auto nx = std::cos(armor_yaw.theta());
  auto ny = std::sin(armor_yaw.theta());
  auto tx = -ny;
  auto ty = nx;
  auto dx = center_point.x() - armor_position.x();
  auto dy = center_point.y() - armor_position.y();
  auto tangential_err = tx * dx + ty * dy;
  auto radial_err = nx * dx + ny * dy - radius_b;
  auto z_err = center_point.z() + dz - armor_position.z();
  auto pred_armor_yaw = gtsam::Rot2::fromAngle(
      center_yaw.theta() + getArmorBetweenYawFromIndex(armor_index_));
  auto yaw_err = armor_yaw.localCoordinates(pred_armor_yaw).x();
  gtsam::Vector4 error{tangential_err, radial_err, z_err, yaw_err};
  if (H1) {
    // auto error_func = [this, &radius, &center_yaw, &center_point](
    //                       const gtsam::Pose3 &pose) -> gtsam::Vector {
    //   return this->evaluateError(pose, radius, center_yaw, center_point,
    //                              nullptr, nullptr, nullptr, nullptr);
    // };
    // (*H1) = numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    //     error_func, armor_pose_camera, 1e-6);
    Eigen::Matrix<double, 4, 3> J_p;
    J_p << -tx, -ty, 0.0, -nx, -ny, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0;
    auto radial_proj = nx * dx + ny * dy;
    Eigen::Matrix<double, 4, 1> d_e_d_psi;
    d_e_d_psi << -radial_proj, tangential_err, 0.0, -1.0;
    Eigen::Vector3d rpy =
        tools::rotationMatrixToRPY(armor_pose_odom.rotation().matrix());
    auto roll = rpy.x();
    auto pitch = rpy.y();
    auto cos_pitch = std::cos(pitch);
    Eigen::Matrix<double, 1, 3> d_psi_d_omega;
    // Pose3 的局部扰动使用右乘重traction，旋转分量是 body-frame omega
    d_psi_d_omega << 0.0, std::sin(roll) / cos_pitch,
        std::cos(roll) / cos_pitch;
    Eigen::Matrix<double, 4, 3> J_rot = d_e_d_psi * d_psi_d_omega;
    Eigen::Matrix<double, 4, 3> J_trans =
        J_p * armor_pose_odom.rotation().matrix();
    Eigen::Matrix<double, 4, 6> H1_mat;
    // 注意gtsam的pose3旋转扰动在前，平移扰动在后
    H1_mat.leftCols<3>() = J_rot;
    H1_mat.rightCols<3>() = J_trans;
    (*H1) = H1_mat;
  }
  if (H2) {
    auto d_radius_d_rb =
        tools::logisticDerivative(radius_b, radius_min_, radius_max_);
    (*H2) = (gtsam::Matrix(4, 1) << 0.0, -d_radius_d_rb, 0.0, 0.0).finished();
  }
  if (H3) {
    (*H3) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 1.0, 0.0).finished();
  }
  if (H4) {
    (*H4) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 0.0, 1.0).finished();
  }
  if (H5) {
    (*H5) = (gtsam::Matrix(4, 3) << tx, ty, 0.0, nx, ny, 0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0)
                .finished();
  }
  return error;
}

auto_aim::ArmorReprojFactor::ArmorReprojFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key armor_pose_key,
    const cv::Mat &camera_matrix, const cv::Mat &distortion_coefficients,
    types::ArmorType type, types::ArmorPointPosition point_position,
    Eigen::Vector2d px_point)
    : Base(model, armor_pose_key), px_point_(px_point) {
  auto point_cv = types::points::getArmorPointsCV(type).at(
      static_cast<int>(point_position));
  armor_point_ = gtsam::Point3{point_cv.x, point_cv.y, point_cv.z};
  double fx = camera_matrix.at<double>(0, 0);
  double fy = camera_matrix.at<double>(1, 1);
  double s = camera_matrix.at<double>(0, 1);
  double u0 = camera_matrix.at<double>(0, 2);
  double v0 = camera_matrix.at<double>(1, 2);
  double k1 = distortion_coefficients.at<double>(0, 0);
  double k2 = distortion_coefficients.at<double>(0, 1);
  double p1 = distortion_coefficients.at<double>(0, 2);
  double p2 = distortion_coefficients.at<double>(0, 3);
  calib_ = gtsam::Cal3DS2(fx, fy, s, u0, v0, k1, k2, p1, p2);
}

// NOTE: 这里就需要订阅回调里保留armor相机系的位姿，查询变换后仅传入因子中
gtsam::Vector auto_aim::ArmorReprojFactor::evaluateError(
    const gtsam::Pose3 &armor_pose_camera, gtsam::OptionalMatrixType H) const {
  // 获取点的位姿
  gtsam::Matrix36 H_transform;
  gtsam::Point3 p_cam = armor_pose_camera.transformFrom(
      armor_point_, H ? &H_transform : nullptr, nullptr);
  // if (p_cam.z() <= 1e-6) {
  //   if (H)
  //     *H = gtsam::Matrix::Zero(2, 6);
  //   return gtsam::Vector2::Zero();
  // }
  // 投影
  gtsam::Matrix23 H_norm;
  gtsam::Point2 pn = gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(
      p_cam, H ? &H_norm : nullptr);
  // 添加畸变
  gtsam::Matrix22 H_calib;
  gtsam::Point2 px = calib_.uncalibrate(pn, {}, H ? &H_calib : nullptr);
  if (H)
    *H = H_calib * H_norm * H_transform;
  return px - px_point_;
}
