#include "factors.hpp"
#include "gtsam/base/Matrix.h"
#include "math/angle_tools.hpp"
#include "types.hpp"

#include <cmath>
#include <gtsam/base/Vector.h>

// HACK:
// 这一版的因子图是不考虑风车的倾斜角的（即假设始终正对着镜头，只有roll的自由度）
// 即没法斜着开符

inline double
getBuffBladeBetweenRollFromIndex(auto_buff::BuffBladeIndex index) {
  return static_cast<int>(index) * (std::numbers::pi * 2.0 / 5);
}

auto_buff::RollFactor::RollFactor(const gtsam::SharedNoiseModel &model,
                                  gtsam::Key r_pre, gtsam::Key w_pre,
                                  gtsam::Key r_cur, double dt)
    : Base(model, r_pre, w_pre, r_cur), dt_(dt) {}

gtsam::Vector auto_buff::RollFactor::evaluateError(
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

auto_buff::ConstVRollFactor::ConstVRollFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key w_pre, gtsam::Key w_cur)
    : Base(model, w_pre, w_cur) {}

gtsam::Vector auto_buff::ConstVRollFactor::evaluateError(
    const double &w_pre, const double &w_cur, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2) const {
  gtsam::Vector1 error{w_cur - w_pre};
  if (H1)
    *H1 = -gtsam::Matrix1::Identity();
  if (H2)
    *H2 = gtsam::Matrix1::Identity();
  return error;
}

auto_buff::ConstPositionFactor::ConstPositionFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key x_pre, gtsam::Key x_cur)
    : Base(model, x_pre, x_cur) {}

gtsam::Vector auto_buff::ConstPositionFactor::evaluateError(
    const gtsam::Point3 &x_pre, const gtsam::Point3 &x_cur,
    gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2) const {
  gtsam::Vector3 error{x_cur - x_pre};
  if (H1)
    *H1 = -gtsam::Matrix3::Identity();
  if (H2)
    *H2 = gtsam::Matrix3::Identity();
  return error;
}

auto_buff::BuffBladeReprojFactor::BuffBladeReprojFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key buff_blade_pose_key,
    const cv::Mat &camera_matrix, const cv::Mat &distortion_coefficients,
    BuffPointPosition point_position, Eigen::Vector2d px_point)
    : Base(model, buff_blade_pose_key), px_point_(px_point) {
  auto point_cv = BUFF_BLADE_OBJ_POINTS.at(static_cast<int>(point_position));
  buff_point_ = gtsam::Point3{point_cv.x, point_cv.y, point_cv.z};
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

gtsam::Vector auto_buff::BuffBladeReprojFactor::evaluateError(
    const gtsam::Pose3 &armor_pose_camera, gtsam::OptionalMatrixType H) const {
  // 获取点的位姿
  gtsam::Matrix36 H_transform;
  gtsam::Point3 p_cam = armor_pose_camera.transformFrom(
      buff_point_, H ? &H_transform : nullptr, nullptr);
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

auto_buff::BuffBladeFactor::BuffBladeFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key buff_blade_pose_key,
    gtsam::Key roll_key, gtsam::Key center_point_key,
    const Eigen::Isometry3d &T_camera_to_odom, BuffBladeIndex blade_index)
    : Base(model, buff_blade_pose_key, roll_key, center_point_key),
      T_camera_to_odom_(T_camera_to_odom), blade_index_(blade_index) {}

gtsam::Vector auto_buff::BuffBladeFactor::evaluateError(
    const gtsam::Pose3 &buff_blade_pose_camera, const gtsam::Rot2 &center_roll,
    const gtsam::Point3 &center_point, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3) const {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  pose.pretranslate(buff_blade_pose_camera.translation());
  pose.rotate(buff_blade_pose_camera.rotation().matrix());
  Eigen::Isometry3d buff_blade_pose_odom = T_camera_to_odom_ * pose;
  auto blade_roll = gtsam::Rot2::fromAngle(
      tools::rotationMatrixToRPY(buff_blade_pose_odom.rotation().matrix()).x());
  Eigen::Vector3d blade_position = buff_blade_pose_odom.translation();
  auto pred_blade_roll = gtsam::Rot2::fromAngle(
      center_roll.theta() + getBuffBladeBetweenRollFromIndex(blade_index_));
  gtsam::Vector3 xyz_err = center_point - blade_position;
  auto roll_error = blade_roll.localCoordinates(pred_blade_roll).x();
  gtsam::Vector4 error{xyz_err.x(), xyz_err.y(), xyz_err.z(), roll_error};
  if (H1) {
    Eigen::Matrix<double, 4, 6> H1_mat = Eigen::Matrix<double, 4, 6>::Zero();
    Eigen::Matrix3d R_odom_blade = buff_blade_pose_odom.rotation().matrix();
    H1_mat.block<3, 3>(0, 3) = -R_odom_blade;
    Eigen::Vector3d rpy =
        tools::rotationMatrixToRPY(buff_blade_pose_odom.rotation().matrix());
    auto roll = rpy.x();
    auto pitch = rpy.y();
    auto tan_pitch = std::tan(pitch);
    Eigen::Matrix<double, 1, 3> d_roll_d_omega;
    // Pose3 的局部扰动使用右乘 retraction，旋转分量是 body-frame omega
    d_roll_d_omega << 1.0, std::sin(roll) * tan_pitch,
        std::cos(roll) * tan_pitch;
    H1_mat.block<1, 3>(3, 0) = -d_roll_d_omega;
    (*H1) = H1_mat;
  }
  if (H2) {
    (*H2) = (gtsam::Matrix(4, 1) << 0.0, 0.0, 0.0, 1.0).finished();
  }
  if (H3) {
    (*H3) = (gtsam::Matrix(4, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0)
                .finished();
  }
  return error;
}
