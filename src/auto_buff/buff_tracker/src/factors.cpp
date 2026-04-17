#include "factors.hpp"
#include "types.hpp"
#include <gtsam/base/Vector.h>

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

auto_buff::VRollFactor::VRollFactor(const gtsam::SharedNoiseModel &model,
                                    gtsam::Key w_pre, gtsam::Key w_cur)
    : Base(model, w_pre, w_cur) {}

gtsam::Vector
auto_buff::VRollFactor::evaluateError(const double &w_pre, const double &w_cur,
                                      gtsam::OptionalMatrixType H1,
                                      gtsam::OptionalMatrixType H2) const {
  gtsam::Vector1 error{w_cur - w_pre};
  if (H1)
    *H1 = -gtsam::Matrix1::Identity();
  if (H2)
    *H2 = gtsam::Matrix1::Identity();
  return error;
}

auto_buff::PositionFactor::PositionFactor(const gtsam::SharedNoiseModel &model,
                                          gtsam::Key x_pre, gtsam::Key x_cur)
    : Base(model, x_pre, x_cur) {}

gtsam::Vector auto_buff::PositionFactor::evaluateError(
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
    BuffIndex buff_index, BuffPointPosition point_position,
    Eigen::Vector2d px_point)
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
