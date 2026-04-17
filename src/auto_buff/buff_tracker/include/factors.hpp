#pragma once

#include "types.hpp"
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <opencv2/core.hpp>

namespace auto_buff {

// 零速模型，仅做帧间约束
class PositionFactor
    : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3>;

public:
  PositionFactor(const gtsam::SharedNoiseModel &model, gtsam::Key x_pre,
                 gtsam::Key x_cur);

  gtsam::Vector evaluateError(const gtsam::Point3 &x_pre,
                              const gtsam::Point3 &x_cur,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2) const override;
};

// NOTE: 这个是给小符用的匀速约束（和auto_aim的一样），大符的再说
class RollFactor
    : public gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2>;

public:
  RollFactor(const gtsam::SharedNoiseModel &model, gtsam::Key r_pre,
             gtsam::Key w_pre, gtsam::Key r_cur, double dt);
  gtsam::Vector evaluateError(const gtsam::Rot2 &r_pre, const double &w_pre,
                              const gtsam::Rot2 &r_cur,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2,
                              gtsam::OptionalMatrixType H3) const override;

private:
  double dt_;
};

// NOTE: 这个是给小符用的匀速约束（和auto_aim的一样），大符的再说
class VRollFactor : public gtsam::NoiseModelFactorN<double, double> {
  using Base = gtsam::NoiseModelFactorN<double, double>;

public:
  VRollFactor(const gtsam::SharedNoiseModel &model, gtsam::Key w_pre,
              gtsam::Key w_cur);

  gtsam::Vector evaluateError(const double &w_pre, const double &w_cur,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2) const override;
};

// NOTE:
// 一个扇叶对应着五个keypoint，N个观测会产生N*5个重投影因子，优化N个扇叶位姿
class BuffBladeReprojFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

public:
  BuffBladeReprojFactor(const gtsam::SharedNoiseModel &model,
                        gtsam::Key buff_blade_pose_key,
                        const cv::Mat &camera_matrix,
                        const cv::Mat &distortion_coefficients,
                        BuffIndex buff_index, BuffPointPosition point_position,
                        Eigen::Vector2d px_point);
  gtsam::Vector evaluateError(const gtsam::Pose3 &armor_pose_camera,
                              gtsam::OptionalMatrixType H) const override;

private:
  gtsam::Point2 px_point_;
  gtsam::Point3 buff_point_;
  gtsam::Cal3DS2 calib_;
};

// NOTE: 每一个扇叶约束因子连接一个扇叶和xyz roll，每一帧可能添加0-5个约束因子
class BuffBladeFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, double, gtsam::Rot2,
                                      gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, double, gtsam::Rot2,
                                        gtsam::Point3>;

public:
  BuffBladeFactor(const gtsam::SharedNoiseModel &model,
                  gtsam::Key armor_pose_key, gtsam::Key radius_key,
                  gtsam::Key center_yaw_key, gtsam::Key center_point_key,
                  const Eigen::Isometry3d &T_camera_to_odom,
                  BuffIndex armor_index, double radius_min, double radius_max);

  gtsam::Vector
  evaluateError(const gtsam::Pose3 &armor_pose_camera, const double &radius,
                const gtsam::Rot2 &center_yaw,
                const gtsam::Point3 &center_point, gtsam::OptionalMatrixType H1,
                gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
                gtsam::OptionalMatrixType H4) const override;

private:
  Eigen::Isometry3d T_camera_to_odom_;
  BuffIndex armor_index_;
  double radius_min_, radius_max_;
};

} // namespace auto_buff
