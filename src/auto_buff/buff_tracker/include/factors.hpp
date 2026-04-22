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
class ConstPositionFactor
    : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Point3>;

public:
  ConstPositionFactor(const gtsam::SharedNoiseModel &model, gtsam::Key x_pre,
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
class ConstVRollFactor : public gtsam::NoiseModelFactorN<double, double> {
  using Base = gtsam::NoiseModelFactorN<double, double>;

public:
  ConstVRollFactor(const gtsam::SharedNoiseModel &model, gtsam::Key w_pre,
                   gtsam::Key w_cur);

  gtsam::Vector evaluateError(const double &w_pre, const double &w_cur,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2) const override;
};

// NOTE:
// 一个扇叶对应着五个keypoint，N个观测会产生N*5个重投影因子，优化N个扇叶位姿
// 重投影因子只需要知道自身的点编号（用来访问世界点）
// 从每个重投影因子优化的扇叶的位姿约束整个风车位姿是下面的因子干的活
// 即这个类不需要BladeIndex
class BuffBladeReprojFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

public:
  BuffBladeReprojFactor(const gtsam::SharedNoiseModel &model,
                        gtsam::Key buff_blade_pose_key,
                        const cv::Mat &camera_matrix,
                        const cv::Mat &distortion_coefficients,
                        BuffPointPosition point_position,
                        Eigen::Vector2d px_point);
  gtsam::Vector evaluateError(const gtsam::Pose3 &armor_pose_camera,
                              gtsam::OptionalMatrixType H) const override;

private:
  gtsam::Point2 px_point_;
  gtsam::Point3 buff_point_;
  gtsam::Cal3DS2 calib_;
};

// NOTE: 每一个扇叶约束因子连接一个扇叶和roll xyz，每一帧可能添加0-5个约束因子
class BuffBladeFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Rot2,
                                      gtsam::Point3> {
  using Base =
      gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Rot2, gtsam::Point3>;

public:
  BuffBladeFactor(const gtsam::SharedNoiseModel &model,
                  gtsam::Key buff_blade_pose_key, gtsam::Key roll_key,
                  gtsam::Key center_point_key,
                  const Eigen::Isometry3d &T_camera_to_odom,
                  BuffBladeIndex blade_index);

  // 返回的误差定义为{x y z roll}
  gtsam::Vector evaluateError(const gtsam::Pose3 &buff_blade_pose_camera,
                              const gtsam::Rot2 &center_roll,
                              const gtsam::Point3 &center_point,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2,
                              gtsam::OptionalMatrixType H3) const override;

private:
  Eigen::Isometry3d T_camera_to_odom_;
  BuffBladeIndex blade_index_;
};

} // namespace auto_buff
