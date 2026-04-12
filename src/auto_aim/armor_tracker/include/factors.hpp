// Copyright (c) 2026 Fsk. All Rights Reserved.
#pragma once

#include "types.hpp"
#include "types/ArmorType.hpp"

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

namespace auto_aim {
// NOTE:
// Point3, Vector3,  Rot2, double, double, double, double
// pose,    velocity, yaw,  vyaw,   radiusA,radiusB,Dz
// X(k),    V(k),     R(k), W(k),   A(0),   B(0),   Z(0)
// 注意，每帧优化前，要向values插入从上一帧位姿预测的本帧位姿作为优化起点
// factors: [运动因子，用来更新位置]
//          V(k-1)--TranslationFactor--X(k)
//          X(k-1)__/
//          R(k-1)--RotationFactor--R(k)
//          W(k-1)__/
//          [CV模型下的速度角速度因子]
//          V(k-1)--VelocityFactor--V(k)
//          W(k-1)--VyawFactor--W(k)
//          [观测因子把中心-装甲板位移分解为切向/径向，半径只由径向项约束]
//          [这样可避免yaw相位误差通过切向残差把半径拉小]
//          A(0)--ArmorRadiusAFactor--X(k)
//                                 \__R(k)
//          B(0)--ArmorFactorRbDz--X(k)
//          Z(0)__|             \__R(k)
// PNPFGO中，连接链路应该是：-四个关键点->重投影因子-pose3d->装甲板观测因子-Point3Rot2->运动约束因子
// 但是这样是否意味着Point3和Rot2的先验无法传递给Pose3？
// 不，应该直接让观测因子连接四个角点，根据Point3、Rot2和pnp得到的roll和pitch得到三维点，再转换到相机系进行投影
// 这样连接四个像素观测的因子返回的误差向量就应该是四个角点的重投影误差了
// 存在同时添加两个装甲板的情况，即使同一个armor之间没有直接的运动因子相连（纯靠观测因子提供先验约束）
// 也需要有独立的变量分配，先暂时给四块装甲板设成F(k)U(k)C(k)K(k)了

class TranslationFactor
    : public gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Vector3,
                                      gtsam::Point3> {
  using Base =
      gtsam::NoiseModelFactorN<gtsam::Point3, gtsam::Vector3, gtsam::Point3>;

public:
  TranslationFactor(const gtsam::SharedNoiseModel &model, gtsam::Key x_pre,
                    gtsam::Key v_pre, gtsam::Key x_cur, double dt);
  gtsam::Vector evaluateError(const gtsam::Point3 &x_pre,
                              const gtsam::Vector3 &v_pre,
                              const gtsam::Point3 &x_cur,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2,
                              gtsam::OptionalMatrixType H3) const override;

private:
  double dt_;
};

class YawFactor
    : public gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Rot2, double, gtsam::Rot2>;

public:
  YawFactor(const gtsam::SharedNoiseModel &model, gtsam::Key r_pre,
            gtsam::Key w_pre, gtsam::Key r_cur, double dt);
  gtsam::Vector evaluateError(const gtsam::Rot2 &r_pre, const double &w_pre,
                              const gtsam::Rot2 &r_cur,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2,
                              gtsam::OptionalMatrixType H3) const override;

private:
  double dt_;
};

class VelocityFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector3>;

public:
  VelocityFactor(const gtsam::SharedNoiseModel &model, gtsam::Key v_pre,
                 gtsam::Key v_cur);

  gtsam::Vector evaluateError(const gtsam::Vector3 &r_pre,
                              const gtsam::Vector3 &r_cur,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2) const override;
};

class VyawFactor : public gtsam::NoiseModelFactorN<double, double> {
  using Base = gtsam::NoiseModelFactorN<double, double>;

public:
  VyawFactor(const gtsam::SharedNoiseModel &model, gtsam::Key w_pre,
             gtsam::Key w_cur);

  gtsam::Vector evaluateError(const double &w_pre, const double &w_cur,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2) const override;
};

class ArmorRadiusCenterZFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, double, gtsam::Rot2,
                                      gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, double, gtsam::Rot2,
                                        gtsam::Point3>;

public:
  ArmorRadiusCenterZFactor(const gtsam::SharedNoiseModel &model,
                           gtsam::Key armor_pose_key, gtsam::Key radius_key,
                           gtsam::Key center_yaw_key,
                           gtsam::Key center_point_key,
                           const Eigen::Isometry3d &T_camera_to_odom,
                           ArmorIndex armor_index, double radius_min,
                           double radius_max);

  gtsam::Vector
  evaluateError(const gtsam::Pose3 &armor_pose_camera, const double &radius,
                const gtsam::Rot2 &center_yaw,
                const gtsam::Point3 &center_point, gtsam::OptionalMatrixType H1,
                gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
                gtsam::OptionalMatrixType H4) const override;

private:
  Eigen::Isometry3d T_camera_to_odom_;
  ArmorIndex armor_index_;
  double radius_min_, radius_max_;
};

class ArmorRadiusDZFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, double, double, gtsam::Rot2,
                                      gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, double, double,
                                        gtsam::Rot2, gtsam::Point3>;

public:
  ArmorRadiusDZFactor(const gtsam::SharedNoiseModel &model,
                      gtsam::Key armor_pose_key, gtsam::Key radius_key,
                      gtsam::Key dz_key, gtsam::Key center_yaw_key,
                      gtsam::Key center_point_key,
                      const Eigen::Isometry3d &T_camera_to_odom,
                      ArmorIndex armor_index, double radius_min,
                      double radius_max, int armor_numbers = 4);

  gtsam::Vector
  evaluateError(const gtsam::Pose3 &armor_pose_camera, const double &radius,
                const double &dz, const gtsam::Rot2 &center_yaw,
                const gtsam::Point3 &center_point, gtsam::OptionalMatrixType H1,
                gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
                gtsam::OptionalMatrixType H4,
                gtsam::OptionalMatrixType H5) const override;

private:
  Eigen::Isometry3d T_camera_to_odom_;
  ArmorIndex armor_index_;
  double radius_min_, radius_max_;
  int armor_numbers_;
};

class ArmorReprojFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

public:
  ArmorReprojFactor(const gtsam::SharedNoiseModel &model,
                    gtsam::Key armor_pose_key, const cv::Mat &camera_matrix,
                    const cv::Mat &distortion_coefficients,
                    types::ArmorType type,
                    types::ArmorPointPosition point_position,
                    Eigen::Vector2d px_point);
  gtsam::Vector evaluateError(const gtsam::Pose3 &armor_pose_camera,
                              gtsam::OptionalMatrixType H) const override;

private:
  gtsam::Point2 px_point_;
  gtsam::Point3 armor_point_;
  gtsam::Cal3DS2 calib_;
};

} // namespace auto_aim
