// Copyright (c) 2026 Fsk. All Rights Reserved.
#pragma once

#include "types.hpp"

#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

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
//          [下面两个观测因子误差定义为由X+A/BZ推导的装甲板位置和直接观测的差(预测-观测)]
//          [观测由构造时传入，注意需要手动匹配装甲板位置(来决定使用哪个因子)]
//          A(0)--ArmorRadiusAFactor--X(k)
//                                 \__R(k)
//          B(0)--ArmorFactorRbDz--X(k)
//          Z(0)__|             \__R(k)

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

class ArmorRadiusAFactor
    : public gtsam::NoiseModelFactorN<double, gtsam::Rot2, gtsam::Point3> {
  using Base = gtsam::NoiseModelFactorN<double, gtsam::Rot2, gtsam::Point3>;

public:
  ArmorRadiusAFactor(const gtsam::SharedNoiseModel &model, gtsam::Key rad_a,
                     gtsam::Key rot_cur, gtsam::Key x_cur,
                     const Eigen::Vector3d &obs_armor_position,
                     double obs_armor_yaw, ArmorIndex armor_index,
                     double radius_min, double radius_max);

  gtsam::Vector evaluateError(const double &ra, const gtsam::Rot2 &rotation,
                              const gtsam::Point3 &center,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2,
                              gtsam::OptionalMatrixType H3) const override;

private:
  gtsam::Vector3 obs_armor_position_;
  gtsam::Rot2 obs_armor_yaw_;
  ArmorIndex armor_index_;
  double min_, max_;
};
class ArmorRadiusBDZFactor
    : public gtsam::NoiseModelFactorN<double, double, gtsam::Rot2,
                                      gtsam::Point3> {
  using Base =
      gtsam::NoiseModelFactorN<double, double, gtsam::Rot2, gtsam::Vector3>;

public:
  ArmorRadiusBDZFactor(const gtsam::SharedNoiseModel &model, gtsam::Key rad_b,
                       gtsam::Key dz, gtsam::Key rot_cur, gtsam::Key x_cur,
                       const Eigen::Vector3d &obs_armor_position,
                       double obs_armor_yaw, ArmorIndex armor_index,
                       double radius_min, double radius_max);

  gtsam::Vector
  evaluateError(const double &rb, const double &dz, const gtsam::Rot2 &rotation,
                const gtsam::Point3 &center, gtsam::OptionalMatrixType H1,
                gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
                gtsam::OptionalMatrixType H4) const override;

private:
  gtsam::Vector3 obs_armor_position_;
  gtsam::Rot2 obs_armor_yaw_;
  ArmorIndex armor_index_;
  double min_, max_;
};

class OutpostFactor {};
} // namespace auto_aim
