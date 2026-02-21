#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace auto_aim {

enum {
  X_X,
  X_VX,
  X_Y,
  X_VY,
  X_Z,
  X_VZ,
  X_YAW,
  X_VYAW,
  X_R1, // 前后半径
  X_R2, // 左右半径
  X_DZ,
  X_DIM
};
class RobotMotionFactor
    : public gtsam::NoiseModelFactorN<gtsam::Vector11, gtsam::Vector11> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Vector11, gtsam::Vector11>;

public:
  gtsam::Vector evaluateError(const gtsam::Vector11 &prev,
                              const gtsam::Vector11 &curr,
                              gtsam::OptionalMatrixType H1,
                              gtsam::OptionalMatrixType H2) const override;

private:
  double dt_sec_;
};

} // namespace auto_aim
