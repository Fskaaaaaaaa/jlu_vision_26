// Copyright (c) 2026 gugugaga. All Rights Reserved.
#pragma once

#include "configs.hpp"
#include "opencv2/core/types.hpp"
#include "types.hpp"
#include "types/CameraInfo.hpp"

#include "quill/Logger.h"
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace auto_aim {

class BASolver {
public:
  BASolver(quill::Logger *logger, const BaConfig &config,
           const types::CameraInfo &cam_info);

  bool optimizeArmorPose(Armor &armor) const;

private:
  quill::Logger *logger_;
  BaConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
};

class ProjectionFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3> {
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3>;

public:
  ProjectionFactor(const gtsam::SharedNoiseModel &model, gtsam::Key key,
                   const gtsam::Point3 &obj_point,
                   const gtsam::Point2 &measurement, const gtsam::Cal3DS2 &K);

  gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                              gtsam::OptionalMatrixType H) const override;

private:
  gtsam::Point3 point_;
  gtsam::Point2 measurement_;
  gtsam::Cal3DS2 K_;
  // NOTE: 从z向前的pnp/project的相机系到z向上的ros系的旋转矩阵
  Eigen::Matrix3d R_ros_to_camera_;
};

} // namespace auto_aim
