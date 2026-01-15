// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
namespace msgs {
struct Point2d {
  double x;
  double y;
};
struct Point3d {
  double x;
  double y;
  double z;
};
struct Vector3d {
  double x;
  double y;
  double z;
};
struct Vector4d {
  double x;
  double y;
  double z;
  double w;
};

} // namespace msgs
