#pragma once

namespace types {
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
struct RpyAngle {
  double roll;
  double pitch;
  double yaw;
};
} // namespace types
