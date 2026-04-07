#pragma once

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

struct Vector2d {
  double x;
  double y;
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
struct RpyRadian {
  double roll;
  double pitch;
  double yaw;
};

} // namespace msgs
