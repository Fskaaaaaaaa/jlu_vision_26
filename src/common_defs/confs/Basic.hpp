#pragma once

#include <array>
namespace confs {
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
struct RpyAngle {
  double roll;
  double pitch;
  double yaw;
};
template <int ROWS, int COLS> struct Matrixd {
  int rols{ROWS};
  int cols{COLS};
  std::array<double, ROWS * COLS> data;
};
} // namespace confs
