#pragma once

#include "msgs/Basic.hpp"

namespace msgs {

struct [[deprecated]] BaseKinematicFunc {
  Vector3d center_xyz;
  RpyRadian center_rpy;
};
struct [[deprecated]] RobotKinematicFunc {
  Vector3d center_xyz;
  Vector3d linear_velocity;
  RpyRadian center_rpy;
  RpyRadian angular_velocity;
  double radius1;
  double radius2;
  double dz;
};
struct [[deprecated]] OutpostKinematicFunc {
  Vector3d center_xyz;
  Vector3d linear_velocity;
  RpyRadian center_rpy;
  RpyRadian angular_velocity;
  double radius;
  double dz1;
  double dz2;
};
struct [[deprecated]] SmallBuffKinematicFunc {
  Vector3d center_xyz;
  Vector3d linear_velocity;
  RpyRadian center_rpy;
  RpyRadian angular_velocity;
  double radius;
};
struct [[deprecated]] BigBuffKinematicFunc {
  Vector3d center_xyz;
  double initial_phase;
  // TODO
};

} // namespace msgs
