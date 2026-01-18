#pragma once

#include "msgs/Basic.hpp"

namespace msgs {
struct KinematicFuncRobot {
  Vector3d center_xyz;
  Vector3d center_rpy;
  Vector3d center_vxyz;
  Vector3d center_vrpy;
  double radius1;
  double radius2;
  double dz;
};
} // namespace msgs
