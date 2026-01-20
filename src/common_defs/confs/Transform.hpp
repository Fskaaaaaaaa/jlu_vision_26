#pragma once
#include "Basic.hpp"

#include <string>

namespace confs {
struct Transform {
  std::string parent_frame_id;
  std::string child_frame_id;
  Vector3d tvec;
  RpyAngle rpy_angle;
  Vector4d getQuaterniond();
};
} // namespace confs
