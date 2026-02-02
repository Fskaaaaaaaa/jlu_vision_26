// Copyright (c) 2026 F. All Rights Reserved.
#pragma once
#include "Basic.hpp"

namespace msgs {

struct ImuData {
  Vector4d orientation;
  Vector3d angular_velocity;
  Vector3d linear_acceleration;
};

} // namespace msgs
