// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

#include "KinematicFunctions.hpp"

#include "iox/variant.hpp"
namespace msgs {

struct [[deprecated]] Target {
  int type;
  iox::variant<BaseKinematicFunc, RobotKinematicFunc, OutpostKinematicFunc,
               SmallBuffKinematicFunc, BigBuffKinematicFunc>
      parametric_func;
};
} // namespace msgs
