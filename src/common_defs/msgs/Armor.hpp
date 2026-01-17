// Copyright (c) 2026 Fsk. All Rights Reserved.
#pragma once
#include "msgs/Basic.hpp"
#include <iceoryx_hoofs/cxx/string.hpp>
#include <iceoryx_hoofs/cxx/vector.hpp>

namespace msgs {
struct Armor {
  int armor_type;
  double distance_to_image_center;
  Vector3d tvec;
  Vector4d quaternion;
};
} // namespace msgs
