// Copyright (c) 2026 Fsk. All Rights Reserved.
#pragma once
#include "msgs/Basic.hpp"

#include <iceoryx_hoofs/cxx/string.hpp>
#include <iceoryx_hoofs/cxx/vector.hpp>

namespace msgs {
struct Armor {
  int armor_type;
  int armor_color;
  double distance_to_image_center;
  Point3d position;
  Vector4d orientation;
  float confidence;
  bool key_frame;
  bool heart_beat;
};
} // namespace msgs
