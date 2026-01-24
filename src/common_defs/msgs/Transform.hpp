// Copyright (c) 2026 Feng. All Rights Reserved.
#pragma once

#include "Basic.hpp"

#include <iceoryx_hoofs/cxx/string.hpp>
#include <iceoryx_hoofs/cxx/vector.hpp>

namespace msgs {
struct Transform {
  iox::string<10> child_frame_id;
  Vector3d translate;
  Vector4d quaterniond;
};

} // namespace msgs
