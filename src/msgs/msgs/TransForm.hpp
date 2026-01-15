// Copyright (c) 2026 Feng. All Rights Reserved.
#include "Basic.hpp"
#include <Eigen/Dense>
#include <iceoryx_hoofs/cxx/string.hpp>
#include <iceoryx_hoofs/cxx/vector.hpp>

namespace msgs {
struct TransForm { // 必须配合Header一起使用
  iox::cxx::string<10> child_frame_id;
  Vector3d translate;
  Vector4d quaterniond;
  bool is_static;
};
} // namespace msgs
