// Copyright (c) 2026 Fsk. All Rights Reserved.
#pragma once
#include <iceoryx_hoofs/cxx/vector.hpp>

namespace msgs {
struct CameraInfo {
  iox::cxx::vector<double, 9> camera_matrix;
  iox::cxx::vector<double, 5> distortion_coefficients;
};
} // namespace msgs
