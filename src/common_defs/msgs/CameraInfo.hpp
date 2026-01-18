// Copyright (c) 2026 Fsk. All Rights Reserved.
#pragma once
#include <iceoryx_hoofs/cxx/vector.hpp>

namespace msgs {
struct CameraInfo {
  int view_width_px;
  int view_height_px;
  iox::cxx::vector<double, 9> camera_matrix;
  iox::cxx::vector<double, 5> distortion_coefficients;
};
} // namespace msgs
