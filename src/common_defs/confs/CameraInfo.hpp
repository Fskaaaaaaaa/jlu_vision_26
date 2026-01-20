#pragma once

#include <array>
namespace confs {
struct CameraInfo {
  int view_width_px;
  int view_height_px;
  std::array<double, 9> camera_matrix;
  std::array<double, 5> distortion_coefficients;
};
} // namespace confs
