// Copyright (c) 2026 Fsk. All Rights Reserved.
#pragma once
#include <fcntl.h>
#include <opencv2/core/hal/interface.h>

namespace msgs {

namespace impl {
constexpr int getElemSize(int type) {
  int depth = type & CV_MAT_DEPTH_MASK;
  int size = (depth == 0 || depth == 1)                 ? 1
             : (depth == 2 || depth == 3 || depth == 7) ? 2
             : (depth == 4 || depth == 5)               ? 4
             : (depth == 6)                             ? 8
                                                        : 0;
  int channels = 1 + (type >> CV_CN_SHIFT);
  return size * channels;
}
} // namespace impl

template <int WIDTH, int HEIGHT, int CV_TYPE> struct Image {
  static constexpr int cols{WIDTH};
  static constexpr int rols{HEIGHT};
  static constexpr int cv_type{CV_TYPE};
  static constexpr int data_size{impl::getElemSize(CV_TYPE) * WIDTH * HEIGHT};
  unsigned char data[impl::getElemSize(CV_TYPE) * WIDTH * HEIGHT];

  static_assert(WIDTH > 0 && HEIGHT > 0, "Width/Height must be positive!");
  static_assert(impl::getElemSize(CV_TYPE) > 0,
                "Invalid CV_TYPE (elemSize=0)!");
};

using Image1920x1080_8UC3 = Image<1920, 1080, CV_8UC3>;
using Image1440x1080_8UC3 = Image<1440, 1080, CV_8UC3>;
using Image800x600_8UC3 = Image<800, 600, CV_8UC3>;

} // namespace msgs
