#pragma once

#include "confs/CameraInfo.hpp"
#include "confs/CameraParams.hpp"

#include <string>

namespace hardware {

enum class CameraType {
  hik,
  galaxy,
  usb,
};

struct CameraConfigs {
  confs::CameraInfo camera_info;
  confs::CameraParams camera_params;
  CameraType camera_type;
  std::string camera_name;
  int image_publish_interval_ms;
  int cam_info_publish_interval_ms;
};

} // namespace hardware
