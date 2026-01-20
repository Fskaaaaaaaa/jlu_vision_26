#pragma once

#include "confs/CameraInfo.hpp"
#include "confs/CameraParams.hpp"

#include "quill/core/LogLevel.h"

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
  std::string camera_frame_id;
  int image_publish_interval_ms;
  int cam_info_publish_interval_ms;
  quill::LogLevel log_level;
};

} // namespace hardware
