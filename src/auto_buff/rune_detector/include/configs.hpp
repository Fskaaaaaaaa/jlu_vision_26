#pragma once

#include "confs/CameraParams.hpp"
// #include "confs/IceoryxServiceDescription.hpp"

#include "quill/core/LogLevel.h"

namespace auto_buff {
struct RuneDetectorConfigs {
  quill::LogLevel log_level;

  int num;
};

} // namespace auto_buff
