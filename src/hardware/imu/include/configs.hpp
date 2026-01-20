#pragma once

#include "quill/core/LogLevel.h"

#include <string>
#include <termios.h>

namespace hardware {
enum class ImuType {
  xr,
  dm,
};
struct ImuConfig {
  std::string imu_serial_port = "/dev/ttyACM0";
  std::string imu_frame_id = "imu";
};
struct ImuConfigs {
  ImuType imu_type = ImuType::xr;
  std::string imu_name = "imu0";
  int imu_data_publish_interval_ms;
  ImuConfig imu_config;
  quill::LogLevel log_level;
};
} // namespace hardware
