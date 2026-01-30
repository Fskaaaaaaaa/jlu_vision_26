// Copyright (c) 2026 aaa. All Rights Reserved.
#pragma once

#include "confs/IceoryxServiceDescription.hpp"

#include "quill/core/LogLevel.h"
#include "serial/serial.h"
#include "types/EnemyColor.hpp"
#include "types/TaskMode.hpp"

#include <cstdint>

namespace hardware {

struct SerialConfig {
  std::string device_name;
  std::uint32_t baudrate;
  serial::stopbits_t stopbits;
  serial::flowcontrol_t flowcontrol;
  serial::parity_t parity;
};
struct IecoryxConfig {
  confs::IceoryxServiceDescription aim_command_topic;
  confs::IceoryxServiceDescription gimbal_info_topic;
  confs::IceoryxServiceDescription enemy_color_topic;
  confs::IceoryxServiceDescription task_mode_topic;
};
struct SerialConfigs {
  SerialConfig serial_conf;
  IecoryxConfig iceoryx_conf;
  quill::LogLevel log_level;
};

} // namespace hardware
