// Copyright (c) 2026 F. All Rights Reserved.
#pragma once
#include "configs.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "msgs/TaskMode.hpp"
#include "quill/Logger.h"
#include "serial/serial.h"
#include "types/TaskMode.hpp"
#include <atomic>
#include <thread>

namespace hardware {
class Serial {
public:
  Serial(quill::Logger *logger, const SerialConfigs &configs);

private:
  quill::Logger *logger_;
  SerialConfigs configs_;
  serial::Serial serial_;
  iox::popo::Publisher<msgs::TaskMode> task_mode_publisher_;
  iox::popo::Publisher<msgs::TaskMode> bullet_speed_publisher_;
};
} // namespace hardware
