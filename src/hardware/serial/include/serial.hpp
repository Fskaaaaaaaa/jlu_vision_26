// Copyright (c) 2026 F. All Rights Reserved.
#pragma once
#include "configs.hpp"
#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "msgs/AimCommand.hpp"
#include "msgs/BulletId.hpp"
#include "msgs/EnemyColor.hpp"
#include "msgs/GimbalInfo.hpp"
#include "msgs/Header.hpp"
#include "msgs/TaskMode.hpp"

#include "iceoryx_posh/popo/publisher.hpp"
#include "quill/Logger.h"
#include "serial/serial.h"
#include <atomic>
#include <thread>

namespace hardware {

class Serial {
public:
  Serial(quill::Logger *logger, const SerialConfigs &configs);
  ~Serial();

private:
  void reopenPort();
  static void onAimCommandReceivedCallback(
      iox::popo::Subscriber<msgs::AimCommand, msgs::Header> *subscriber,
      Serial *self);
  void receiveThread();

  quill::Logger *logger_;
  SerialConfigs configs_;
  serial::Serial serial_;
  std::jthread receive_thread_;
  iox::popo::Publisher<msgs::TaskMode, msgs::Header> task_mode_pub_;
  iox::popo::Publisher<msgs::GimbalInfo, msgs::Header> gimbal_info_pub_;
  iox::popo::Publisher<msgs::EnemyColor, msgs::Header> enemy_color_pub_;
  iox::popo::Publisher<msgs::BulletId, msgs::Header> bullet_id_pub_;
  iox::popo::Subscriber<msgs::AimCommand, msgs::Header> aim_cmd_sub_;
  iox::popo::Listener aim_cmd_listener_;
};

} // namespace hardware
