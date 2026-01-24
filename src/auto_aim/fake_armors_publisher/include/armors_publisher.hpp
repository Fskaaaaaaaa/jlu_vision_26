// Copyright (c) 2026 Fuck. All Rights Reserved.
#pragma once

#include "configs.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "quill/Logger.h"
#include "types/Armor.hpp"
#include <atomic>
#include <mutex>
#include <thread>
namespace auto_aim {
class ArmorsPublisher {
public:
  ArmorsPublisher(quill::Logger *logger, const ArmorsPublisherConfig &config,
                  const std::atomic<types::Armor> &data_to_publish,
                  std::mutex &mtx);

private:
  quill::Logger *logger_;
  ArmorsPublisherConfig config_;
  iox::popo::Publisher<msgs::Armor, msgs::Header> armors_pub_;
  std::jthread armors_pub_thread_;
  std::atomic<msgs::Armor> &armor_data_;
};
}; // namespace auto_aim
