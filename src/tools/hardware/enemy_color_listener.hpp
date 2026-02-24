#pragma once

#include "confs/IceoryxServiceDescription.hpp"
#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "msgs/EnemyColor.hpp"
#include "msgs/Header.hpp"
#include "quill/Logger.h"
#include "types/EnemyColor.hpp"
#include <atomic>
#include <thread>

namespace hardware {

class EnemyColorListener {
public:
  EnemyColorListener(quill::Logger *logger, types::EnemyColor default_color,
                     double time_out_sec = 10,
                     const confs::IceoryxServiceDescription &description =
                         confs::IceoryxServiceDescription{
                             .service = "enemy_color",
                             .instance = "serial",
                             .event = "data"});
  types::EnemyColor getEnemyColor() const;
  types::EnemyColor getSelfColor() const;

private:
  static void onSampleReceivedCallback(
      iox::popo::Subscriber<msgs::EnemyColor, msgs::Header> *subscriber,
      EnemyColorListener *self);
  quill::Logger *logger_;
  iox::popo::Subscriber<msgs::EnemyColor, msgs::Header> enemy_color_sub_;
  std::atomic<types::EnemyColor> color_;
  std::atomic<bool> color_ok_;
  iox::popo::Listener listener_;
  double time_out_sec_;
};

} // namespace hardware
