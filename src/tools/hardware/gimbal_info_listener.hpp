#pragma once

#include "confs/IceoryxServiceDescription.hpp"
#include "msgs/GimbalInfo.hpp"
#include "msgs/Header.hpp"

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "quill/Logger.h"

#include <atomic>

namespace hardware {

// NOTE: 暂时不含有缓冲区，只是单纯返回最新的云台信息
class GimbalInfoListener {
public:
  GimbalInfoListener(quill::Logger *logger,
                     const confs::IceoryxServiceDescription &description =
                         confs::IceoryxServiceDescription{
                             .service = "gimbal_info",
                             .instance = "serial",
                             .event = "data"});
  msgs::GimbalInfo getLatestInfo() const;

private:
  quill::Logger *logger_;
  static void onSampleReceivedCallback(
      iox::popo::Subscriber<msgs::GimbalInfo, msgs::Header> *subscriber,
      GimbalInfoListener *self);

  iox::popo::Subscriber<msgs::GimbalInfo, msgs::Header> gimbal_info_sub_;
  iox::popo::Listener listener_;
  std::atomic<msgs::GimbalInfo> cache_;
};

} // namespace hardware
