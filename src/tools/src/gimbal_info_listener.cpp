#include "hardware/gimbal_info_listener.hpp"
#include "msgs/GimbalInfo.hpp"
#include "msgs/Header.hpp"
#include "types/IceoryxServiceDescription.hpp"

#include "iceoryx_posh/internal/popo/base_subscriber.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "quill/LogMacros.h"

#include <cstdlib>

hardware::GimbalInfoListener::GimbalInfoListener(
    quill::Logger *logger, const confs::IceoryxServiceDescription &description)
    : logger_(logger),
      gimbal_info_sub_(
          types::IceoryxServiceDescription{description}.description),
      cache_(msgs::GimbalInfo{0, 0, 0, 0, 0, 22}) {
  listener_
      .attachEvent(gimbal_info_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onSampleReceivedCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger_, "unable to attach gimbal_info_sub");
        std::exit(EXIT_FAILURE);
      });
}

void hardware::GimbalInfoListener::onSampleReceivedCallback(
    iox::popo::Subscriber<msgs::GimbalInfo, msgs::Header> *subscriber,
    GimbalInfoListener *self) {
  while (subscriber->take().and_then(
      [subscriber, self](const iox::popo::Sample<const msgs::GimbalInfo,
                                                 const msgs::Header> &sample) {
        self->cache_.store(msgs::GimbalInfo{
            .roll = sample->roll,
            .pitch = sample->pitch,
            .yaw = sample->yaw,
            .pitch_vel = sample->pitch_vel,
            .yaw_vel = sample->yaw_vel,
            .bullet_speed = sample->bullet_speed,
        });
      })) {
  }
}

msgs::GimbalInfo hardware::GimbalInfoListener::getLatestInfo() const {
  return cache_.load();
}
