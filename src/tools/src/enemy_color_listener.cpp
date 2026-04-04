#include "hardware/enemy_color_listener.hpp"
#include "msgs/EnemyColor.hpp"
#include "types/EnemyColor.hpp"
#include "types/IceoryxServiceDescription.hpp"

#include "quill/LogMacros.h"

#include <atomic>

hardware::EnemyColorListener::EnemyColorListener(
    quill::Logger *logger, types::EnemyColor default_color, double time_out_sec,
    const confs::IceoryxServiceDescription &description)
    : logger_(logger),
      enemy_color_sub_(
          types::IceoryxServiceDescription{description}.description),
      enemy_color_(default_color), time_out_sec_(time_out_sec) {
  listener_
      .attachEvent(enemy_color_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onSampleReceivedCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger_, "unable to attach enemy_color_sub");
        std::exit(EXIT_FAILURE);
      });
}

void hardware::EnemyColorListener::onSampleReceivedCallback(
    iox::popo::Subscriber<msgs::EnemyColor, msgs::Header> *subscriber,
    EnemyColorListener *self) {
  while (subscriber->take().and_then(
      [subscriber, self](const iox::popo::Sample<const msgs::EnemyColor,
                                                 const msgs::Header> &sample) {
        auto color_update = static_cast<types::EnemyColor>(sample->color);
        if (color_update != self->enemy_color_.load())
          LOG_DEBUG(self->logger_, "Enemy color changed! Current color: {}.",
                    (color_update == types::EnemyColor::Red) ? "Red" : "Blue");
        self->enemy_color_.store(color_update);
      })) {
  }
}

types::EnemyColor hardware::EnemyColorListener::getEnemyColor() const {
  return enemy_color_.load();
}

types::EnemyColor hardware::EnemyColorListener::getSelfColor() const {
  return (getEnemyColor() == types::EnemyColor::Red) ? types::EnemyColor::Blue
                                                     : types::EnemyColor::Red;
}
