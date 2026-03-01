#include "hardware/enemy_color_listener.hpp"
#include "msgs/EnemyColor.hpp"
#include "rfl/enums.hpp"
#include "types/EnemyColor.hpp"
#include "types/IceoryxServiceDescription.hpp"

#include "quill/LogMacros.h"
#include <atomic>
#include <chrono>
#include <thread>

hardware::EnemyColorListener::EnemyColorListener(
    quill::Logger *logger, types::EnemyColor default_color, double time_out_sec,
    const confs::IceoryxServiceDescription &description)
    : logger_(logger),
      enemy_color_sub_(
          types::IceoryxServiceDescription{description}.description),
      color_(default_color), color_ok_(false), time_out_sec_(time_out_sec) {
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
        self->color_.store(static_cast<types::EnemyColor>(sample->color));
        self->color_ok_.store(true);
      })) {
  }
}

types::EnemyColor hardware::EnemyColorListener::getEnemyColor() const {
  auto start = std::chrono::system_clock::now();
  auto timeout = [start, this]() {
    auto now = std::chrono::system_clock::now();
    double dt =
        std::chrono::duration_cast<std::chrono::duration<double>>(now - start)
            .count();
    return dt >= time_out_sec_;
  };
  do {
    if (timeout()) {
      LOG_WARNING(logger_,
                  "Waiting enemy_color time out! Use default_color {}.",
                  rfl::enum_to_string(color_.load()));
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
  } while (!color_ok_.load());
  return color_.load();
}

types::EnemyColor hardware::EnemyColorListener::getSelfColor() const {
  return (getEnemyColor() == types::EnemyColor::Red) ? types::EnemyColor::Blue
                                                     : types::EnemyColor::Red;
}
