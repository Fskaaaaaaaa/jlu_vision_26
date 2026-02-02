// Copyright (c) 2026 Char. All Rights Reserved.
#include "hardware/task_mode_listener.hpp"
#include "msgs/Header.hpp"
#include "msgs/TaskMode.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include "types/TaskMode.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include "quill/LogMacros.h"
#include "rfl/enums.hpp"

#include <cstdlib>

hardware::TaskModeListener::TaskModeListener(
    quill::Logger *logger, types::TaskMode target_mode,
    const confs::IceoryxServiceDescription &task_topic)
    : logger_(logger), target_mode_(target_mode),
      task_sub_(types::IceoryxServiceDescription{task_topic}.description) {
  this->task_listener_
      .attachEvent(task_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onTaskModeReceiveCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger_, "unable to attach task mode!");
        std::exit(EXIT_FAILURE);
      });
  LOG_DEBUG(logger_, "task mode listener launched.");
}

void hardware::TaskModeListener::onTaskModeReceiveCallback(
    iox::popo::Subscriber<msgs::TaskMode, msgs::Header> *subscriber,
    TaskModeListener *self) {
  while (subscriber->take().and_then(
      [subscriber, self](const iox::popo::Sample<const msgs::TaskMode,
                                                 const msgs::Header> &sample) {
        auto instance_string =
            subscriber->getServiceDescription().getInstanceIDString();
        if (instance_string !=
            self->task_sub_.getServiceDescription().getInstanceIDString())
          return;
        auto mode = static_cast<types::TaskMode>(sample->mode);
        if (mode != self->current_mode_.load()) {
          self->current_mode_.store(mode);
          LOG_TRACE_L1(self->logger_, "task mode change, current mode: {}",
                       rfl::enum_to_string(self->current_mode_.load()));
        }
      })) {
  } // end of while
}

bool hardware::TaskModeListener::isOnTask() {
  return current_mode_.load() == target_mode_;
}

bool hardware::TaskModeListener::isTask(types::TaskMode task) {
  return current_mode_.load() == task;
}
