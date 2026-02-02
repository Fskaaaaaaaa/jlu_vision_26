#pragma once

#include "confs/IceoryxServiceDescription.hpp"
#include "msgs/Header.hpp"
#include "msgs/TaskMode.hpp"
#include "types/TaskMode.hpp"

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "quill/Logger.h"

#include <atomic>
namespace hardware {
class TaskModeListener {
public:
  TaskModeListener(quill::Logger *logger, types::TaskMode target_mode,
                   const confs::IceoryxServiceDescription &task_topic = {
                       "task_mode", "serial", "data"});
  bool isOnTask();
  bool isTask(types::TaskMode task);

private:
  static void onTaskModeReceiveCallback(
      iox::popo::Subscriber<msgs::TaskMode, msgs::Header> *subscriber,
      TaskModeListener *self);
  quill::Logger *logger_;
  iox::popo::Subscriber<msgs::TaskMode, msgs::Header> task_sub_;
  iox::popo::Listener task_listener_;
  const types::TaskMode target_mode_;
  std::atomic<types::TaskMode> current_mode_;
};
} // namespace hardware
