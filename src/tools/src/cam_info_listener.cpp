// Copyright (c) 2026 F. All Rights Reserved.
#include "hardware/cam_info_listener.hpp"

#include "iceoryx_posh/internal/popo/base_subscriber.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iox/string.hpp"
#include "msgs/CameraInfo.hpp"
#include "msgs/Header.hpp"
#include "quill/LogMacros.h"
#include "quill/core/ThreadContextManager.h"

#include <atomic>
#include <cstdlib>
#include <string>

hardware::CameraInfoListener::CameraInfoListener(quill::Logger *logger,
                                                 const std::string &camera_name)
    : logger_(logger),
      camera_instance_id_({iox::TruncateToCapacity, camera_name.c_str()}),
      cam_info_sub_({"camera_info", camera_instance_id_, "data"}),
      cam_info_ok_(false) {
  LOG_DEBUG(logger_, "Constructor: this = {}", static_cast<void *>(this));
  this->cam_info_listener_
      .attachEvent(cam_info_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onCamInfoReceivedCallback, *this))
      .or_else([&](auto) {
        LOG_ERROR(logger_, "unable attach event subscribe cam info!");
        std::exit(EXIT_FAILURE);
      });
  LOG_DEBUG(logger_, "camera info listener success init.");
}

hardware::CameraInfoListener::~CameraInfoListener() {
  LOG_DEBUG(logger_, "Destructor: this = {}", static_cast<void *>(this));
  cam_info_listener_.detachEvent(cam_info_sub_,
                                 iox::popo::SubscriberEvent::DATA_RECEIVED);
}

void hardware::CameraInfoListener::onCamInfoReceivedCallback(
    iox::popo::Subscriber<msgs::CameraInfo, msgs::Header> *subscriber,
    CameraInfoListener *self) {
  LOG_DEBUG(self->logger_, "Callback: self = {}", static_cast<void *>(self));
  subscriber->take().and_then(
      [subscriber, self](const iox::popo::Sample<const msgs::CameraInfo,
                                                 const msgs::Header> &sample) {
        if (subscriber->getServiceDescription().getInstanceIDString() ==
            self->camera_instance_id_) {
          self->cam_info_ = {sample};
          self->cam_info_sub_.unsubscribe();
          LOG_DEBUG(self->logger_,
                    "success recieve info from camera {}, unsubscribed!",
                    self->camera_instance_id_.c_str());
          self->cam_info_ok_.store(true, std::memory_order_release);
          self->cam_info_ok_.notify_one();
        }
      });
}

types::CameraInfo hardware::CameraInfoListener::get() {
  LOG_INFO(this->logger_, "start waiting for camera info.");
  LOG_DEBUG(logger_, "get() called, this = {}", static_cast<void *>(this));
  this->cam_info_ok_.wait(false, std::memory_order_acquire);
  LOG_DEBUG(logger_, "get() returning, this = {}", static_cast<void *>(this));
  return this->cam_info_;
}
