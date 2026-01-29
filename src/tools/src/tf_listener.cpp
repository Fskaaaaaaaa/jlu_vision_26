// Copyright (c) 2026 F. All Rights Reserved.
#include "transform/tf_listener.hpp"
#include "msgs/Header.hpp"
#include "types/Transform.hpp"

#include "quill/LogMacros.h"
#include "quill/Logger.h"

#include <cstdlib>
#include <stdexcept>
#include <variant>

tf::TransformListener::TransformListener(
    quill::Logger *logger, fast_tf::detail::transform_buffer &buffer)
    : buffer_(buffer), logger_(logger),
      dynamic_tf_subscriber_({"tf", "dynamic", "data"}),
      static_tf_subscriber_({"tf", "static", "data"}) {}

bool tf::TransformListener::init() {
  bool success{true};
  this->tf_listener_
      .attachEvent(this->static_tf_subscriber_,
                   iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onTransFormReceivedCallback, *this))
      .or_else([&success, this](auto) {
        success = false;
        LOG_ERROR(this->logger_, "unable to attach tf static");
      });
  this->tf_listener_
      .attachEvent(this->dynamic_tf_subscriber_,
                   iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onTransFormReceivedCallback, *this))
      .or_else([&success, this](auto) {
        success = false;
        LOG_ERROR(this->logger_, "unable to attach tf dynamic");
      });
  return success;
}

void tf::TransformListener::onTransFormReceivedCallback(
    iox::popo::Subscriber<msgs::Transform, msgs::Header> *subscriber,
    TransformListener *self) {
  while (subscriber->take().and_then(
      [subscriber, self](const iox::popo::Sample<const msgs::Transform,
                                                 const msgs::Header> &sample) {
        auto instance_string =
            subscriber->getServiceDescription().getInstanceIDString();
        types::Transform tf{sample};
        // store the sample in the corresponding cache
        try {
          if (instance_string == iox::capro::IdString_t("static")) {
            self->buffer_.set(tf.parent_frame_id, tf.child_frame_id, tf.stamp,
                              tf.getIsometry3d(), true);
            LOG_DEBUG(self->logger_, "recieve static tf msg from {} to {}",
                      tf.parent_frame_id, tf.child_frame_id);
          } else if (instance_string == iox::capro::IdString_t("dynamic")) {
            self->buffer_.set(tf.parent_frame_id, tf.child_frame_id, tf.stamp,
                              tf.getIsometry3d(), false);
            LOG_DEBUG(self->logger_, "recieve dynamic tf msg from {} to {}",
                      tf.parent_frame_id, tf.child_frame_id);
          }
        } catch (const std::runtime_error &e) {
          LOG_CRITICAL(self->logger_, "runtime_error: {}", e.what());
          std::exit(EXIT_FAILURE);
        } catch (const std::bad_variant_access &e) {
          LOG_CRITICAL(self->logger_, "bad_variant_access: {}", e.what());
          std::exit(EXIT_FAILURE);
        }
      })) {
  } // end of while
  return;
}
