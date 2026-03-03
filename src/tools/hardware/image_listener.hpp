#pragma once

#include "basic/time_tools.hpp"
#include "msgs/Header.hpp"

#include "iceoryx_posh/internal/popo/base_subscriber.hpp"
#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/notification_callback.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "opencv2/core.hpp"
#include "quill/LogMacros.h"
#include "quill/Logger.h"

#include <chrono>
#include <functional>

namespace hardware {
template <typename ImageT> class ImageListener {
public:
  ImageListener(
      quill::Logger *logger, const std::string &camera_name,
      const std::function<
          void(const cv::Mat &image, const std::string &frame_id,
               const std::chrono::system_clock::time_point &stamp)> &callback)
      : logger_(logger),
        image_sub_({"image_raw",
                    {iox::TruncateToCapacity, camera_name.c_str()},
                    "data"},
                   {1U}),
        user_callback_(callback) {
    image_listener_
        .attachEvent(image_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                     iox::popo::createNotificationCallback(
                         onImageReceivedCallback, *this))
        .or_else([this](auto) {
          LOG_CRITICAL(logger_, "unable to attach image_sub!");
          std::exit(EXIT_FAILURE);
        });
  }

private:
  static void onImageReceivedCallback(
      iox::popo::Subscriber<ImageT, msgs::Header> *subscriber,
      ImageListener<ImageT> *self) {
    subscriber->take()
        .and_then([subscriber, self](
                      const iox::popo::Sample<const ImageT, const msgs::Header>
                          &sample) {
          if (subscriber->getServiceDescription().getInstanceIDString() !=
              self->image_sub_.getServiceDescription().getInstanceIDString())
            return;
          auto stamp =
              tools::nanoSecToChronoPoint(sample.getUserHeader().stamp_ns);
          std::string frame_id{sample.getUserHeader().frame_id.c_str()};
          const cv::Mat image{sample->rows, sample->cols, sample->cv_type,
                              const_cast<unsigned char *>(sample->data)};
          self->user_callback_(image, frame_id, stamp);
        })
        .or_else([self](auto) {
          LOG_DEBUG(self->logger_,
                    "[image_listener]: error on receiving image.");
        });
  }

  quill::Logger *logger_;
  iox::popo::Subscriber<ImageT, msgs::Header> image_sub_;
  iox::popo::Listener image_listener_;
  std::function<void(const cv::Mat &image, const std::string &frame_id,
                     const std::chrono::system_clock::time_point &stamp)>
      user_callback_;
};

} // namespace hardware
