#pragma once

#include "basic/time_tools.hpp"
#include "msgs/Header.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iox/signal_watcher.hpp"
#include "opencv2/core.hpp"
#include "quill/LogMacros.h"
#include "quill/Logger.h"

#include <chrono>
#include <thread>
// #include <type_traits>

namespace hardware {

// template <typename F>
// concept ImageCallback =
//     requires(F &&f, const cv::Mat &image, const std::string &frame_id,
//              const std::chrono::system_clock::time_point &stamp) {
//       { f(image, frame_id, stamp) } -> std::same_as<void>;
//     };

template <typename ImageT> class ImagePoller {
public:
  ImagePoller(
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
    this->polling_thread_ = std::jthread{[&]() {
      LOG_INFO(logger_, "Image polling thread start!");
      while (!iox::hasTerminationRequested()) {
        image_sub_.take()
            .and_then([&](const iox::popo::Sample<const ImageT,
                                                  const msgs::Header> &sample) {
              auto stamp =
                  tools::nanoSecToChronoPoint(sample.getUserHeader().stamp_ns);
              std::string frame_id{sample.getUserHeader().frame_id.c_str()};
              const cv::Mat image{sample->rows, sample->cols, sample->cv_type,
                                  const_cast<unsigned char *>(sample->data)};
              user_callback_(image, frame_id, stamp);
            })
            .or_else([](auto) {
              std::this_thread::sleep_for(std::chrono::milliseconds{1});
            });
      }
      LOG_INFO(logger_, "Image polling thread stop!");
    }};
  }

private:
  quill::Logger *logger_;
  iox::popo::Subscriber<ImageT, msgs::Header> image_sub_;
  std::function<void(const cv::Mat &image, const std::string &frame_id,
                     const std::chrono::system_clock::time_point &stamp)>
      user_callback_;
  std::jthread polling_thread_;
};

// template <typename ImageT, typename CallbackT>
// std::unique_ptr<ImagePoller<ImageT, std::decay_t<CallbackT>>>
// makeImagePollerUnique(quill::Logger *logger, const std::string &camera_name,
//                       CallbackT &&callback) {
//   return std::make_unique<ImagePoller<ImageT, std::decay_t<CallbackT>>>(
//       logger, camera_name, std::forward<CallbackT>(callback));
// }

} // namespace hardware
