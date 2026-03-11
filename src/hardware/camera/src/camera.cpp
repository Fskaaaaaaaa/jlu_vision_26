#include "camera.hpp"
#include "basic/time_tools.hpp"
#include "configs.hpp"
#include "galaxy.hpp"
#include "hikrobot.hpp"
#include "video_capture.hpp"

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iox/signal_watcher.hpp"
#include "iox/string.hpp"
#include "msgs/CameraInfo.hpp"
#include "msgs/CameraParams.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"
#include "quill/LogMacros.h"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

hardware::Camera::Camera(quill::Logger *logger, const CameraConfigs &configs)
    : logger_(logger), configs_(configs),
      image_pub_({"image_raw",
                  {iox::TruncateToCapacity, configs_.camera_name.c_str()},
                  "data"},
                 {0U}),
      cam_info_pub_({"camera_info",
                     {iox::TruncateToCapacity, configs_.camera_name.c_str()},
                     "data"}),
      cam_params_change_sub_(
          {"change_camera_params",
           {iox::TruncateToCapacity, configs_.camera_name.c_str()},
           "request"}) {
  // init camera
  switch (configs_.camera_type) {
  case CameraType::galaxy:
    this->camera_ = std::make_unique<Galaxy>(logger, configs_.camera_params);
    break;
  case CameraType::hik:
    this->camera_ = std::make_unique<HikRobot>(logger, configs_.camera_params);
    break;
  case CameraType::video:
    this->camera_ = std::make_unique<VideoCapture>(logger, configs_.video_path);
    break;
  default:
    LOG_CRITICAL(logger_, "Unknown camera type!");
    std::exit(EXIT_FAILURE);
  }
  this->image_read_pub_thread_ = std::jthread{[&]() {
    LOG_INFO(logger_, "Image pub thread start!");
    while (!iox::hasTerminationRequested()) {
      this->publishImage();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(configs_.image_publish_interval_ms));
    }
    LOG_INFO(logger_, "Image pub thread stop!");
  }};
  this->cam_info_pub_thread_ = std::jthread{[&]() {
    LOG_INFO(logger_, "CameraInfo pub thread start!");
    while (!iox::hasTerminationRequested()) {
      this->publishCamInfo();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(configs_.cam_info_publish_interval_ms));
    }
    LOG_INFO(logger_, "CameraInfo pub thread stop!");
  }};
  cam_param_change_listener_
      .attachEvent(cam_params_change_sub_,
                   iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onCameraParamRecievedCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger, "unable to attach subscriber CameraParams");
        std::exit(EXIT_FAILURE);
      });
}

bool hardware::Camera::publishImage() {
  bool success;
  this->image_pub_.loan()
      .and_then([&](iox::popo::Sample<msgs::Image1440x1080_8UC3, msgs::Header>
                        &sample) {
        std::chrono::system_clock::time_point stamp;
        success =
            this->camera_->readImage(sample->data, sample->data_size, stamp);
        sample.getUserHeader().frame_id = {iox::TruncateToCapacity,
                                           configs_.camera_frame_id.c_str()};
        // NOTE: 这里改成用硬件时间戳，不知道会不会更精准
        // 另外硬件时间戳是systemclock还是steady_clock还不太确定
        sample.getUserHeader().stamp_ns = tools::chronoPointToNanoSec(stamp);
        if (success)
          sample.publish();
        else
          LOG_WARNING(logger_, "something wrong on getting image!");
      })
      .or_else([&](auto) {
        success = false;
        LOG_WARNING(logger_, "unable loan sample to publish image!");
      });
  return success;
}

bool hardware::Camera::publishCamInfo() {
  bool status{true};
  this->cam_info_pub_.loan()
      .and_then([&](iox::popo::Sample<msgs::CameraInfo, msgs::Header> &sample) {
        sample.getUserHeader().frame_id = {iox::TruncateToCapacity,
                                           configs_.camera_frame_id.c_str()};
        sample.getUserHeader().stamp_ns = tools::getTimeNowNanoSec();
        std::copy(configs_.camera_info.camera_matrix.begin(),
                  configs_.camera_info.camera_matrix.end(),
                  sample->camera_matrix.begin());
        std::copy(configs_.camera_info.distortion_coefficients.begin(),
                  configs_.camera_info.distortion_coefficients.end(),
                  sample->distortion_coefficients.begin());
        sample->view_height_px = configs_.camera_info.view_height_px;
        sample->view_width_px = configs_.camera_info.view_width_px;
        sample.publish();
        LOG_DEBUG(logger_, "camera info published.");
      })
      .or_else([&](auto) {
        status = false;
        LOG_WARNING(logger_, "unable loan sample to publish camera info!");
      });
  return status;
}

void hardware::Camera::onCameraParamRecievedCallback(
    iox::popo::Subscriber<msgs::CameraParams, msgs::Header> *subscriber,
    Camera *self) {
  while (subscriber->take().and_then(
      [&subscriber, &self](
          const iox::popo::Sample<const msgs::CameraParams, const msgs::Header>
              &sample) {
        auto instance_string =
            subscriber->getServiceDescription().getInstanceIDString();
        if (instance_string ==
            iox::capro::IdString_t{iox::TruncateToCapacity,
                                   self->configs_.camera_name.c_str()}) {
          self->camera_->changeExposureGain(sample->exposure_time,
                                            sample->gain);
        }
      })) {
  } // end of take samples
}
