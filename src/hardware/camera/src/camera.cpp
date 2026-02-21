#include "camera.hpp"
#include "configs.hpp"
#include "galaxy.hpp"
// #include "hikrobot.hpp"

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
#include "quill/core/ThreadContextManager.h"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

hardware::Camera::Camera(quill::Logger *logger, const CameraConfigs &configs)
    : logger_(logger), configs_(configs),
      image_pub_({"image_raw",
                  {iox::TruncateToCapacity, configs_.camera_name.c_str()},
                  "data"},
                 {0}),
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
    // TODO
    break;
  case CameraType::usb:
    // TODO
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
  int status;
  this->image_pub_.loan()
      .and_then([&](iox::popo::Sample<msgs::Image1440x1080_8UC3, msgs::Header>
                        &sample) {
        sample.getUserHeader().frame_id = {iox::TruncateToCapacity,
                                           configs_.camera_frame_id.c_str()};
        sample.getUserHeader().stamp_ns = tools::getTimeNowNanoSec();
        status = this->camera_->captureImage(sample->data, sample->data_size);
        if (status == 0) {
          sample.publish();
        } else {
          LOG_WARNING(logger_, "something wrong on getting image!");
        }
      })
      .or_else([&](auto) {
        status = EXIT_FAILURE;
        LOG_WARNING(logger_, "unable loan sample to publish image!");
      });
  return status == EXIT_SUCCESS;
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
