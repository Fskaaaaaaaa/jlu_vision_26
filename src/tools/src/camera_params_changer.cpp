// Copyright (c) 2026 CCB. All Rights Reserved.
#include "hardware/camera_params_changer.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iox/string.hpp"
#include "msgs/CameraParams.hpp"
#include "msgs/Header.hpp"
#include "quill/LogMacros.h"

hardware::CameraParamsChanger::CameraParamsChanger(
    quill::Logger *logger, const std::string &camera_name)
    : logger_(logger),
      camera_params_pub_({"change_camera_params",
                          {iox::TruncateToCapacity, camera_name.c_str()},
                          "request"}) {}
void hardware::CameraParamsChanger::changeCameraParams(
    const confs::CameraParams &params) {
  this->camera_params_pub_.loan()
      .and_then(
          [&](iox::popo::Sample<msgs::CameraParams, msgs::Header> &sample) {
            // NOTE: 实际上Header并没有用。但为了保持一致性还是加上吧
            sample->exposure_time = params.exposure_time;
            sample->gain = params.gain;
            sample.publish();
            LOG_DEBUG(logger_, "[CameraParamsChanger]: success publish camera "
                               "params change request.");
          })
      .or_else([&](auto) {
        LOG_WARNING(logger_, "[CameraParamsChanger]: fail to publish camera "
                             "params change request!");
      });
}
