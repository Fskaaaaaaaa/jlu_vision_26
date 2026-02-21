// Copyright (c) 2026 aaa. All Rights Reserved.
#pragma once

#include "confs/CameraParams.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "msgs/CameraParams.hpp"
#include "msgs/Header.hpp"
#include "quill/Logger.h"

#include <string>

namespace hardware {
class CameraParamsChanger {
public:
  CameraParamsChanger(quill::Logger *logger,
                      const std::string &camera_name = "camera0");
  void changeCameraParams(const confs::CameraParams &params);

private:
  quill::Logger *logger_;
  iox::popo::Publisher<msgs::CameraParams, msgs::Header> camera_params_pub_;
};
} // namespace hardware
