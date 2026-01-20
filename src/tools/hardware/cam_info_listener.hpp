// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

#include "iceoryx_posh/iceoryx_posh_types.hpp"
#include "msgs/CameraInfo.hpp"
#include "msgs/Header.hpp"
#include "types/CameraInfo.hpp"

#include <iceoryx_posh/popo/listener.hpp>
#include <iceoryx_posh/popo/sample.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <quill/Logger.h>

#include <atomic>

namespace hw {
class CameraInfoListener {
public:
  CameraInfoListener(quill::Logger *logger,
                     const std::string &camera_name = "camera0");
  types::CameraInfo get();

private:
  static void onCamInfoReceivedCallback(
      iox::popo::Subscriber<msgs::CameraInfo, msgs::Header> *subscriber,
      CameraInfoListener *self);
  quill::Logger *logger_;
  iox::capro::IdString_t camera_instance_id_;
  iox::popo::Subscriber<msgs::CameraInfo, msgs::Header> cam_info_sub_;
  iox::popo::Listener cam_info_listener_;
  std::atomic<bool> cam_info_ok_;
  types::CameraInfo cam_info_;
};
} // namespace hw
