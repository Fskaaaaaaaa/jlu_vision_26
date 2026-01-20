#pragma once
#include "basic/time_tools.hpp"
#include "configs.hpp"
#include "confs/CameraParams.hpp"
#include "iceoryx_posh/popo/listener.hpp"
#include "msgs/CameraInfo.hpp"
#include "msgs/CameraParams.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"

#include <iceoryx_posh/popo/publisher.hpp>
#include <iceoryx_posh/popo/sample.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <opencv2/opencv.hpp>
#include <quill/Logger.h>

#include <cstddef>
#include <memory>
#include <thread>

namespace hardware {

class CameraBase {
public:
  virtual ~CameraBase() = default;
  virtual int read(unsigned char *buffer, std::size_t buffer_size) = 0;
  // NOTE: 从得到的工业相机sdk定义的图像类型写入缓冲区（ioxsample）
  virtual int changeGain(double new_gain) = 0;
  virtual int changeExposure(int new_exp) = 0;
};

class Camera {
public:
  Camera(quill::Logger *logger, const CameraConfigs &configs);

private:
  int publishImage();
  bool publishCamInfo();
  static void onCameraParamRecievedCallback(
      iox::popo::Subscriber<msgs::CameraInfo, msgs::Header> *subscriber,
      Camera *self);

  quill::Logger *logger_;
  CameraConfigs configs_;
  std::unique_ptr<CameraBase> camera_;
  iox::popo::Publisher<msgs::Image1440x1080_8UC3, msgs::Header> image_pub_;
  std::jthread image_read_pub_thread_;
  iox::popo::Publisher<msgs::CameraInfo, msgs::Header> cam_info_pub_;
  std::jthread cam_info_pub_thread_;
  iox::popo::Subscriber<msgs::CameraParams, msgs::Header> cam_param_sub_;
  iox::popo::Listener cam_param_change_listener_;
};

} // namespace hardware
