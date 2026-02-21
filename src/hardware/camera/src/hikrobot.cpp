#include "hikrobot.hpp"
#include "confs/CameraParams.hpp"

#include "MvCameraControl.h"
#include "quill/LogMacros.h"
#include <cstdlib>

// XXX: 一坨狗屎，一点异常处理都没有
hardware::HikRobot::HikRobot(quill::Logger *logger,
                             const confs::CameraParams &camera_params)
    : logger_(logger), camera_params_(camera_params) {
  LOG_INFO(logger_, "start hik camera.");
  // 查找并打开设备
  MV_CC_DEVICE_INFO_LIST device_list;
  MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  LOG_INFO(logger_, "Found camera count = {}", device_list.nDeviceNum);
  int fail_count = 0;
  while (device_list.nDeviceNum == 0) {
    fail_count++;
    LOG_ERROR(logger_, "No camera found!");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (fail_count > 5)
      break;
    // XXX: 可能导致ub
  }
  MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  MV_CC_OpenDevice(handle_);

  MV_IMAGE_BASIC_INFO img_info;
  MV_CC_GetImageInfo(handle_, &img_info);
  payload_size_ = img_info.nHeightMax * img_info.nWidthMax * 3;

  // 设置相机参数和帧率
  setEnumValue("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  setEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  setEnumValue("GainAuto", MV_GAIN_MODE_OFF);
  setFloatValue("ExposureTime", camera_params_.exposure_time);
  setFloatValue("Gain", camera_params_.gain);
  MV_CC_SetFrameRate(handle_, camera_params_.frame_rate);

  // 开始采集
  MV_CC_StartGrabbing(handle_);
}

void hardware::HikRobot::setFloatValue(const std::string &name, double value) {
  unsigned int ret;
  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);
  if (ret != MV_OK)
    LOG_WARNING(logger_, "MV_CC_SetFloatValue({}, {}) failed: {}", name, value,
                ret);
}

void hardware::HikRobot::setEnumValue(const std::string &name,
                                      unsigned int value) {
  unsigned int ret;
  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);
  if (ret != MV_OK)
    LOG_WARNING(logger_, "MV_CC_SetEnumValue({}, {}) failed: {}", name, value,
                ret);
}

int hardware::HikRobot::captureImage(unsigned char *buffer,
                                     std::size_t buffer_size) {
  if (buffer_size < payload_size_) {
    LOG_ERROR(logger_, "Insufficient buffer size! require {}, actual {}",
              payload_size_, buffer_size);
    return EXIT_FAILURE;
  }
  if (error_count_ > 5) {
    LOG_DEBUG(logger_, "[hikrobot]: error_count: {}, restart grabbing!",
              error_count_);
    MV_CC_StopGrabbing(handle_);
    MV_CC_StartGrabbing(handle_);
  }
  MV_FRAME_OUT raw; // NOTE: 还得是海康啊，不用手动管理buffer真是太爽了
  unsigned int nMsec = 100;
  auto ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
  if (ret != MV_OK) {
    LOG_ERROR(logger_, "MV_CC_GetImageBuffer failed: {}", ret);
    error_count_++;
    return EXIT_FAILURE;
  }
  const auto &frame_info = raw.stFrameInfo;
  auto pixel_type = frame_info.enPixelType;
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes>
      type_map = {{PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
                  {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
                  {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
                  {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};

  cv::cvtColor(cv::Mat{cv::Size(frame_info.nWidth, frame_info.nHeight), CV_8U,
                       raw.pBufAddr},
               cv::Mat{frame_info.nHeight, frame_info.nWidth, CV_8UC3, buffer},
               type_map.at(pixel_type));
  error_count_ = 0;
  return EXIT_SUCCESS;
}

int hardware::HikRobot::changeExposureGain(double exposure, double gain) {
  LOG_INFO(logger_, "Try to change exposure time to {}.", exposure);
  setFloatValue("ExposureTime", exposure);
  LOG_INFO(logger_, "Try to change gain to {}.", gain);
  setFloatValue("Gain", gain);
  return EXIT_SUCCESS;
}
