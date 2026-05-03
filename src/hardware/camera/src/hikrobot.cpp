#include "hikrobot.hpp"
#include "confs/CameraParams.hpp"

#include "MvCameraControl.h"
#include "quill/LogMacros.h"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <thread>
#include <unordered_map>

// XXX: 一坨狗屎，一点异常处理都没有
hardware::HikRobot::HikRobot(quill::Logger *logger,
                             const confs::CameraParams &camera_params,
                             bool reverse_xy)
    : logger_(logger), camera_params_(camera_params), handle_(nullptr),
      buffer_inited_(false), error_count_(0), payload_size_(0) {
  LOG_INFO(logger_, "start hik camera.");
  // 查找并打开设备
  MV_CC_DEVICE_INFO_LIST device_list{};
  int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK) {
    LOG_CRITICAL(logger_, "MV_CC_EnumDevices failed: {} (0x{:08X})", ret,
                 static_cast<std::uint32_t>(ret));
    std::exit(EXIT_FAILURE);
  }
  LOG_INFO(logger_, "Found camera count = {}", device_list.nDeviceNum);
  int fail_count = 0;
  while (device_list.nDeviceNum == 0) {
    fail_count++;
    LOG_ERROR(logger_, "No camera found!");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
      LOG_WARNING(logger_,
                  "MV_CC_EnumDevices retry failed: {} (0x{:08X}), retrying...",
                  ret, static_cast<std::uint32_t>(ret));
    }
    if (fail_count > 5)
      break;
    // XXX: 可能导致ub
  }
  if (device_list.nDeviceNum == 0) {
    LOG_CRITICAL(logger_, "No camera found after retries.");
    std::exit(EXIT_FAILURE);
  }
  ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
  if (ret != MV_OK || handle_ == nullptr) {
    LOG_CRITICAL(logger_, "MV_CC_CreateHandle failed: {} (0x{:08X})", ret,
                 static_cast<std::uint32_t>(ret));
    std::exit(EXIT_FAILURE);
  }
  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    LOG_CRITICAL(logger_, "MV_CC_OpenDevice failed: {} (0x{:08X})", ret,
                 static_cast<std::uint32_t>(ret));
    std::exit(EXIT_FAILURE);
  }

  MV_IMAGE_BASIC_INFO img_info{};
  ret = MV_CC_GetImageInfo(handle_, &img_info);
  if (ret != MV_OK) {
    LOG_CRITICAL(logger_, "MV_CC_GetImageInfo failed: {} (0x{:08X})", ret,
                 static_cast<std::uint32_t>(ret));
    std::exit(EXIT_FAILURE);
  }
  payload_size_ = static_cast<std::size_t>(img_info.nHeightValue) *
                  static_cast<std::size_t>(img_info.nWidthValue) * 3;
  if (payload_size_ == 0) {
    payload_size_ = static_cast<std::size_t>(img_info.nHeightMax) *
                    static_cast<std::size_t>(img_info.nWidthMax) * 3;
  }

  // 设置相机参数和帧率
  setEnumValue("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
  setEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  setEnumValue("GainAuto", MV_GAIN_MODE_OFF);
  setFloatValue("ExposureTime", camera_params_.exposure_time);
  setFloatValue("Gain", camera_params_.gain);
  if (reverse_xy) {
    setBoolValue("ReverseX", true);
    setBoolValue("ReverseY", true);
  }
  ret = MV_CC_SetFrameRate(handle_, camera_params_.frame_rate);
  if (ret != MV_OK) {
    LOG_WARNING(logger_, "MV_CC_SetFrameRate({}) failed: {} (0x{:08X})",
                camera_params_.frame_rate, ret,
                static_cast<std::uint32_t>(ret));
  }
  ret = MV_CC_SetImageNodeNum(handle_, 3);
  if (ret != MV_OK) {
    LOG_WARNING(logger_, "MV_CC_SetImageNodeNum(3) failed: {} (0x{:08X})", ret,
                static_cast<std::uint32_t>(ret));
  }

  // 开始采集
  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    LOG_CRITICAL(logger_, "MV_CC_StartGrabbing failed: {} (0x{:08X})", ret,
                 static_cast<std::uint32_t>(ret));
    std::exit(EXIT_FAILURE);
  }
}

hardware::HikRobot::~HikRobot() {
  if (handle_ == nullptr) {
    return;
  }
  int ret = MV_CC_StopGrabbing(handle_);
  if (ret != MV_OK) {
    LOG_WARNING(logger_, "MV_CC_StopGrabbing failed in dtor: {} (0x{:08X})",
                ret, static_cast<std::uint32_t>(ret));
  }
  ret = MV_CC_CloseDevice(handle_);
  if (ret != MV_OK) {
    LOG_WARNING(logger_, "MV_CC_CloseDevice failed in dtor: {} (0x{:08X})", ret,
                static_cast<std::uint32_t>(ret));
  }
  ret = MV_CC_DestroyHandle(handle_);
  if (ret != MV_OK) {
    LOG_WARNING(logger_, "MV_CC_DestroyHandle failed in dtor: {} (0x{:08X})",
                ret, static_cast<std::uint32_t>(ret));
  }
  handle_ = nullptr;
}

void hardware::HikRobot::setFloatValue(const std::string &name, double value) {
  int ret;
  ret = MV_CC_SetFloatValue(handle_, name.c_str(), value);
  if (ret != MV_OK)
    LOG_WARNING(logger_, "MV_CC_SetFloatValue({}, {}) failed: {}", name, value,
                ret);
}

void hardware::HikRobot::setEnumValue(const std::string &name,
                                      unsigned int value) {
  int ret;
  ret = MV_CC_SetEnumValue(handle_, name.c_str(), value);
  if (ret != MV_OK)
    LOG_WARNING(logger_, "MV_CC_SetEnumValue({}, {}) failed: {}", name, value,
                ret);
}

bool hardware::HikRobot::setBoolValue(const std::string &name, bool value) {
  int ret = MV_CC_SetBoolValue(handle_, name.c_str(), value ? 1 : 0);
  if (ret != MV_OK) {
    LOG_WARNING(logger_, "MV_CC_SetBoolValue({}, {}) failed: {}", name, value,
                ret);
    return false;
  }
  return true;
}

bool hardware::HikRobot::readImage(
    unsigned char *buffer, std::size_t buffer_size,
    std::chrono::system_clock::time_point &stamp) {
  if (buffer_size < payload_size_) {
    LOG_ERROR(logger_, "Insufficient buffer size! require {}, actual {}",
              payload_size_, buffer_size);
    return false;
  }
  if (error_count_ > 5) {
    LOG_DEBUG(logger_, "[hikrobot]: error_count: {}, restart grabbing!",
              error_count_);
    int ret = MV_CC_StopGrabbing(handle_);
    if (ret != MV_OK) {
      LOG_WARNING(logger_, "MV_CC_StopGrabbing failed: {} (0x{:08X})", ret,
                  static_cast<std::uint32_t>(ret));
    }
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
      LOG_ERROR(logger_, "MV_CC_StartGrabbing failed: {} (0x{:08X})", ret,
                static_cast<std::uint32_t>(ret));
      error_count_++;
      return false;
    }
    error_count_ = 0;
  }
  MV_FRAME_OUT raw;
  unsigned int nMsec = 100;
  int ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
  if (ret != MV_OK) {
    LOG_ERROR(logger_, "MV_CC_GetImageBuffer failed: {} (0x{:08X})", ret,
              static_cast<std::uint32_t>(ret));
    error_count_++;
    return false;
  }

  bool success{false};
  const auto &frame_info = raw.stFrameInfo;

  // stamp = tools::nanoSecToChronoPoint(frame_info.nHostTimeStamp *
  // 1000000ull);
  // XXX: 硬件时间戳似乎两帧才更新一次，有点怪
  stamp = std::chrono::system_clock::now();

  auto pixel_type = frame_info.enPixelType;
  const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes>
      type_map = {{PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
                  {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
                  {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
                  {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}};
  if (!type_map.contains(pixel_type)) {
    LOG_ERROR(logger_, "Unsupported pixel type: {}",
              static_cast<std::uint32_t>(pixel_type));
  } else {
    const auto pixel_type_cv = type_map.at(pixel_type);
    const auto required_size = static_cast<std::size_t>(frame_info.nWidth) *
                               static_cast<std::size_t>(frame_info.nHeight) * 3;
    if (buffer_size < required_size) {
      LOG_ERROR(logger_, "Insufficient buffer for frame! require {}, actual {}",
                required_size, buffer_size);
    } else {
      cv::cvtColor(
          cv::Mat{cv::Size(frame_info.nWidth, frame_info.nHeight), CV_8U,
                  raw.pBufAddr},
          cv::Mat{frame_info.nHeight, frame_info.nWidth, CV_8UC3, buffer},
          pixel_type_cv);
      success = true;
    }
  }

  ret = MV_CC_FreeImageBuffer(handle_, &raw);
  if (ret != MV_OK)
    LOG_WARNING(logger_, "MV_CC_FreeImageBuffer failed: {} (0x{:08X})", ret,
                static_cast<std::uint32_t>(ret));

  if (success)
    error_count_ = 0;
  else
    error_count_++;

  return success;
}

bool hardware::HikRobot::changeExposureGain(double exposure, double gain) {
  LOG_INFO(logger_, "Try to change exposure time to {}.", exposure);
  setFloatValue("ExposureTime", exposure);
  LOG_INFO(logger_, "Try to change gain to {}.", gain);
  setFloatValue("Gain", gain);
  return true;
}
