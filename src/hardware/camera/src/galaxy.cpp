#include "galaxy.hpp"

#include "GxIAPI.h"
#include "confs/CameraParams.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "quill/LogMacros.h"
#include "quill/Logger.h"
#include <cstdlib>
#include <unordered_map>

#define GX_SUCCESS(X) (X == GX_STATUS_SUCCESS)

hardware::Galaxy::Galaxy(quill::Logger *logger,
                         const confs::CameraParams &camera_params)
    : logger_(logger), camera_params_({0, 0}), buffer_inited_(false) {
  LOG_INFO(logger_, "starting galaxy camera.");
  GX_STATUS status;
  // Init lib
  status = GXInitLib();
  if (!GX_SUCCESS(status)) {
    LOG_CRITICAL(logger, "Init GxIAPI failed, code = {}!", status);
    std::exit(status);
  }
  // Constantly try opening one camera
  while (true) {
    uint32_t device_count = 0;
    status = GXUpdateDeviceList(&device_count, 100); // 枚举所有设备
    if (device_count < 1) {
      LOG_WARNING(logger_, "No camera found. device_count = {}", device_count);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    status = GXOpenDeviceByIndex(1, &camera_handle_); // 通过序号打开设备
    if (!GX_SUCCESS(status)) {
      LOG_ERROR(logger_, "Can not open camera, status = {}", status);
    } else {
      break;
    }
  };
  // Get camera infomation
  GXGetInt(camera_handle_, GX_INT_WIDTH, &img_info_.nWidthValue); // 图像宽度
  GXGetInt(camera_handle_, GX_INT_WIDTH_MAX,
           &img_info_.nWidthMax); // 最大宽度
  GXGetInt(camera_handle_, GX_INT_HEIGHT, &img_info_.nHeightValue);
  GXGetInt(camera_handle_, GX_INT_HEIGHT_MAX, &img_info_.nHeightMax);

  // Set default exp gain
  this->changeExposureGain(camera_params.exposure_time, camera_params.gain);
  this->camera_params_ = camera_params;
  // 设置帧率(为了解决缓冲区满导致的抽搐问题)
  GXSetEnum(camera_handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
            GX_ACQUISITION_FRAME_RATE_MODE_ON);
  GXSetFloat(camera_handle_, GX_FLOAT_ACQUISITION_FRAME_RATE,
             camera_params_.frame_rate);

  // 开始采集
  GXSendCommand(camera_handle_,
                GX_COMMAND_ACQUISITION_START); // 发送控制命令
}

int hardware::Galaxy::captureImage(unsigned char *buffer,
                                   std::size_t buffer_size) {
  GX_FRAME_DATA bayer_frame{};
  GX_STATUS status;

  // Initialize frame   初始化帧
  int64_t payloadSize;
  GXGetInt(camera_handle_, GX_INT_PAYLOAD_SIZE, &payloadSize);
  if (!buffer_inited_) {
    this->bayer_buffer_holder_.reserve(payloadSize);
    buffer_inited_ = true;
  }
  if (buffer_size < payloadSize) {
    LOG_ERROR(logger_, "Insufficient buffer size! require {}, actual {}",
              payloadSize, buffer_size);
    return EXIT_FAILURE;
  }
  bayer_frame.pImgBuf = bayer_buffer_holder_.data();

  if (fail_conut_ > 5) {
    LOG_ERROR(this->logger_, "Retry camera!");
    GXCloseDevice(camera_handle_);
    while (true) {
      uint32_t device_count = 0;
      status = GXUpdateDeviceList(&device_count, 100); // 枚举所有设备
      if (device_count < 1) {
        LOG_WARNING(this->logger_, "No camera found. device_count = {}",
                    device_count);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      status = GXOpenDeviceByIndex(1, &camera_handle_); // 通过序号打开设备
      if (!GX_SUCCESS(status)) {
        // std::cout<<device_count<<std::endl;
        LOG_ERROR(this->logger_, "Can not open camera, status = {}", status);
      } else {
        GXSetBool(camera_handle_, GX_BOOL_REVERSE_X, 1);
        GXSetBool(camera_handle_, GX_BOOL_REVERSE_Y, 1);
        fail_conut_ = 0;
        break;
      }
    };
  }
  double pn_value;
  GXGetFloat(camera_handle_, GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE,
             &pn_value);
  LOG_DEBUG(logger_, "Current acquisition frame rate: {}", pn_value);

  int64_t link_cur;
  GXGetInt(camera_handle_, GX_INT_DEVICE_LINK_CURRENT_THROUGHPUT, &link_cur);
  LOG_DEBUG(logger_, "Current device bandwidth: {}", link_cur);

  // Fetch image
  status = GXGetImage(camera_handle_, &bayer_frame, 500); // 直接获取一帧图像

  if (!GX_SUCCESS(status)) {
    LOG_WARNING(logger_, "Get buffer failed, status = {}", status);
    GXSendCommand(camera_handle_, GX_COMMAND_ACQUISITION_STOP);
    status = GXSendCommand(camera_handle_, GX_COMMAND_ACQUISITION_START);
    LOG_INFO(logger_, "status = {}", status);
    fail_conut_++;
    return EXIT_FAILURE;
  }
  DX_PIXEL_COLOR_FILTER bayer_type;
  switch (bayer_frame.nPixelFormat) { // 每个像素在图像中存储的颜色信息的格式
  case GX_PIXEL_FORMAT_BAYER_GR8:
    bayer_type = BAYERGR;
    break;
  case GX_PIXEL_FORMAT_BAYER_RG8:
    bayer_type = BAYERRG;
    break;
  case GX_PIXEL_FORMAT_BAYER_GB8:
    bayer_type = BAYERGB;
    break;
  case GX_PIXEL_FORMAT_BAYER_BG8:
    bayer_type = BAYERBG;
    break;
  default:
    LOG_CRITICAL(logger_, "Unsupported Bayer layout: {}!",
                 bayer_frame.nPixelFormat);
    return EXIT_FAILURE;
  }
  const static std::unordered_map<DX_PIXEL_COLOR_FILTER,
                                  cv::ColorConversionCodes>
      type_map{
          {BAYERGB, cv::COLOR_BayerGB2BGR},
          {BAYERGR, cv::COLOR_BayerGR2BGR},
          {BAYERRG, cv::COLOR_BayerRG2BGR},
          {BAYERBG, cv::COLOR_BayerBG2BGR},
      };
  // status =
  //     DxRaw8toRGB24(bayer_frame.pImgBuf, buffer, bayer_frame.nWidth,
  //                   bayer_frame.nHeight, RAW2RGB_NEIGHBOUR, bayer_type,
  //                   false);
  cv::cvtColor(
      cv::Mat{bayer_frame.nHeight, bayer_frame.nWidth, CV_8U,
              bayer_frame.pImgBuf},
      cv::Mat{bayer_frame.nHeight, bayer_frame.nWidth, CV_8UC3, buffer},
      type_map.at(bayer_type));
  // if (!GX_SUCCESS(status)) {
  //   LOG_ERROR(logger_, "Failed to convert Bayer to RGB, status = {}",
  //   status); return EXIT_FAILURE;
  // }
  LOG_DEBUG(logger_, "Get image: {}x{}", bayer_frame.nWidth,
            bayer_frame.nHeight);
  fail_conut_ = 0;
  return EXIT_SUCCESS;
}

int hardware::Galaxy::changeExposureGain(double exposure, double gain) {
  GX_STATUS status;
  if (exposure != camera_params_.exposure_time) {
    status = GXSetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME, exposure);
    if (!GX_SUCCESS(status)) {
      LOG_ERROR(logger_, "Failed to change exposure to {}!", exposure);
    } else {
      LOG_INFO(logger_, "Succeeded to change exposure time from {} to {}!",
               camera_params_.exposure_time, exposure);
      camera_params_.exposure_time = exposure;
    }
  }
  if (gain != camera_params_.gain) {
    status = GXSetFloat(camera_handle_, GX_FLOAT_GAIN, gain);
    if (!GX_SUCCESS(status)) {
      LOG_ERROR(logger_, "Failed to change gain to {}!", gain);
    } else {
      LOG_INFO(logger_, "Succeeded to change gain from {} to {}!",
               camera_params_.gain, gain);
      camera_params_.gain = gain;
    }
  }
  return status;
}
