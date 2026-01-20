#include "galaxy.hpp"

#include "quill/LogMacros.h"
#include "quill/Logger.h"

#define GX_SUCCESS(X) (X == GX_STATUS_SUCCESS)

hardware::Galaxy::Galaxy(quill::Logger *logger,
                         const confs::CameraParams &camera_params)
    : logger_(logger) {
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

  GXSendCommand(camera_handle_, GX_COMMAND_ACQUISITION_START); // 发送控制命令

  // TODO
}

int hardware::Galaxy::read(unsigned char *buffer, std::size_t buffer_size) {
  GX_FRAME_DATA bayer_frame{};
  GX_STATUS status;
  std::vector<char> bayer_buffer_holder;

  // Initialize frame   初始化帧
  int64_t payloadSize;
  GXGetInt(camera_handle_, GX_INT_PAYLOAD_SIZE, &payloadSize);
  bayer_buffer_holder.reserve(payloadSize);
  bayer_frame.pImgBuf = bayer_buffer_holder.data();

  RCLCPP_INFO(this->get_logger(), "Publishing image!");

  image_msg_.header.frame_id = "camera_optical_frame";
  image_msg_.encoding = "rgb8";

  while (rclcpp::ok()) {
    if (fail_conut_ > 5) {
      RCLCPP_FATAL(this->get_logger(), "Retry camera!");
      // rclcpp::shutdown();
      while (true) {
        uint32_t device_count = 0;
        status = GXUpdateDeviceList(&device_count, 100); // 枚举所有设备
        if (device_count < 1) {
          RCLCPP_WARN(this->get_logger(), "No camera found. device_count = %d",
                      device_count);
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }
        status = GXOpenDeviceByIndex(1, &camera_handle_); // 通过序号打开设备
        if (!GX_SUCCESS(status)) {
          // std::cout<<device_count<<std::endl;
          RCLCPP_ERROR(this->get_logger(), "Can not open camera, status = %d",
                       status);
        } else {
          GXSetBool(camera_handle_, GX_BOOL_REVERSE_X, 1);
          GXSetBool(camera_handle_, GX_BOOL_REVERSE_Y, 1);
          fail_conut_ = 0;
          break;
        }
      };
    }
    // double pnValue;
    // GXGetFloat(camera_handle_, GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE,
    // &pnValue); std::cout << "当前采集帧率: " << pnValue << std::endl;

    // int64_t link_cur;
    // GXGetInt(camera_handle_, GX_INT_DEVICE_LINK_CURRENT_THROUGHPUT,
    // &link_cur); std::cout<< "当前设备带宽: " << link_cur << std::endl;

    // Fetch image
    status = GXGetImage(camera_handle_, &bayer_frame, 700); // 直接获取一帧图像

    if (GX_SUCCESS(status)) {
      DX_PIXEL_COLOR_FILTER bayer_type;
      switch (
          bayer_frame.nPixelFormat) { // 每个像素在图像中存储的颜色信息的格式
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
        RCLCPP_FATAL(this->get_logger(), "Unsupported Bayer layout: %d!",
                     bayer_frame.nPixelFormat);
        return;
      }

      image_msg_.header.stamp = this->now();
      image_msg_.height = bayer_frame.nHeight;
      image_msg_.width = bayer_frame.nWidth;
      image_msg_.step = bayer_frame.nWidth * 3;
      image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

      status = DxRaw8toRGB24(bayer_frame.pImgBuf, image_msg_.data.data(),
                             bayer_frame.nWidth, bayer_frame.nHeight,
                             RAW2RGB_NEIGHBOUR, bayer_type, false);

      if (!GX_SUCCESS(status)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to convert Bayer to RGB, status = %d", status);
        continue;
      }

      RCLCPP_DEBUG(this->get_logger(), "Publish image: %dx%d",
                   bayer_frame.nWidth, bayer_frame.nHeight);

      camera_info_msg_.header = image_msg_.header;
      camera_pub_.publish(image_msg_, camera_info_msg_);

      fail_conut_ = 0;
    } else {
      RCLCPP_WARN(this->get_logger(), "Get buffer failed, status = %d", status);
      GXSendCommand(camera_handle_, GX_COMMAND_ACQUISITION_STOP);
      status = GXSendCommand(camera_handle_, GX_COMMAND_ACQUISITION_START);
      RCLCPP_INFO(this->get_logger(), "status = %d", status);
      fail_conut_++;
    }
    if (fail_conut_ > 5) {
      GXCloseDevice(camera_handle_);
    }
  }
}
