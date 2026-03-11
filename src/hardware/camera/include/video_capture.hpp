#pragma once

#include "camera.hpp"

#include "opencv2/videoio.hpp"
#include "quill/Logger.h"
#include <chrono>

namespace hardware {

class VideoCapture : public CameraBase {
public:
  VideoCapture(quill::Logger *logger, const std::string &video_path);
  bool readImage(unsigned char *buffer, std::size_t buffer_size,
                 std::chrono::system_clock::time_point &stamp) override;
  // NOTE: 从得到的工业相机sdk定义的图像类型写入缓冲区（ioxsample）
  bool changeExposureGain(double exposure, double gain) override;

private:
  quill::Logger *logger_;
  cv::VideoCapture video_;
  std::string video_path_;
};

} // namespace hardware
