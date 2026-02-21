#include "video_capture.hpp"

#include "opencv2/core/types.hpp"
#include "opencv2/videoio.hpp"
#include "quill/LogMacros.h"

#include <cstddef>
#include <cstdlib>

hardware::VideoCapture::VideoCapture(quill::Logger *logger,
                                     const std::string &video_path)
    : logger_(logger), video_(video_path), video_path_(video_path) {
  LOG_INFO(logger_, "VideoCapture start!");
}

int hardware::VideoCapture::captureImage(unsigned char *buffer,
                                         std::size_t buffer_size) {
  cv::Mat frame;
  video_ >> frame;
  if (frame.empty()) {
    LOG_INFO(logger_, "The video has finished playing and will restart.");
    video_.set(cv::CAP_PROP_POS_FRAMES, 0);
    video_ >> frame;
    if (frame.empty()) {
      this->video_ = cv::VideoCapture(video_path_);
      video_ >> frame;
      if (frame.empty()) {
        LOG_ERROR(logger_, "Failed to read frame after reset.");
        std::exit(EXIT_FAILURE);
      }
    }
  }
  if (frame.channels() != 3) {
    LOG_CRITICAL(logger_, "Frame channels must be 3!");
    std::exit(EXIT_FAILURE);
  }
  cv::Size2i target_size{1440, 1080};
  std::size_t image_data_size = target_size.area() * 3;
  if (image_data_size > buffer_size) {
    LOG_CRITICAL(logger_, "Insufficient buffer size! require {}, actual {}",
                 image_data_size, buffer_size);
    std::exit(EXIT_FAILURE);
  }

  // 保持长宽比的缩放，可能裁切
  double scale = std::max(static_cast<double>(target_size.width) / frame.cols,
                          static_cast<double>(target_size.height) / frame.rows);
  cv::Size scaledSize(static_cast<int>(std::round(frame.cols * scale)),
                      static_cast<int>(std::round(frame.rows * scale)));
  cv::Mat scaled;
  cv::resize(frame, scaled, scaledSize, 0, 0, cv::INTER_LINEAR);
  int cropX = (scaled.cols - target_size.width) / 2;
  int cropY = (scaled.rows - target_size.height) / 2;
  cv::Rect roi(cropX, cropY, target_size.width, target_size.height);
  cv::Mat(target_size, CV_8UC3, buffer) = scaled(roi);
  cv::resize(frame, cv::Mat{target_size, CV_8UC3, buffer}, target_size);
  return 0;
}

int hardware::VideoCapture::changeExposureGain(double exposure, double gain) {
  return 0;
}
