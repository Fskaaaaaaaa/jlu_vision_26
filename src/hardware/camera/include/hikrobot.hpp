#pragma once
#include "camera.hpp"
#include "quill/Logger.h"

#include <chrono>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <string>

namespace hardware {
class HikRobot : public CameraBase {
public:
  HikRobot(quill::Logger *logger, const confs::CameraParams &camera_params);
  ~HikRobot() override;
  bool readImage(unsigned char *buffer, std::size_t buffer_size,
                 std::chrono::system_clock::time_point &stamp) override;
  bool changeExposureGain(double exposure, double gain) override;

private:
  void setFloatValue(const std::string &name, double value);
  void setEnumValue(const std::string &name, unsigned int value);

  quill::Logger *logger_;
  confs::CameraParams camera_params_;
  void *handle_;
  std::vector<char> bayer_buffer_holder_;
  bool buffer_inited_;
  int error_count_;
  std::size_t payload_size_;
};

} // namespace hardware
