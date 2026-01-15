#pragma once
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace hardware {
class CameraBase {
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat &img,
                    std::chrono::steady_clock::time_point &timestamp) = 0;
};

class Camera {
public:
  Camera(const std::string &config_path);
  void read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp);

private:
  std::unique_ptr<CameraBase> camera_;
};

} // namespace hardware
