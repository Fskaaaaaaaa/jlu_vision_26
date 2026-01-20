
#include "camera.hpp"
#include "configs.hpp"
#include "galaxy.hpp"
#include "hikrobot.hpp"

namespace hardware {
Camera::Camera(const CameraConfigs &configs) {

  // if (camera_name == "mindvision") {
  //   auto gamma = tools::read<double>(yaml, "gamma");
  //   auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
  //   camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  // }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat &img,
                  std::chrono::steady_clock::time_point &timestamp) {
  camera_->read(img, timestamp);
}

} // namespace hardware
