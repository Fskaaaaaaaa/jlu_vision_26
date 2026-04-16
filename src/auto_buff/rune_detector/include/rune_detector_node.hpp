#pragma once
#include "single.hpp"
#include "hardware/image_poller.hpp"
#include "msgs/Image.hpp"
#include "hardware/enemy_color_listener.hpp"
#include "hardware/camera_params_changer.hpp"
#include "hardware/task_mode_listener.hpp"


#include <opencv2/core/types.hpp>

#include <chrono>
#include <string>
#include <memory>

namespace auto_buff {
class RuneDetectorNode : public Single<RuneDetectorNode> {
  friend class Single<RuneDetectorNode>;
protected:
  RuneDetectorNode();
  ~RuneDetectorNode();

public:
  int run();

private:
  void init();
  void imageCallback(const cv::Mat &image, const std::string &frame_id,
                     const std::chrono::system_clock::time_point &stamp);

private:
  hardware::CameraParamsChanger cam_params_changer_;
  hardware::EnemyColorListener enemy_color_listener_;
  hardware::TaskModeListener* task_mode_listener_;
  std::unique_ptr<hardware::ImagePoller<msgs::Image1440x1080_8UC3>> image_poller_;
};
} // namespace auto_buff
