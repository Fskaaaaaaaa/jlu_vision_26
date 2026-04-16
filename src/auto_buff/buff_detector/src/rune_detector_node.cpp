#include "rune_detector_node.hpp"
#include "configs.hpp"

#include "opencv2/highgui.hpp"
#include "quill/LogMacros.h"

auto_buff::RuneDetectorNode::RuneDetectorNode()
    : cam_params_changer_(ConfigManager::instance()->logger(),
                          ConfigManager::instance()->configs().camera_name),
      enemy_color_listener_(
          ConfigManager::instance()->logger(),
          ConfigManager::instance()->configs().default_enemy_color)
    {
  
}

auto_buff::RuneDetectorNode::~RuneDetectorNode() {
  cv::destroyAllWindows();
}

int auto_buff::RuneDetectorNode::run() {
  LOG_INFO(ConfigManager::instance()->logger(), "rune detector node is running!");
  init();

  while (true) {
  }

  return 0;
}

void auto_buff::RuneDetectorNode::init() {
  this->task_mode_listener_ = new hardware::TaskModeListener(
      ConfigManager::instance()->logger(), types::TaskMode::SmallBuff, [this]() {
        this->cam_params_changer_.changeCameraParams(
            ConfigManager::instance()->configs().camera_params);
        LOG_INFO(ConfigManager::instance()->logger(), "on task! change camera params.");
      });
  this->image_poller_ =
      std::make_unique<hardware::ImagePoller<msgs::Image1440x1080_8UC3>>(
          ConfigManager::instance()->logger(),
          ConfigManager::instance()->configs().camera_name,
          [this](const cv::Mat &image, const std::string frame_id,
                 const std::chrono::system_clock::time_point &stamp) {
            bool should_continue =
                task_mode_listener_->isOnTask() ||
                (ConfigManager::instance()->configs().detect_when_idle &&
                 task_mode_listener_->isTask(types::TaskMode::Idle));
            if (should_continue)
              this->imageCallback(image, frame_id, stamp);
          });
}

void auto_buff::RuneDetectorNode::imageCallback(
    const cv::Mat &image, const std::string &frame_id,
    const std::chrono::system_clock::time_point &stamp) {
  // auto infer_start = std::chrono::system_clock::now();
  cv::imshow("image", image);
  cv::waitKey(1);
  // auto infer_end = std::chrono::system_clock::now();
  // auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
  //               infer_end - infer_start)
  //               .count() *
  //           1000;
  // auto camera_latency =
  //     std::chrono::duration_cast<std::chrono::duration<double>>(infer_start -
  //                                                               stamp)
  //         .count() *
  //     1000;
}