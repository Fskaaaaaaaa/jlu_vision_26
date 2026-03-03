// Copyright (c) 2026 idot. All Rights Reserved.
#pragma once

#include "ba_solver.hpp"
#include "configs.hpp"
#include "detector.hpp"
#include "hardware/camera_params_changer.hpp"
// #include "hardware/image_listener.hpp"
#include "hardware/image_poller.hpp"
#include "hardware/task_mode_listener.hpp"
#include "lightbar_corrector.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"
#include "opencv2/core/types.hpp"
#include "pnp_solver.hpp"
#include "types.hpp"

#include "iceoryx_posh/popo/publisher.hpp"
#include "quill/Logger.h"
#include "types/EnemyColor.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <thread>

namespace auto_aim {

class DetectorNode {
public:
  DetectorNode(quill::Logger *logger, const DetectorConfigs &configs);
  ~DetectorNode();

private:
  void imageCallback(const cv::Mat &image, const std::string &frame_id,
                     const std::chrono::system_clock::time_point &stamp);
  // NOTE:
  // PCA、PNP、BA、publish，需要保证传入的armors的生命周期，返回调试用的图像
  std::optional<cv::Mat>
  afterDetect(const cv::Mat &bgr_image, std::vector<Armor> &armors,
              const std::string &frame_id,
              const std::chrono::system_clock::time_point &stamp);
  void publishArmors(const std::vector<Armor> &armors);
  void publishHeartbeat(const std::chrono::system_clock::time_point &stamp);
  void drawArmor(const Armor &armor, cv::Mat &image, const cv::Scalar &color,
                 bool draw_text = false);

  quill::Logger *logger_;
  DetectorConfigs configs_;
  types::EnemyColor self_color_;

  hardware::CameraParamsChanger cam_params_changer_;
  std::unique_ptr<hardware::TaskModeListener> task_mode_listener_;
  // std::unique_ptr<hardware::ImageListener<msgs::Image1440x1080_8UC3>>
  //     image_listener_;
  std::unique_ptr<hardware::ImagePoller<msgs::Image1440x1080_8UC3>>
      image_poller_;

  iox::popo::Publisher<msgs::Armor, msgs::Header> armor_pub_;

  std::unique_ptr<MTDetector> mt_detector_;
  std::unique_ptr<STDetector> st_detector_;
  std::unique_ptr<LightCornerCorrector> lightbar_corrector_;
  std::unique_ptr<PnPSolver> pnp_solver_;
  std::unique_ptr<BASolver> ba_solver_;

  std::jthread pop_thread_;
};

} // namespace auto_aim
