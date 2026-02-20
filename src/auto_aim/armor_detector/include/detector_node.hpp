// Copyright (c) 2026 idot. All Rights Reserved.
#pragma once

#include "ba_solver.hpp"
#include "configs.hpp"
#include "detector.hpp"
#include "hardware/image_listener.hpp"
#include "hardware/task_mode_listener.hpp"
#include "lightbar_corrector.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"
#include "pnp_solver.hpp"
#include "types.hpp"

#include "iceoryx_posh/popo/publisher.hpp"
#include "quill/Logger.h"

#include <memory>
#include <thread>

namespace auto_aim {

class DetectorNode {
public:
  DetectorNode(quill::Logger *logger, const DetectorConfigs &configs);

private:
  void drawArmor(const Armor &armor, cv::Mat &image);

  quill::Logger *logger_;
  DetectorConfigs configs_;

  hardware::ImageListener<msgs::Image1440x1080_8UC3> image_listener_;
  iox::popo::Publisher<msgs::Armor, msgs::Header> armor_pub_;

  std::unique_ptr<MTDetector> mt_detector_;
  std::optional<std::jthread> pop_thread_;
  std::unique_ptr<STDetector> st_detector_;
  LightCornerCorrector lightbar_corrector_;
  PnPSolver pnp_solver_;
  BaSolver ba_solver_;

  hardware::TaskModeListener task_mode_listener_;
};

} // namespace auto_aim
