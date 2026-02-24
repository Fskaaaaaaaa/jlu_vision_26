// Copyright (c) 2026 Can U send me 30k. All Rights Reserved.
#pragma once

#include "basic/colors.hpp"
#include "basic/plotter.hpp"
#include "configs.hpp"
#include "hardware/gimbal_info_listener.hpp"
#include "hardware/image_listener.hpp"
#include "hardware/task_mode_listener.hpp"
#include "msgs/AimCommand.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"
#include "planner.hpp"
#include "target.hpp"
#include "transform/tf_listener.hpp"
#include "types.hpp"
#include "types/ArmorType.hpp"

#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "opencv2/core/types.hpp"
#include "quill/Logger.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace auto_aim {

class TrackerNode {
public:
  TrackerNode(quill::Logger *logger, const TrackerConfigs configs);

private:
  static void onArmorsReceivedCallback(
      iox::popo::Subscriber<msgs::Armor, msgs::Header> *subscriber,
      TrackerNode *self);
  // NOTE: 注意使用前设锁
  void drawTarget(const Target &target, cv::Mat &image,
                  const std::chrono::system_clock::time_point &img_stamp) const;
  void drawArmor(const Armor &armor, cv::Mat &image,
                 const std::chrono::system_clock::time_point &img_stamp,
                 const cv::Scalar &color = tools::Color::bgr::RED,
                 bool publish_armor = false) const;

  quill::Logger *logger_;
  TrackerConfigs configs_;

  iox::popo::Subscriber<msgs::Armor, msgs::Header> armors_sub_;
  iox::popo::Listener armors_listener_;
  hardware::TaskModeListener task_mode_listener_;
  tf::TransformListener tf_listener_;
  fast_tf::detail::transform_buffer tf_buffer_;
  hardware::GimbalInfoListener gimbal_listener_;
  iox::popo::Publisher<msgs::AimCommand, msgs::Header> aimcommand_pub_;

  std::unordered_map<types::ArmorType, std::unique_ptr<Target>> targets_;
  std::atomic<types::ArmorType> selected_target_;
  Planner planner_;
  std::jthread plan_thread_;

  // for debug usage
  std::unique_ptr<hardware::ImageListener<msgs::Image1440x1080_8UC3>>
      image_listener_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  std::pair<cv::Mat, std::chrono::system_clock::time_point> image_stamp_cache_;
  std::mutex image_mtx_;
  std::jthread image_show_thread_;
  std::optional<Armor> aimed_armor_cache_;
  std::unique_ptr<iox::popo::Publisher<msgs::Armor, msgs::Header>> armors_pub_;
  tools::Plotter plotter_;
};

} // namespace auto_aim
