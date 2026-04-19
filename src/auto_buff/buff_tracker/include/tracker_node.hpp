#pragma once

#include "basic/colors.hpp"
#include "basic/plotter.hpp"
#include "configs.hpp"
#include "hardware/gimbal_info_listener.hpp"
#include "hardware/image_poller.hpp"
#include "hardware/task_mode_listener.hpp"
#include "msgs/AimCommand.hpp"
#include "msgs/BuffBlade.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"
#include "targets.hpp"
#include "transform/tf_listener.hpp"

#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "quill/Logger.h"
#include "types.hpp"

#include <chrono>
#include <memory>
namespace auto_buff {

class TrackerNode {
public:
  TrackerNode(quill::Logger *logger, const TrackerConfigs &configs);

private:
  static void onBladesReceivedCallback(
      iox::popo::Subscriber<msgs::BuffBlade, msgs::Header> *subscriber,
      TrackerNode *self);

  void drawBuffBlade(const BladePositionRoll &blade_odom, cv::Mat &image,
                     const Eigen::Isometry3d &T_odom_to_camera,
                     const cv::Scalar &color = tools::Color::bgr::RED,
                     const std::string &txt = "") const;
  // 子类向常引用基类的转换不会发生对象切片
  void drawBuff(const BuffTarget &buff_target, cv::Mat &image,
                const std::chrono::system_clock::time_point &stamp,
                const Eigen::Isometry3d &T_odom_to_camera) const;
  void imageCallback(const cv::Mat &image, const std::string &frame_id,
                     const std::chrono::system_clock::time_point &stamp) const;

  quill::Logger *logger_;
  TrackerConfigs configs_;
  iox::popo::Subscriber<msgs::BuffBlade, msgs::Header> buff_blade_sub_;
  iox::popo::Publisher<msgs::AimCommand, msgs::Header> aimcommand_pub_;
  hardware::TaskModeListener task_mode_listener_;
  fast_tf::detail::transform_buffer tf_buffer_;
  tf::TransformListener tf_listener_;
  hardware::GimbalInfoListener gimbal_info_listener_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  iox::popo::Listener buff_blade_listener_;
  std::unique_ptr<SmallBuffTarget> small_buff_target_;
  // BigBuffTarget big_buff_target_;
  std::jthread plan_thread_;
  // for debug
  std::unique_ptr<hardware::ImagePoller<msgs::Image1440x1080_8UC3>>
      image_poller_;
  tools::Plotter plotter_;
};

} // namespace auto_buff
