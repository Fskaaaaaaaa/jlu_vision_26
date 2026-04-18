#include "tracker_node.hpp"
#include "configs.hpp"
#include "hardware/cam_info_listener.hpp"
#include "hardware/gimbal_info_listener.hpp"
#include "hardware/task_mode_listener.hpp"
#include "msgs/BuffBlade.hpp"
#include "quill/LogMacros.h"
#include "targets.hpp"
#include "types.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include "types/TaskMode.hpp"
#include <memory>
#include <vector>

auto_buff::TrackerNode::TrackerNode(quill::Logger *logger,
                                    const TrackerConfigs &configs)
    : logger_(logger), configs_(configs),
      buff_blade_sub_(
          types::IceoryxServiceDescription{configs_.buff_blade_topic}
              .description),
      aimcommand_pub_(
          types::IceoryxServiceDescription{configs_.serial_topic}.description),
      task_mode_listener_(logger, types::TaskMode::SmallBuff),
      tf_listener_(logger_, tf_buffer_), gimbal_info_listener_(logger_) {
  LOG_INFO(logger_, "TrackerNode start!");
  tf_listener_.init();
  auto camera_info =
      hardware::CameraInfoListener{logger_, configs_.camera_name}.get();
  LOG_INFO(logger_, "Get camera_info");
  this->camera_matrix_ = camera_info.camera_matrix.clone();
  this->distortion_coefficients_ = camera_info.distortion_coefficients.clone();
  this->small_buff_target_ = std::make_unique<SmallBuffTarget>(
      logger_, configs_.small_buff_conf, camera_matrix_,
      distortion_coefficients_);
  LOG_INFO(logger_, "Add Target: SmallBuff");
  // 开始订阅扇叶
  buff_blade_listener_
      .attachEvent(buff_blade_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onBladesReceivedCallback, *this))
      .or_else([this](auto) {
        LOG_CRITICAL(logger_, "Failed to attach buff_blade_sub");
        std::exit(EXIT_FAILURE);
      });
  // 启动云台规划线程
  // this->plan_thread_ = std::jthread{[this](){}
  // 启动可视化线程
  if (configs_.show_image)
    this->image_poller_ =
        std::make_unique<hardware::ImagePoller<msgs::Image1440x1080_8UC3>>(
            logger_, configs_.camera_name,
            [this](const cv::Mat &image, const std::string &frame_id,
                   const std::chrono::system_clock::time_point &stamp) {
              cv::Mat copy;
              image.copyTo(copy);
              cv::imshow("tracker", copy);
              cv::waitKey(1);
            });
  LOG_INFO(logger_, "Tracker node initialization complete!");
}

void auto_buff::TrackerNode::onBladesReceivedCallback(
    iox::popo::Subscriber<msgs::BuffBlade, msgs::Header> *subscriber,
    TrackerNode *self) {
  std::vector<BuffBlade> blades;
  while (subscriber->take().and_then(
      [&blades, subscriber,
       self](const iox::popo::Sample<const msgs::BuffBlade, const msgs::Header>
                 &sample) {
        if (subscriber->getServiceDescription().getInstanceIDString() ==
            self->buff_blade_sub_.getServiceDescription()
                .getInstanceIDString()) {
          BuffBlade blade{sample};
          blades.emplace_back(blade);
        }
      })) {
  } // end of while
  // 排除连心跳都没有的假唤醒
  if (blades.empty())
    return;
  // 由于frame_id从相机的配置文件读取，依赖图像回调获得，故心跳帧只有时间戳无坐标系
  auto image_stamp = blades.front().stamp; // 假设一次接收的armors来自同一帧
  std::erase_if(blades, [&](const BuffBlade &blade) {
    if (blade.heart_beat)
      return true;
    if (blade.stamp != image_stamp)
      return true;
    return false;
  }); // 之后blades可能为空
  LOG_TRACE_L1(self->logger_, "receive {} valid blades.", blades.size());
  // TODO: 完成NODE
}
