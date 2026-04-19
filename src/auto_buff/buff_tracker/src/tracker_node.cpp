#include "tracker_node.hpp"
#include "basic/colors.hpp"
#include "basic/image_tools.hpp"
#include "configs.hpp"
#include "hardware/cam_info_listener.hpp"
#include "hardware/gimbal_info_listener.hpp"
#include "hardware/task_mode_listener.hpp"
#include "math/angle_tools.hpp"
#include "msgs/BuffBlade.hpp"
#include "targets.hpp"
#include "types.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include "types/TaskMode.hpp"

#include "opencv2/highgui.hpp"
#include "quill/LogMacros.h"
#include <opencv2/core/eigen.hpp>

#include <cmath>
#include <memory>
#include <string>
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
              this->imageCallback(image, frame_id, stamp);
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
  Eigen::Isometry3d T_camera_to_odom = Eigen::Isometry3d::Identity();
  if (!blades.empty())
    try {
      T_camera_to_odom = self->tf_buffer_.get(
          self->configs_.odom_frame_id, blades.front().frame_id,
          blades.front().stamp,
          std::chrono::nanoseconds{static_cast<int64_t>(
              self->configs_.tf_query_tolerance_ms * 1e6)});
    } catch (const std::exception &e) {
      LOG_ERROR(self->logger_, "tf from {} to {} failed: {}",
                blades.front().frame_id, self->configs_.odom_frame_id,
                e.what());
      blades.clear(); // 失败舍弃该帧接收的扇叶
    }
  if (self->task_mode_listener_.isTask(types::TaskMode::SmallBuff) ||
      self->configs_.always_on_task_small_buff)
    self->small_buff_target_->track(blades, image_stamp, T_camera_to_odom);
  // XXX: 这里最好加上些发布调试信息的东西
}

void auto_buff::TrackerNode::drawBuffBlade(
    const BladePositionRoll &blade_odom, cv::Mat &image,
    const Eigen::Isometry3d &T_odom_to_camera, const cv::Scalar &color,
    const std::string &txt) const {
  Eigen::Isometry3d blade_pose_odom = Eigen::Isometry3d::Identity();
  blade_pose_odom.pretranslate(blade_odom.position);
  // HACK: 这里的yaw最好加到因子图里
  blade_pose_odom.rotate(tools::rpyToQuaterniond(
      {blade_odom.roll.theta(), 0.0,
       std::atan2(blade_odom.position.y(), blade_odom.position.x())}));
  Eigen::Isometry3d blade_pose_camera = T_odom_to_camera * blade_pose_odom;
  Eigen::Matrix3d R_eg = blade_pose_camera.rotation();
  Eigen::Vector3d t_eg = blade_pose_camera.translation();
  cv::Mat R, rvec, tvec;
  cv::eigen2cv(R_eg, R);
  cv::Rodrigues(R, rvec);
  cv::eigen2cv(t_eg, tvec);
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(BUFF_BLADE_OBJ_POINTS, rvec, tvec, camera_matrix_,
                    distortion_coefficients_, image_points);
  tools::drawPoints(image, image_points, color);
  cv::putText(image, txt, image_points.front(), cv::FONT_HERSHEY_SIMPLEX, 1.0,
              color, 2);
}

void auto_buff::TrackerNode::drawBuff(
    const BuffTarget &buff_target, cv::Mat &image,
    const std::chrono::system_clock::time_point &stamp,
    const Eigen::Isometry3d &T_odom_to_camera) const {
  auto [buff_state, track_state] = buff_target.getTargetTrackState();
  if (track_state.state == TrackState::State::LOST)
    return;
  auto dt = std::chrono::duration_cast<
                std::chrono::duration<double, std::chrono::seconds::period>>(
                stamp - track_state.stamp_last_update)
                .count();
  buff_state = buff_state.predict(dt);
  auto buff_index{0};
  for (const auto &blade : buff_state.blades())
    drawBuffBlade(blade, image, T_odom_to_camera, tools::Color::bgr::RED,
                  std::to_string(buff_index++));
}

void auto_buff::TrackerNode::imageCallback(
    const cv::Mat &image, const std::string &frame_id,
    const std::chrono::system_clock::time_point &stamp) const {
  Eigen::Isometry3d T_odom_to_camera = Eigen::Isometry3d::Identity();
  try {
    T_odom_to_camera =
        this->tf_buffer_.get(frame_id, configs_.odom_frame_id, stamp,
                             std::chrono::nanoseconds{static_cast<int64_t>(
                                 configs_.tf_query_tolerance_ms * 1e6)});
  } catch (const std::exception &e) {
    LOG_ERROR(logger_, "tf from {} to {} failed: {}", frame_id,
              configs_.odom_frame_id, e.what());
    return;
  }
  cv::Mat copy;
  image.copyTo(copy);
  this->drawBuff(*small_buff_target_, copy, stamp, T_odom_to_camera);
  // TODO: 完成调试信息绘制
  cv::imshow("buff_tracker", copy);
  cv::waitKey(1);
}
