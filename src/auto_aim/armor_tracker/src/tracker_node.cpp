// Copyright (c) 2026 I dont have 30k. All Rights Reserved.
// TODO 防御性编程添加相机离线检测，或者相机离线后发布警告图像
#include "tracker_node.hpp"
#include "basic/colors.hpp"
#include "basic/image_tools.hpp"
#include "configs.hpp"
#include "hardware/cam_info_listener.hpp"
#include "hardware/gimbal_info_listener.hpp"
#include "hardware/image_poller.hpp"
#include "hardware/task_mode_listener.hpp"
#include "math/angle_tools.hpp"
#include "msgs/AimCommand.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"
#include "planner.hpp"
#include "target.hpp"
#include "types.hpp"
#include "types/Armor.hpp"
#include "types/ArmorPoints.hpp"
#include "types/ArmorType.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include "types/TaskMode.hpp"

#include "iceoryx_posh/internal/popo/base_subscriber.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iox/signal_watcher.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "quill/LogMacros.h"
#include "rfl/enums.hpp"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <memory>
#include <thread>
#include <vector>

auto_aim::TrackerNode::TrackerNode(quill::Logger *logger,
                                   const TrackerConfigs configs)
    : logger_(logger), configs_(configs),
      armors_sub_(types::IceoryxServiceDescription{configs_.armors_sub_topic}
                      .description),
      task_mode_listener_(logger_, types::TaskMode::Armor),
      tf_listener_(logger_, tf_buffer_), gimbal_listener_(logger_),
      aimcommand_pub_(
          types::IceoryxServiceDescription{configs_.serial_topic}.description),
      aiming_target_(types::ArmorType::Negative),
      planner_(logger_, configs_.planner_conf) {
  LOG_INFO(logger_, "start tracker node!");
  // 添加目标
  for (auto target_type : std::array{
           types::ArmorType::One, types::ArmorType::Two,
           types::ArmorType::Three, types::ArmorType::Four,
           types::ArmorType::Sentry,
           // types::ArmorType::Outpost,
           // types::ArmorType::Base,
       }) {
    targets_.emplace(target_type,
                     std::make_unique<RobotTarget>(logger_, configs_.robot_conf,
                                                   target_type));
    LOG_INFO(logger_, "add target {}.", rfl::enum_to_string(target_type));
  }
  // 初始化畸变内参和坐标变换
  tf_listener_.init();
  auto camera_info =
      hardware::CameraInfoListener{logger_, configs_.camera_name}.get();
  this->camera_matrix_ = camera_info.camera_matrix.clone();
  this->distortion_coefficients_ = camera_info.distortion_coefficients.clone();
  // 开始订阅装甲板
  armors_listener_
      .attachEvent(armors_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onArmorsReceivedCallback, *this))
      .or_else([this](auto) {
        LOG_CRITICAL(logger_, "unable to attach armors_sub");
        std::exit(EXIT_FAILURE);
      });
  // 启动云台规划线程
  this->plan_thread_ = std::jthread{[this]() {
    LOG_INFO(logger_, "plan_thread start!");
    while (!iox::hasTerminationRequested()) {
      bool on_task =
          configs_.always_on_task ? true : task_mode_listener_.isOnTask();
      if (!on_task || !targets_.contains(aiming_target_.load())) { // 无锁定状态
        aimcommand_pub_.loan().and_then(
            [&](iox::popo::Sample<msgs::AimCommand, msgs::Header> &sample) {
              sample->control = false;
              sample.publish();
            });
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
            configs_.planner_conf.fail_polling_interval_sec * 1000)));
        continue;
      }
      auto [target_state, track_state] =
          targets_.at(aiming_target_.load())->getTargetTrackState();
      if (track_state.state == TrackState::State::LOST) {
        aimcommand_pub_.loan().and_then(
            [&](iox::popo::Sample<msgs::AimCommand, msgs::Header> &sample) {
              sample->control = false;
              sample.publish();
            });
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
            configs_.planner_conf.fail_polling_interval_sec * 1000)));
        continue; // 锁定的敌人已经丢失
      }
      auto gimbal_info = gimbal_listener_.getLatestInfo();
      auto cmd = planner_.plan(target_state, track_state.stamp_last_update,
                               gimbal_info);
      if (!cmd.control) {
        LOG_WARNING(logger_, "Aimcommand not control!");
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
            configs_.planner_conf.fail_polling_interval_sec * 1000)));
        continue; // 轨迹规划失败
      }
      aimcommand_pub_.loan()
          .and_then(
              [&](iox::popo::Sample<msgs::AimCommand, msgs::Header> &sample) {
                sample.getUserHeader().frame_id = {
                    iox::TruncateToCapacity, configs_.odom_frame_id.c_str()};
                sample.getUserHeader().stamp_ns = tools::getTimeNowNanoSec();
                sample->control = cmd.control;
                sample->fire_thres_yaw = cmd.fire_thres_yaw;
                sample->fire_thres_pitch = cmd.fire_thres_pitch;
                sample->target_yaw = cmd.target_yaw;
                sample->target_pitch = cmd.target_pitch;
                sample->yaw = cmd.yaw;
                sample->yaw_vel = cmd.yaw_vel;
                sample->yaw_acc = cmd.yaw_acc;
                sample->pitch = cmd.pitch;
                sample->pitch_vel = cmd.pitch_vel;
                sample->pitch_acc = cmd.pitch_acc;
                sample->bullet_id = cmd.bullet_id;
                sample.publish();
              })
          .or_else([this](auto) {
            LOG_WARNING(logger_, "Fail to publish AimCommand!");
          });
      // plot瞄准信息
      if (configs_.plot_info) {
        auto [roll, pitch, yaw, pitch_vel, yaw_vel, bullet_speed,
              receive_bullet_id] = gimbal_info;
        static auto last_receive_bullet_id{receive_bullet_id};
        plotter_.plot("fired", receive_bullet_id != last_receive_bullet_id);
        last_receive_bullet_id = receive_bullet_id;
        plotter_.plot("fire_thres_yaw",
                      tools::radian2Angle(cmd.fire_thres_yaw));
        plotter_.plot("fire_thres_pitch",
                      tools::radian2Angle(cmd.fire_thres_pitch));
        plotter_.plot("target_yaw", tools::radian2Angle(cmd.target_yaw));
        plotter_.plot("target_pitch", tools::radian2Angle(cmd.target_pitch));
        plotter_.plot("yaw", tools::radian2Angle(cmd.yaw));
        plotter_.plot("yaw_vel", tools::radian2Angle(cmd.yaw_vel));
        plotter_.plot("yaw_acc", tools::radian2Angle(cmd.yaw_acc));
        plotter_.plot("pitch", tools::radian2Angle(cmd.pitch));
        plotter_.plot("pitch_vel", tools::radian2Angle(cmd.pitch_vel));
        plotter_.plot("pitch_acc", cmd.pitch_acc);
        plotter_.plot("bullet_id", cmd.bullet_id);
        plotter_.plot("gimbal_roll", tools::radian2Angle(roll));
        plotter_.plot("gimbal_pitch", tools::radian2Angle(pitch));
        plotter_.plot("gimbal_yaw", tools::radian2Angle(yaw));
        plotter_.plot("gimbal_pitch_vel", tools::radian2Angle(pitch_vel));
        plotter_.plot("gimbal_yaw_vel", tools::radian2Angle(yaw_vel));
        plotter_.plot("bullet_speed", bullet_speed);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{
          static_cast<int>(configs_.planner_conf.dt_sec * 1000)});
    }
    LOG_INFO(logger_, "plan_thread stop!");
  }};
  // 启动可视化线程
  if (configs_.show_image)
    this->image_poller_ =
        std::make_unique<hardware::ImagePoller<msgs::Image1440x1080_8UC3>>(
            logger_, configs_.camera_name,
            [this](const cv::Mat &image, const std::string &frame_id,
                   const std::chrono::system_clock::time_point &stamp) {
              cv::Mat copy;
              image.copyTo(copy);
              if (auto aim_target = this->aiming_target_.load();
                  targets_.contains(aim_target)) {
                auto [target_state, track_state] =
                    targets_.at(aim_target)->getTargetTrackState();
                auto [aimed_armor, armor_index, predict_time] =
                    planner_.getAimingArmorIndexPredictTime(target_state);
                // HACK: 不应该放到这里
                plotter_.plot("select_index", static_cast<int>(armor_index));
                plotter_.plot("predict_time", predict_time);
                const auto &target = targets_.at(aim_target);
                // 绘制所有目标装甲板（红色）
                drawTarget(*target, copy, stamp);
                // 绘制正在瞄准的装甲板（绿色）
                drawArmor(aimed_armor, target_state.type, copy, stamp,
                          tools::Color::bgr::GREEN);
              }
              cv::imshow("tracker", copy);
              cv::waitKey(1);
            });
  LOG_INFO(logger_, "Tracker node initialization complete!");
}

void auto_aim::TrackerNode::onArmorsReceivedCallback(
    iox::popo::Subscriber<msgs::Armor, msgs::Header> *subscriber,
    TrackerNode *self) {
  std::vector<types::Armor> armors;
  // take all samples from the subscriber queue
  while (subscriber->take().and_then(
      [&armors, subscriber,
       self](const iox::popo::Sample<const msgs::Armor, const msgs::Header>
                 &sample) {
        if (subscriber->getServiceDescription().getInstanceIDString() ==
            self->armors_sub_.getServiceDescription().getInstanceIDString()) {
          types::Armor armor{sample};
          armors.emplace_back(armor);
        }
      })) {
  } // end of while
  // 排除连心跳都没有的假唤醒
  if (armors.empty())
    return;
  // 过滤掉心跳信号并按照光心距离排序，之后装甲板可能为空
  auto image_stamp = armors.front().stamp; // 假设一次接收的armors来自同一帧
  std::erase_if(armors, [](const types::Armor &a) { return a.heart_beat; });
  LOG_TRACE_L1(self->logger_, "receive {} valid armors.", armors.size());
  // 将坐标变换到odom系并抹除变换失败的装甲板
  std::erase_if(armors, [&](types::Armor &armor) {
    // 过滤掉非同一帧的装甲板
    if (self->configs_.erase_if_not_key_frame && !armor.key_frame)
      return true;
    if (armor.stamp != image_stamp)
      return true;
    try {
      Eigen::Isometry3d armor_pose_camera{Eigen::Isometry3d::Identity()};
      // 虽然PNP得到的是绝对坐标，但发布出来时已经是prerotate的了
      armor_pose_camera.pretranslate(armor.position);
      armor_pose_camera.rotate(armor.orientation.matrix());
      Eigen::Isometry3d T = self->tf_buffer_.get(
          self->configs_.odom_frame_id, armor.frame_id, armor.stamp,
          std::chrono::nanoseconds{static_cast<int64_t>(
              self->configs_.tf_query_tolerance_ms * 1e6)});
      Eigen::Isometry3d armor_pose_odom = T * armor_pose_camera;
      LOG_TRACE_L3(self->logger_, "armor position before tf: x{},y{},z{}",
                   armor.position.x(), armor.position.y(), armor.position.z());
      armor.position = armor_pose_odom.translation();
      armor.orientation =
          Eigen::Quaterniond{armor_pose_odom.rotation().matrix()};
      LOG_TRACE_L3(self->logger_, "armor position after tf: x{},y{},z{}",
                   armor.position.x(), armor.position.y(), armor.position.z());
      return false;
    } catch (const std::exception &e) {
      LOG_ERROR(self->logger_, "tf from {} to {} failed: {}", armor.frame_id,
                self->configs_.odom_frame_id, e.what());
      return true;
    }
  });
  std::ranges::sort(
      armors, [](const types::Armor &a, const types::Armor &b) -> bool {
        return a.distance_to_image_center < b.distance_to_image_center;
      });
  // 选择光心最近装甲板作为打击目标
  // XXX: 不合适，被操作手抱怨打团战时对敌人“雨露均沾”了QAQ
  if (!armors.empty()) {
    if (self->aiming_target_.load() != armors.front().type) {
      self->aiming_target_.store(armors.front().type);
      LOG_INFO(self->logger_, "Select target {}!",
               rfl::enum_to_string(armors.front().type));
    }
  }
  // NOTE:
  // 更新所有目标。track由图像时间戳驱动，图像到发射瞬间的补偿由planner完成
  bool all_targets_lost{true};
  for (auto &[type, target] : self->targets_) {
    auto track_state = target->track(armors, image_stamp);
    if (track_state != TrackState::State::LOST)
      all_targets_lost = false;
  }
  if (all_targets_lost)
    self->aiming_target_.store(types::ArmorType::Negative);

  // NOTE: 下面的都是发布调试波形的了
  if (!self->configs_.plot_info ||
      self->aiming_target_.load() == types::ArmorType::Negative)
    return;
  if (!armors.empty()) {
    const auto &armor = armors.front();
    auto armor_name = rfl::enum_to_string(armor.color) +
                      rfl::enum_to_string(armor.type) +
                      self->configs_.odom_frame_id;
    auto rpy = armor.getRpy();
    auto xyz = armor.position;
    self->plotter_.plot(armor_name + "Roll", tools::radian2Angle(rpy(0)));
    self->plotter_.plot(armor_name + "Pitch", tools::radian2Angle(rpy(1)));
    self->plotter_.plot(armor_name + "Yaw", tools::radian2Angle(rpy(2)));
    self->plotter_.plot(armor_name + "X", xyz.x());
    self->plotter_.plot(armor_name + "Y", xyz.y());
    self->plotter_.plot(armor_name + "Z", xyz.z());
  }
  auto aiming_target = self->aiming_target_.load();
  if (!self->targets_.contains(aiming_target))
    return;
  const auto &target_ptr = self->targets_.at(aiming_target);
  auto state = target_ptr->getTargetTrackState().first;
  auto type_name = rfl::enum_to_string(state.type);
  self->plotter_.plot(type_name + "x", state.center_position.x());
  self->plotter_.plot(type_name + "y", state.center_position.y());
  self->plotter_.plot(type_name + "z", state.center_position.z());
  self->plotter_.plot(type_name + "vx", state.center_velocity.x());
  self->plotter_.plot(type_name + "vy", state.center_velocity.y());
  self->plotter_.plot(type_name + "vz", state.center_velocity.z());
  self->plotter_.plot(type_name + "yaw", tools::radian2Angle(state.center_yaw));
  self->plotter_.plot(type_name + "vyaw",
                      tools::radian2Angle(state.center_vyaw));
  if (state.type == types::ArmorType::Outpost) {
    // TODO
  } else if (state.type != types::ArmorType::Base) {
    auto ra = target_ptr->get("ra");
    auto rb = target_ptr->get("rb");
    auto dz = target_ptr->get("dz");
    self->plotter_.plot(type_name + "r_a", ra);
    self->plotter_.plot(type_name + "r_b", rb);
    self->plotter_.plot(type_name + "dz", dz);
  }
}

void auto_aim::TrackerNode::drawArmor(
    const ArmorPositionYaw &armor, types::ArmorType type, cv::Mat &image,
    const std::chrono::system_clock::time_point &stamp,
    const cv::Scalar &color) const {
  try {
    Eigen::Isometry3d T_odom_to_camera =
        tf_buffer_.get(configs_.camera_frame_id, configs_.odom_frame_id, stamp,
                       std::chrono::nanoseconds{static_cast<int64_t>(
                           configs_.tf_query_tolerance_ms * 1e6)});
    Eigen::Isometry3d armor_pose_odom{Eigen::Isometry3d::Identity()};
    // NOTE: PNP得到的是绝对坐标，先平移
    armor_pose_odom.pretranslate(armor.position);
    double pitch =
        tools::angle2Radian(type == types::ArmorType::Outpost ? -15 : 15);
    double yaw = armor.yaw.theta();
    armor_pose_odom.rotate(tools::rpyToQuaterniond({0.0, pitch, yaw}));
    Eigen::Isometry3d armor_pose_in_camera = T_odom_to_camera * armor_pose_odom;
    Eigen::Matrix3d R_eg = armor_pose_in_camera.rotation();
    Eigen::Vector3d t_eg = armor_pose_in_camera.translation();
    cv::Mat R, rvec, tvec;
    cv::eigen2cv(R_eg, R);
    cv::Rodrigues(R, rvec);
    cv::eigen2cv(t_eg, tvec);
    std::vector<cv::Point2f> image_points;
    const auto &obj_points = types::points::getArmorPointsCV(type);
    cv::projectPoints(obj_points, rvec, tvec, camera_matrix_,
                      distortion_coefficients_, image_points);
    tools::drawPoints(image, image_points, color);
  } catch (const std::exception &e) {
    LOG_WARNING(logger_, "{}", e.what());
  }
}

void auto_aim::TrackerNode::drawTarget(
    const Target &target, cv::Mat &image,
    const std::chrono::system_clock::time_point &image_stamp) const {
  auto [target_state, track_state] = target.getTargetTrackState();
  auto dt = std::chrono::duration_cast<
                std::chrono::duration<double, std::chrono::seconds::period>>(
                image_stamp - track_state.stamp_last_update)
                .count();
  auto status_predict = target_state.predict(dt);
  // 绘制所有装甲板
  for (const auto &armor : status_predict.armors())
    drawArmor(armor, target_state.type, image, image_stamp);
}
