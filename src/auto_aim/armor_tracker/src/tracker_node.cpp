// Copyright (c) 2026 I dont have 30k. All Rights Reserved.
#include "tracker_node.hpp"
#include "basic/colors.hpp"
#include "basic/image_tools.hpp"
#include "configs.hpp"
#include "hardware/cam_info_listener.hpp"
#include "hardware/gimbal_info_listener.hpp"
#include "hardware/image_listener.hpp"
#include "hardware/task_mode_listener.hpp"
#include "math/angle_tools.hpp"
#include "msgs/AimCommand.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "msgs/Image.hpp"
#include "planner.hpp"
#include "target.hpp"
#include "trajectory.hpp"
#include "types.hpp"
#include "types/Armor.hpp"
#include "types/ArmorPoints.hpp"
#include "types/ArmorType.hpp"
#include "types/EnemyColor.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include "types/TaskMode.hpp"

#include "iceoryx_posh/internal/popo/base_subscriber.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iox/signal_watcher.hpp"
#include "opencv2/calib3d.hpp"
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
#include <mutex>
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
      planner_(logger_, configs_.planner_conf) {
  LOG_INFO(logger_, "start tracker node!");
  tf_listener_.init();
  for (auto type : std::array{
           types::ArmorType::One,
           types::ArmorType::Two,
           types::ArmorType::Three,
           types::ArmorType::Four,
           types::ArmorType::Sentry,
           types::ArmorType::Outpost,
           types::ArmorType::Base,
       }) {
    targets_.emplace(
        type, std::make_unique<Target>(logger_, configs_.target_conf, type));
    LOG_INFO(logger_, "add target {}.", rfl::enum_to_string(type));
  }
  armors_listener_
      .attachEvent(armors_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onArmorsReceivedCallback, *this))
      .or_else([this](auto) {
        LOG_CRITICAL(logger_, "unable to attach armors_sub");
        std::exit(EXIT_FAILURE);
      });
  this->plan_thread_ = std::jthread{[this]() {
    LOG_INFO(logger_, "plan_thread start!");
    while (!iox::hasTerminationRequested()) {
      if (!task_mode_listener_.isOnTask() ||
          !targets_.contains(selected_target_.load())) { // 排除Nigger
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
            configs_.planner_conf.fail_polling_interval_sec * 1000)));
        continue;
      }
      auto [status, stamp] =
          targets_.at(selected_target_.load())->getStatusStamp();
      auto bullet_speed = gimbal_listener_.getLatestInfo().bullet_speed;
      auto cmd = planner_.plan(status, stamp, bullet_speed);
      // NOTE: selected_target的更新由装甲板订阅回调驱动（选择光心最近）
      // 当不控制云台时（可能是track超时或弹道解算异常），放弃锁定这个目标
      if (!cmd.control)
        selected_target_.store(types::ArmorType::Negative);
      aimcommand_pub_.loan()
          .and_then(
              [&](iox::popo::Sample<msgs::AimCommand, msgs::Header> &sample) {
                sample.getUserHeader().frame_id = {
                    iox::TruncateToCapacity, configs_.odom_frame_id.c_str()};
                sample.getUserHeader().stamp_ns = tools::getTimeNowNanoSec();
                sample->control = cmd.control;
                sample->fire = cmd.fire;
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
            LOG_WARNING(logger_, "fail to publish aimcommand!");
          });
      // plot瞄准信息
      if (configs_.debug_mode && cmd.control) {
        plotter_.plot("target_yaw", cmd.target_yaw);
        plotter_.plot("target_pitch", cmd.target_pitch);
        plotter_.plot("yaw", cmd.yaw);
        plotter_.plot("yaw_vel", cmd.yaw_vel);
        plotter_.plot("yaw_acc", cmd.yaw_acc);
        plotter_.plot("pitch", cmd.pitch);
        plotter_.plot("pitch_vel", cmd.pitch_vel);
        plotter_.plot("pitch_acc", cmd.pitch_acc);
        plotter_.plot("bullet_id", cmd.bullet_id);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{
          static_cast<int>(configs_.planner_conf.dt_sec * 1000)});
    }
    LOG_INFO(logger_, "plan_thread stop!");
  }};
  if (configs_.debug_mode) {
    this->image_listener_ =
        std::make_unique<hardware::ImageListener<msgs::Image1440x1080_8UC3>>(
            logger_, configs_.camera_name,
            [this](const cv::Mat &image, const std::string &frame_id,
                   const std::chrono::system_clock::time_point &stamp) {
              std::scoped_lock lk{image_mtx_};
              image.copyTo(image_stamp_cache_.first);
              image_stamp_cache_.second = stamp;
            }); // NOTE: 需要快速释放sample避免阻塞detector
    auto camera_info =
        hardware::CameraInfoListener{logger_, configs_.camera_name}.get();
    this->camera_matrix_ = camera_info.camera_matrix.clone();
    this->distortion_coefficients_ =
        camera_info.distortion_coefficients.clone();
    if (configs_.publish_target_armors)
      this->armors_pub_ =
          std::make_unique<iox::popo::Publisher<msgs::Armor, msgs::Header>>(
              types::IceoryxServiceDescription{configs_.armors_pub_topic}
                  .description);
    this->image_show_thread_ = std::jthread{[this]() {
      LOG_INFO(logger_, "image_show_thread start!");
      while (!iox::hasTerminationRequested()) {
        cv::Mat image;
        std::chrono::system_clock::time_point image_stamp;
        {
          std::scoped_lock lk{image_mtx_};
          this->image_stamp_cache_.first.copyTo(image); // 避免锁住imagelistener
          image_stamp = image_stamp_cache_.second;
        }
        for (const auto &[type, target] : targets_) {
          // 绘制所有目标装甲板（红色）
          drawTarget(*target, image, image_stamp);
          // 发布在线的目标信息
          if (auto [status_opt, stamp] = target->getStatusStamp();
              status_opt.has_value()) {
            const auto &status = status_opt.value();
            auto type_name = rfl::enum_to_string(status.type);
            plotter_.plot(type_name + "x", status_opt->center_position.x());
            plotter_.plot(type_name + "y", status_opt->center_position.y());
            plotter_.plot(type_name + "z", status_opt->center_position.z());
            plotter_.plot(type_name + "vx", status_opt->center_velocity.x());
            plotter_.plot(type_name + "vy", status_opt->center_velocity.y());
            plotter_.plot(type_name + "vz", status_opt->center_velocity.z());
            plotter_.plot(type_name + "yaw", status_opt->center_yaw);
            plotter_.plot(type_name + "vyaw", status_opt->center_vyaw);
            if (status.type == types::ArmorType::Outpost) {
              plotter_.plot(type_name + "dz_a", status_opt->dz_a);
              plotter_.plot(type_name + "dz_b", status_opt->dz_b);
              plotter_.plot(type_name + "r", status_opt->radius);
            } else if (status.type != types::ArmorType::Base) {
              plotter_.plot(type_name + "r_a", status_opt->radius_a);
              plotter_.plot(type_name + "r_b", status_opt->radius_b);
              plotter_.plot(type_name + "dz", status_opt->dz);
            }
          }
        }
        if (auto aim_target = this->selected_target_.load();
            aim_target != types::ArmorType::Negative) {
          if (auto status_opt = targets_.at(aim_target)->getStatusStamp().first;
              status_opt.has_value()) {
            auto status_predict =
                status_opt.value().predict(planner_.aim0_predict_time_.load());
            auto aimed_armor =
                Trajectory::getClosestArmor(status_predict, 0, 0);
            // 绘制正在标准的装甲板（绿色）
            drawArmor(aimed_armor, image, image_stamp,
                      tools::Color::bgr::GREEN);
          }
        }
        if (configs_.show_image)
          cv::imshow("tracker", image);
        cv::waitKey(10); // 100fps
      } // end of while
      LOG_INFO(logger_, "image_show_thread stop!");
    }};
  }
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
        // 过滤掉自己发布的装甲板
        if (subscriber->getServiceDescription().getInstanceIDString() ==
            self->armors_sub_.getServiceDescription().getInstanceIDString()) {
          armors.emplace_back(sample);
        }
      })) {
  } // end of while
  if (armors.empty())
    return;
  auto image_stamp = armors.at(0).stamp; // 假设一次接收的armors来自同一帧
  // 过滤掉心跳信号并按照光心距离排序
  std::erase_if(armors,
                [](const Armor &armor) -> bool { return armor.heart_beat; });
  LOG_TRACE_L2(self->logger_, "recieve {} valid armor(s).", armors.size());
  std::sort(armors.begin(), armors.end(),
            [](const Armor &a, const Armor &b) -> bool {
              return a.distance_to_image_center < b.distance_to_image_center;
            });
  // 将坐标变换到odom系内并抹除变换失败的装甲板
  std::erase_if(armors, [&](types::Armor &armor) {
    try {
      Eigen::Isometry3d armor_pose_camera;
      // 注意PNP得到的是绝对坐标，需要先平移
      armor_pose_camera.pretranslate(armor.position);
      armor_pose_camera.rotate(armor.orientation);
      Eigen::Isometry3d T = self->tf_buffer_.get(
          self->configs_.odom_frame_id, armor.frame_id, armor.stamp,
          std::chrono::nanoseconds{static_cast<int64_t>(
              self->configs_.tf_query_tolerance_ms * 1e6)});
      Eigen::Isometry3d armor_pose_odom = T * armor_pose_camera;
      armor.position = armor_pose_odom.translation();
      armor.orientation =
          Eigen::Quaterniond{armor_pose_odom.rotation().matrix()};
      return false;
    } catch (const std::exception &e) {
      LOG_ERROR(self->logger_, "tf from {} to {} failed: {}", armor.frame_id,
                self->configs_.odom_frame_id, e.what());
      return true;
    }
  });
  if (!armors.empty()) // 更新瞄准目标
    self->selected_target_ = armors.at(0).type;
  // NOTE:
  // 更新所有目标。track由图像时间戳驱动，图像到现在的时间补偿由planner完成
  for (auto &[type, target] : self->targets_)
    target->track(armors, image_stamp);
}

void auto_aim::TrackerNode::drawArmor(
    const Armor &armor, cv::Mat &image,
    const std::chrono::system_clock::time_point &stamp, const cv::Scalar &color,
    bool publish_armor) const {
  try {
    Eigen::Isometry3d T_odom_to_camera =
        tf_buffer_.get(configs_.camera_frame_id, configs_.odom_frame_id, stamp,
                       std::chrono::nanoseconds{static_cast<int64_t>(
                           configs_.tf_query_tolerance_ms * 1e6)});
    Eigen::Isometry3d armor_pose_odom;
    // NOTE: PNP得到的是绝对坐标，先平移
    armor_pose_odom.pretranslate(armor.position);
    double pitch =
        tools::angle2Radian(armor.type == types::ArmorType::Outpost ? -15 : 15);
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
    const auto &obj_points = types::points::getArmorPointsCV(armor.type);
    cv::projectPoints(obj_points, rvec, tvec, camera_matrix_,
                      distortion_coefficients_, image_points);
    tools::drawPoints(image, image_points, color);
    if (publish_armor) {
      Eigen::Quaterniond q{armor_pose_odom.rotation().matrix()};
      armors_pub_->loan()
          .and_then([&](iox::popo::Sample<msgs::Armor, msgs::Header> &sample) {
            sample.getUserHeader().stamp_ns =
                tools::chronoPointToNanoSec(stamp);
            sample->armor_type = static_cast<int>(armor.type);
            sample->armor_color =
                static_cast<int>(types::EnemyColor::Extinguished);
            sample->position.x = armor.position.x();
            sample->position.y = armor.position.y();
            sample->position.z = armor.position.z();
            sample->orientation.x = q.x();
            sample->orientation.y = q.y();
            sample->orientation.z = q.z();
            sample->orientation.w = q.w();
            sample.publish();
          })
          .or_else([&](auto) {
            LOG_WARNING(logger_, "fail to publish armor message");
          });
    }
  } catch (const std::exception &e) {
    LOG_WARNING(logger_, "{}", e.what());
  }
}

void auto_aim::TrackerNode::drawTarget(
    const Target &target, cv::Mat &image,
    const std::chrono::system_clock::time_point &image_stamp) const {
  auto [status_opt, stamp] = target.getStatusStamp();
  if (!status_opt.has_value())
    return;
  auto dt = std::chrono::duration_cast<
                std::chrono::duration<double, std::chrono::seconds::period>>(
                image_stamp - stamp)
                .count();
  if (dt < 0)
    LOG_TRACE_L1(logger_, "stamp image is before stamp target {} last track.",
                 rfl::enum_to_string(target.type_));
  auto status_predict = status_opt.value().predict(dt);
  // 绘制所有装甲板
  for (const auto &armor : status_predict.armors())
    drawArmor(armor, image, image_stamp);
}
