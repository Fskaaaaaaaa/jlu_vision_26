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
#include "types/EnemyColor.hpp"
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
#include <limits>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
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
      aiming_target_(types::ArmorType::Negative), hit_target_(false),
      planner_(logger_, configs_.planner_conf) {
  LOG_INFO(logger_, "start tracker node!");
  // 初始化畸变内参和坐标变换
  tf_listener_.init();
  auto camera_info =
      hardware::CameraInfoListener{logger_, configs_.camera_name}.get();
  this->camera_matrix_ = camera_info.camera_matrix.clone();
  this->distortion_coefficients_ = camera_info.distortion_coefficients.clone();
  // 添加目标
  for (auto target_type : std::array{
           types::ArmorType::One, types::ArmorType::Two,
           types::ArmorType::Three, types::ArmorType::Four,
           types::ArmorType::Sentry,
           types::ArmorType::Base, // HACK: 懒得写基地了，当机器人打吧
       }) {
    targets_.emplace(target_type,
                     std::make_unique<RobotTarget>(logger_, configs_.robot_conf,
                                                   target_type, camera_matrix_,
                                                   distortion_coefficients_));
    LOG_INFO(logger_, "add target {}.", rfl::enum_to_string(target_type));
  }
  targets_.emplace(types::ArmorType::Outpost,
                   std::make_unique<OutpostTarget>(
                       logger_, configs_.outpost_conf, camera_matrix_,
                       distortion_coefficients_));
  LOG_INFO(logger_, "add target {}.",
           rfl::enum_to_string(types::ArmorType::Outpost));
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
      bool switch_to_on_task;
      {
        static bool last_call_on_task{false};
        switch_to_on_task = on_task && !last_call_on_task;
        last_call_on_task = on_task;
      }
      auto aiming_target_state_opt = selectAimingTarget(switch_to_on_task);
      this->aiming_target_.store(aiming_target_state_opt.has_value()
                                     ? aiming_target_state_opt->first.type
                                     : types::ArmorType::Negative);
      if (!on_task || !aiming_target_state_opt.has_value()) {
        aimcommand_pub_.loan().and_then(
            [&](iox::popo::Sample<msgs::AimCommand, msgs::Header> &sample) {
              sample->control = false;
              sample.publish();
            });
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
            configs_.planner_conf.fail_polling_interval_sec * 1000)));
        continue; // 非自瞄模式或无锁定状态
      }
      auto gimbal_info = gimbal_listener_.getLatestInfo();
      auto cmd = planner_.plan(aiming_target_state_opt->first,
                               aiming_target_state_opt->second, gimbal_info);
      // TODO: this->hit_target_.load()
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
                auto [aimed_armor, armor_index, predict_time, fire_thres_yaw,
                      fire_thres_pitch] =
                    planner_.getAimingArmorIndexPredictTimeFireThres(
                        target_state);
                const auto &target = targets_.at(aim_target);
                // 绘制所有目标装甲板（红色）
                drawTarget(*target, copy, stamp);
                // 绘制正在瞄准的装甲板（绿色）
                drawArmor(aimed_armor, target_state.type, copy, stamp,
                          tools::Color::bgr::GREEN);
                drawCrosshair(copy, fire_thres_yaw, fire_thres_pitch);
                std::stringstream predict_time_ss;
                predict_time_ss << "PredictTime: " << std::fixed
                                << std::setprecision(2) << predict_time * 1000
                                << "ms";
                cv::putText(copy, predict_time_ss.str(), cv::Point(10, 90),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0,
                            tools::Color::bgr::GREEN, 2);
              }
              cv::imshow("tracker", copy);
              cv::waitKey(1);
            });
  LOG_INFO(logger_, "Tracker node initialization complete!");
}

std::optional<double> auto_aim::TrackerNode::getPixelDistanceToImageCenter(
    const Eigen::Vector3d &odom_position,
    const std::chrono::system_clock::time_point &stamp) const {
  try {
    Eigen::Isometry3d T_odom_to_camera =
        tf_buffer_.get(configs_.camera_frame_id, configs_.odom_frame_id, stamp,
                       std::chrono::nanoseconds{static_cast<int64_t>(
                           configs_.tf_query_tolerance_ms * 1e6)});
    Eigen::Isometry3d pose_odom{Eigen::Isometry3d::Identity()};
    pose_odom.pretranslate(odom_position);
    Eigen::Vector3d position_camera =
        (T_odom_to_camera * pose_odom).translation();
    if (position_camera.z() < 0)
      return std::nullopt;
    cv::Point3d obj_point{position_camera.x(), position_camera.y(),
                          position_camera.z()};
    // 已经转换到相机系了，外参为单位阵
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    std::vector<cv::Point2d> image_points;
    cv::projectPoints(std::vector{obj_point}, rvec, tvec, camera_matrix_,
                      distortion_coefficients_, image_points);
    auto u = image_points.front().x;
    auto v = image_points.front().y;
    // 投影到图像外边了
    if (u < 0 || u > msgs::Image1440x1080_8UC3::cols || v < 0 ||
        v > msgs::Image1440x1080_8UC3::rows)
      return std::nullopt;
    // 从投影点计算到像素距离
    auto cx = camera_matrix_.at<double>(0, 2);
    auto cy = camera_matrix_.at<double>(1, 2);
    auto dx = image_points.front().x - cx;
    auto dy = image_points.front().y - cy;
    return std::hypot(dx, dy);
  } catch (const std::exception &e) {
    LOG_WARNING(logger_, "{}", e.what());
    return std::nullopt;
  }
}

std::optional<
    std::pair<auto_aim::TargetState, std::chrono::system_clock::time_point>>
auto_aim::TrackerNode::selectAimingTarget(bool reselect) const {
  // 筛选所有可瞄准目标
  std::vector<std::pair<TargetState, TrackState>> targetable_targets;
  for (const auto &[type, target] : this->targets_)
    if (auto state = target->getTargetTrackState();
        state.second.state != TrackState::State::LOST)
      targetable_targets.emplace_back(state);
  static std::optional<types::ArmorType> last_select_type{std::nullopt};
  // 不重新锁定时尝试保持上一帧的锁定，否则跌入重新选择锁定的逻辑，锁定离画面中心最近的
  if (!reselect && last_select_type.has_value())
    for (const auto [target_state, track_state] : targetable_targets)
      if (last_select_type.value() == target_state.type)
        return {{target_state, track_state.stamp_last_update}};
  auto min_px_distance{std::numeric_limits<double>::max()};
  std::optional<std::pair<TargetState, std::chrono::system_clock::time_point>>
      selected_state{std::nullopt};
  for (const auto &[target_state, track_state] : targetable_targets)
    if (auto distance_opt = getPixelDistanceToImageCenter(
            target_state.center_position, track_state.stamp_last_update);
        distance_opt.has_value() && distance_opt.value() < min_px_distance) {
      min_px_distance = distance_opt.value();
      selected_state = {target_state, track_state.stamp_last_update};
    }
  // 此时若有值一定是重新选择的
  if (selected_state.has_value())
    LOG_INFO(logger_, "Select Target {}",
             rfl::enum_to_string(selected_state->first.type));
  last_select_type = selected_state.has_value()
                         ? std::optional{selected_state->first.type}
                         : std::nullopt;
  return selected_state;
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
  // 由于frame_id从相机的配置文件读取，依赖图像回调获得，故心跳帧只有时间戳无坐标系
  auto image_stamp = armors.front().stamp; // 假设一次接收的armors来自同一帧
  bool all_armors_not_hit{true};
  std::erase_if(armors, [&](const types::Armor &armor) {
    if (armor.heart_beat)
      return true;
    if (armor.type == self->aiming_target_.load() &&
        armor.color == types::EnemyColor::Extinguished) {
      self->hit_target_.store(true);
      all_armors_not_hit = false;
    }
    if (self->configs_.erase_if_not_key_frame && !armor.key_frame)
      return true;
    if (armor.stamp != image_stamp)
      return true;
    return false;
  }); // 之后armors可能为空
  if (all_armors_not_hit)
    self->hit_target_.store(false);
  LOG_TRACE_L1(self->logger_, "receive {} valid armors.", armors.size());
  // 查找该帧的坐标变换
  Eigen::Isometry3d T_camera_to_odom;
  if (!armors.empty())
    try {
      T_camera_to_odom = self->tf_buffer_.get(
          self->configs_.odom_frame_id, armors.front().frame_id,
          armors.front().stamp,
          std::chrono::nanoseconds{static_cast<int64_t>(
              self->configs_.tf_query_tolerance_ms * 1e6)});
    } catch (const std::exception &e) {
      LOG_ERROR(self->logger_, "tf from {} to {} failed: {}",
                armors.front().frame_id, self->configs_.odom_frame_id,
                e.what());
      armors.clear(); // 失败舍弃该帧接收的装甲板
    }
  // 更新所有目标。track由图像时间戳驱动，图像到发射瞬间的补偿由planner完成
  for (auto &[type, target] : self->targets_)
    auto track_state = target->track(armors, image_stamp, T_camera_to_odom);

  // NOTE: 下面的都是发布调试波形的了
  if (!self->configs_.plot_info)
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
  if (!self->targets_.contains(aiming_target)) // Negative
    return;
  // 发布正在瞄准目标的观测器状态
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
    auto r = target_ptr->get("r");
    auto dz0 = target_ptr->get("dz0");
    auto dz1 = target_ptr->get("dz1");
    auto dz2 = target_ptr->get("dz2");
    self->plotter_.plot(type_name + "r", r);
    self->plotter_.plot(type_name + "dz_0", dz0);
    self->plotter_.plot(type_name + "dz_1", dz1);
    self->plotter_.plot(type_name + "dz_2", dz2);
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
    const std::chrono::system_clock::time_point &stamp, const cv::Scalar &color,
    const std::string &txt) const {
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
    cv::putText(image, txt, image_points.front(), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                color, 2);
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
  int armor_index{0};
  for (const auto &armor : status_predict.armors())
    drawArmor(armor, target_state.type, image, image_stamp,
              tools::Color::bgr::RED, std::to_string(armor_index++));
}

void auto_aim::TrackerNode::drawCrosshair(cv::Mat &image, double yaw_fire_thres,
                                          double pitch_fire_thres) const {
  auto fx = camera_matrix_.at<double>(0, 0);
  auto fy = camera_matrix_.at<double>(1, 1);
  auto cx = camera_matrix_.at<double>(0, 2);
  auto cy = camera_matrix_.at<double>(1, 2);
  // 用角度偏置在归一化平面上的投影近似准星中心偏移
  auto crosshair_u = cx + fx * std::tan(configs_.planner_conf.yaw_offset);
  auto crosshair_v = cy - fy * std::tan(configs_.planner_conf.pitch_offset);
  if (!std::isfinite(crosshair_u) || !std::isfinite(crosshair_v)) {
    crosshair_u = static_cast<double>(image.cols) / 2.0;
    crosshair_v = static_cast<double>(image.rows) / 2.0;
  }
  auto crosshair_x = std::clamp(static_cast<int>(std::lround(crosshair_u)), 0,
                                std::max(0, image.cols - 1));
  auto crosshair_y = std::clamp(static_cast<int>(std::lround(crosshair_v)), 0,
                                std::max(0, image.rows - 1));
  cv::Point2i crosshair_center{crosshair_x, crosshair_y};
  constexpr auto fire_thres{0.005};
  constexpr auto basic_half_px{10};
  auto yaw_ratio = std::abs(yaw_fire_thres / fire_thres);
  auto pitch_ratio = std::abs(pitch_fire_thres / fire_thres);
  auto yaw_half_px =
      std::max(1, static_cast<int>(std::lround(basic_half_px * yaw_ratio)));
  auto pitch_half_px =
      std::max(1, static_cast<int>(std::lround(basic_half_px * pitch_ratio)));
  cv::ellipse(image, crosshair_center, cv::Size{yaw_half_px, pitch_half_px}, 0,
              0, 360, tools::Color::bgr::YELLOW);
  cv::line(image, crosshair_center + cv::Point2i{0, -pitch_half_px},
           crosshair_center + cv::Point2i{0, pitch_half_px},
           tools::Color::bgr::PURPLE);
  cv::line(image, crosshair_center + cv::Point2i{-yaw_half_px, 0},
           crosshair_center + cv::Point2i{yaw_half_px, 0},
           tools::Color::bgr::PURPLE);
  std::stringstream yaw_ss, pitch_ss;
  yaw_ss << std::fixed << std::setprecision(2)
         << tools::radian2Angle(yaw_fire_thres);
  pitch_ss << std::fixed << std::setprecision(2)
           << tools::radian2Angle(pitch_fire_thres);
  cv::putText(image, yaw_ss.str(),
              crosshair_center + cv::Point2i{yaw_half_px, 0},
              cv::FONT_HERSHEY_SIMPLEX, 1.0, tools::Color::bgr::GREEN, 2);
  cv::putText(image, pitch_ss.str(),
              crosshair_center + cv::Point2i{0, yaw_half_px},
              cv::FONT_HERSHEY_SIMPLEX, 1.0, tools::Color::bgr::GREEN, 2);
}
