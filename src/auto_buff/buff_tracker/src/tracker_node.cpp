#include "tracker_node.hpp"
#include "basic/colors.hpp"
#include "basic/image_tools.hpp"
#include "configs.hpp"
#include "hardware/cam_info_listener.hpp"
#include "hardware/gimbal_info_listener.hpp"
#include "hardware/task_mode_listener.hpp"
#include "math/angle_tools.hpp"
#include "msgs/BuffBlade.hpp"
#include "opencv2/core/types.hpp"
#include "targets.hpp"
#include "trajectory.hpp"
#include "types.hpp"
#include "types/IceoryxServiceDescription.hpp"
#include "types/TaskMode.hpp"

#include "opencv2/highgui.hpp"
#include "quill/LogMacros.h"
#include <opencv2/core/eigen.hpp>

#include <cmath>
#include <memory>
#include <optional>
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
      tf_listener_(logger_, tf_buffer_), gimbal_info_listener_(logger_),
      trajectory_(logger_, configs_.trajectory_conf) {
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
  this->big_buff_target_ =
      std::make_unique<BigBuffTarget>(logger_, configs_.big_buff_conf,
                                      camera_matrix_, distortion_coefficients_);
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
  this->plan_thread_ = std::jthread{[this]() {
    LOG_INFO(logger_, "plan_thread start!");
    while (!iox::hasTerminationRequested()) {
      auto on_small_buff_task =
          configs_.always_on_task_small_buff
              ? true
              : task_mode_listener_.isTask(types::TaskMode::SmallBuff);
      auto on_big_buff_task =
          configs_.always_on_task_big_buff
              ? true
              : task_mode_listener_.isTask(types::TaskMode::BigBuff);
      if (!(on_small_buff_task || on_big_buff_task)) { // 无锁定状态
        aimcommand_pub_.loan().and_then(
            [&](iox::popo::Sample<msgs::AimCommand, msgs::Header> &sample) {
              sample->control = false;
              sample.publish();
            });
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
            configs_.trajectory_conf.fail_polling_interval_sec * 1000)));
        continue;
      }

      BuffState target_state;
      TrackState track_state;
      if (on_small_buff_task)
        std::tie(target_state, track_state) =
            small_buff_target_->getTargetTrackState();
      if (on_big_buff_task)
        std::tie(target_state, track_state) =
            big_buff_target_->getTargetTrackState();
      if (track_state.state == TrackState::State::LOST) {
        aimcommand_pub_.loan().and_then(
            [&](iox::popo::Sample<msgs::AimCommand, msgs::Header> &sample) {
              sample->control = false;
              sample.publish();
            });
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
            configs_.trajectory_conf.fail_polling_interval_sec * 1000)));
        continue; // 没有需要打击的扇叶
      }

      auto gimbal_info = gimbal_info_listener_.getLatestInfo();
      auto cmd = solveAimCommand(target_state, track_state.stamp_last_update,
                                 gimbal_info);
      if (!cmd.control) {
        LOG_WARNING(logger_, "Aimcommand not control!");
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(
            configs_.trajectory_conf.fail_polling_interval_sec * 1000)));
        continue; // 弹道解算失败
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
        plotter_.plot("target_yaw_degree", tools::radian2Angle(cmd.target_yaw));
        plotter_.plot("target_pitch_degree",
                      tools::radian2Angle(cmd.target_pitch));
        plotter_.plot("gimbal_roll_degree", tools::radian2Angle(roll));
        plotter_.plot("gimbal_pitch_degree", tools::radian2Angle(pitch));
        plotter_.plot("gimbal_yaw_degree", tools::radian2Angle(yaw));
        plotter_.plot("gimbal_vpitch_rad", pitch_vel);
        plotter_.plot("gimbal_vyaw_rad", yaw_vel);
        plotter_.plot("bullet_speed", bullet_speed);
        plotter_.plot("buff_center_x", target_state.center_position.x());
        plotter_.plot("buff_center_y", target_state.center_position.y());
        plotter_.plot("buff_center_z", target_state.center_position.z());
        plotter_.plot("buff_roll", target_state.center_roll);
        if (on_small_buff_task)
          plotter_.plot("buff_vroll", small_buff_target_->get("vroll"));
        if (on_big_buff_task) {
          plotter_.plot("buff_a", big_buff_target_->get("a"));
          plotter_.plot("buff_omega", big_buff_target_->get("omega"));
          plotter_.plot("buff_b", big_buff_target_->get("b"));
          plotter_.plot("buff_c", big_buff_target_->get("c"));
          plotter_.plot("buff_d", big_buff_target_->get("d"));
          plotter_.plot("buff_vroll", big_buff_target_->get("vroll"));
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{
          static_cast<int>(configs_.cmd_pub_dt_sec * 1000)});
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
  if (self->task_mode_listener_.isTask(types::TaskMode::BigBuff) ||
      self->configs_.always_on_task_big_buff)
    self->big_buff_target_->track(blades, image_stamp, T_camera_to_odom);
  // NOTE: 开到一半的误识别在detector过滤过一遍了
  // XXX: 这里最好加上些发布调试信息的东西
}

msgs::AimCommand auto_buff::TrackerNode::solveAimCommand(
    const BuffState &target_state,
    const std::chrono::system_clock::time_point &target_stamp,
    const msgs::GimbalInfo &gimbal_info) {
  auto dt_update_to_now_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::system_clock::now() - target_stamp)
          .count() +
      configs_.predict_offset_ms / 1000.0;
  auto cmd_opt = trajectory_.solveBuff(
      target_state.predict(dt_update_to_now_sec), gimbal_info.bullet_speed);
  if (!cmd_opt.has_value()) {
    this->index_predict_time_cache_opt_.store(std::nullopt);
    return {.control = false};
  }
  this->index_predict_time_cache_opt_.store(std::optional{BuffIndexPredictTime{
      .index = cmd_opt->index,
      .predict_time = cmd_opt->fly_time + dt_update_to_now_sec,
  }});
  return {
      .control = true,
      .fire_thres_yaw = -1,
      .fire_thres_pitch = -1,
      .target_yaw = static_cast<float>(cmd_opt->yaw),
      .target_pitch = static_cast<float>(cmd_opt->pitch),
      .yaw = static_cast<float>(cmd_opt->yaw),
      .yaw_vel = 0,
      .yaw_acc = 0,
      .pitch = static_cast<float>(cmd_opt->pitch),
      .pitch_vel = 0,
      .pitch_acc = 0,
      .bullet_id = 0,
  };
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
  cv::projectPoints(std::vector{BUFF_BLADE_HIT_OBJ_POINT}, rvec, tvec,
                    camera_matrix_, distortion_coefficients_, image_points);
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
  auto on_small_buff_mode =
      task_mode_listener_.isTask(types::TaskMode::SmallBuff) ||
      configs_.always_on_task_small_buff;
  auto on_big_buff_mode =
      task_mode_listener_.isTask(types::TaskMode::BigBuff) ||
      configs_.always_on_task_big_buff;
  if (on_big_buff_mode)
    drawBuff(*big_buff_target_, copy, stamp, T_odom_to_camera);
  if (on_small_buff_mode)
    drawBuff(*small_buff_target_, copy, stamp, T_odom_to_camera);
  if (auto idx_time_opt = index_predict_time_cache_opt_.load();
      idx_time_opt.has_value()) {
    auto [index, predict_time] = idx_time_opt.value();
    if (on_big_buff_mode)
      drawBuffBlade(big_buff_target_->getTargetTrackState()
                        .first.predict(predict_time)
                        .blades()
                        .at(static_cast<int>(index)),
                    copy, T_odom_to_camera, tools::Color::bgr::GREEN);
    if (on_small_buff_mode)
      drawBuffBlade(small_buff_target_->getTargetTrackState()
                        .first.predict(predict_time)
                        .blades()
                        .at(static_cast<int>(index)),
                    copy, T_odom_to_camera, tools::Color::bgr::GREEN);
  }
  cv::imshow("buff_tracker", copy);
  cv::waitKey(1);
}
