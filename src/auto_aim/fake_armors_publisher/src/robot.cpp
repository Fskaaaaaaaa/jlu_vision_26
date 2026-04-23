// Copyright (c) 2026 Caonima. All Rights Reserved.
#include "robot.hpp"
#include "basic/time_tools.hpp"
#include "configs.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "iox/signal_watcher.hpp"
#include "math/angle_tools.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "quill/LogMacros.h"
#include "quill/core/ThreadContextManager.h"
#include "types/ArmorPoints.hpp"

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <numbers>
#include <thread>
#include <tuple>
#include <vector>

auto_aim::Robot::Robot(quill::Logger *logger, const RobotConfig &config,
                       RobotContrl &contrl)
    : logger_(logger), config_(config),
      armor_pub_({{iox::TruncateToCapacity,
                   config_.pub_conf.service_instance_event.at(0).c_str()},
                  {iox::TruncateToCapacity,
                   config_.pub_conf.service_instance_event.at(1).c_str()},
                  {iox::TruncateToCapacity,
                   config_.pub_conf.service_instance_event.at(2).c_str()}}),
      tf_listener_(logger_, tf_buffer_),
      cam_info_listener_(logger_, config_.camera_name),
      view_width_px_(0), view_height_px_(0),
      atom_contrl_(contrl) {
  LOG_INFO(logger_, "start robot.");
  if (!tf_listener_.init()) {
    LOG_CRITICAL(logger_, "failed to initialize tf listener.");
    std::exit(EXIT_FAILURE);
  }
  auto camera_info = cam_info_listener_.get();
  camera_matrix_ = camera_info.camera_matrix.clone();
  distortion_coefficients_ = camera_info.distortion_coefficients.clone();
  view_width_px_ = camera_info.view_width_px;
  view_height_px_ = camera_info.view_height_px;
  LOG_INFO(logger_, "camera info loaded, view={}x{}.", view_width_px_,
           view_height_px_);
  resetStatus();
  this->physic_pub_thread_ = std::jthread{[&]() {
    LOG_DEBUG(logger_, "physic_pub_thread start.");
    while (!iox::hasTerminationRequested()) {
      this->updateContrlPhysic();
      this->publishArmors();
      std::this_thread::sleep_for(
          std::chrono::milliseconds(config_.time_step_ms));
    }
    LOG_DEBUG(logger_, "physic_pub_thread stop.");
  }};
}

std::optional<std::array<cv::Point2d, 4>> auto_aim::Robot::getArmorCornersInImageOpt(
    const msgs::Armor &armor, const std::chrono::system_clock::time_point &stamp,
    Eigen::Isometry3d &pose_camera) {
  try {
    const Eigen::Isometry3d T_odom_to_camera = tf_buffer_.get(
        config_.camera_frame_id, config_.odom_frame_id, stamp,
        std::chrono::nanoseconds{
            static_cast<int64_t>(config_.tf_query_tolerance_ms * 1e6)});

    Eigen::Isometry3d armor_pose_odom{Eigen::Isometry3d::Identity()};
    armor_pose_odom.pretranslate(
        Eigen::Vector3d{armor.position.x, armor.position.y, armor.position.z});
    armor_pose_odom.rotate(Eigen::Quaterniond{armor.orientation.w,
                                              armor.orientation.x,
                                              armor.orientation.y,
                                              armor.orientation.z});
    const Eigen::Isometry3d armor_pose_camera = T_odom_to_camera * armor_pose_odom;
    pose_camera = armor_pose_camera;

    const auto &armor_points = types::points::getArmorPointsEG(config_.armor_type);
    for (const auto &p : armor_points) {
      const auto p_in_camera = armor_pose_camera * p;
      if (!std::isfinite(p_in_camera.x()) || !std::isfinite(p_in_camera.y()) ||
          !std::isfinite(p_in_camera.z()) || p_in_camera.z() <= 1e-6) {
        return std::nullopt;
      }
    }

    cv::Mat R, rvec, tvec;
    const Eigen::Matrix3d R_eg = armor_pose_camera.rotation();
    const Eigen::Vector3d t_eg = armor_pose_camera.translation();
    cv::eigen2cv(R_eg, R);
    cv::Rodrigues(R, rvec);
    cv::eigen2cv(t_eg, tvec);

    std::vector<cv::Point2f> image_points;
    cv::projectPoints(types::points::getArmorPointsCV(config_.armor_type), rvec,
                      tvec, camera_matrix_, distortion_coefficients_,
                      image_points);
    if (image_points.size() != 4) {
      return std::nullopt;
    }

    std::array<cv::Point2d, 4> points_in_image{};
    for (size_t i = 0; i < image_points.size(); ++i) {
      const auto &pt = image_points.at(i);
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
        return std::nullopt;
      }
      if (pt.x < 0 || pt.x >= static_cast<float>(view_width_px_) || pt.y < 0 ||
          pt.y >= static_cast<float>(view_height_px_)) {
        return std::nullopt;
      }
      points_in_image.at(i) = {pt.x, pt.y};
    }
    return points_in_image;
  } catch (const std::exception &e) {
    LOG_TRACE_L1(logger_, "failed to project armor corners: {}", e.what());
    return std::nullopt;
  }
}

std::vector<msgs::Armor> auto_aim::Robot::getArmorMsgsFromStatus(
    const std::chrono::system_clock::time_point &stamp) {
  RobotStatus status_snapshot;
  {
    std::scoped_lock lk{status_mtx_};
    status_snapshot = status_;
  }
  std::vector<msgs::Armor> armors_odom;
  for (auto &&i : std::array{0, 1, 2, 3}) {
    auto [r, dz] = i % 2 == 0 ? std::pair{config_.r1, 0.}
                              : std::pair{config_.r2, config_.dz};
    auto armor_yaw =
        tools::limitRadian(status_snapshot.yaw + i * std::numbers::pi / 2.);
    LOG_TRACE_L1(logger_, "armor yaw {}", armor_yaw);
    Eigen::Quaterniond rot{
        tools::rpyToQuaterniond({0, tools::angle2Radian(-15), armor_yaw})};
    armors_odom.emplace_back(
        msgs::Armor{.armor_type = static_cast<int>(config_.armor_type),
                    .armor_color = static_cast<int>(config_.armor_color),
                    .distance_to_image_center = status_snapshot.position.norm(),
                    .position =
                        {
                            .x = status_snapshot.position.x() +
                                 std::cos(armor_yaw) * r,
                            .y = status_snapshot.position.y() +
                                 std::sin(armor_yaw) * r,
                            .z = dz,
                        },
                    .orientation = {
                        .x = rot.x(),
                        .y = rot.y(),
                        .z = rot.z(),
                        .w = rot.w(),
                    }});
  }
  if (config_.hidden_invisible_armors) {
    std::ranges::sort(armors_odom, {}, [](const msgs::Armor &a) {
      return a.position.x * a.position.x + a.position.y * a.position.y;
    });
    Eigen::Vector2d view_dir{1.0, 0.0};
    if (status_snapshot.position.norm() > 1e-8) {
      view_dir = status_snapshot.position.normalized();
    }
    auto is_side_facing = [&](const msgs::Armor &armor) {
      double yaw = Eigen::Quaterniond{armor.orientation.w, armor.orientation.x,
                                      armor.orientation.y, armor.orientation.z}
                       .matrix()
                       .eulerAngles(0, 1, 2)(2);
      Eigen::Vector2d normal{std::cos(yaw), std::sin(yaw)};
      double cos_incidence = std::abs(view_dir.dot(normal));
      return cos_incidence < config_.facing_armor_cos_incidence;
    };
    if (armors_odom.size() >= 2) {
      if (is_side_facing(armors_odom[1])) {
        armors_odom.erase(armors_odom.begin() + 1, armors_odom.end());
      } else if (armors_odom.size() > 2) {
        armors_odom.erase(armors_odom.begin() + 2, armors_odom.end());
      }
    }
  }

  std::vector<msgs::Armor> visible_armors;
  const double cx = camera_matrix_.at<double>(0, 2);
  const double cy = camera_matrix_.at<double>(1, 2);
  for (auto &armor : armors_odom) {
    Eigen::Isometry3d pose_camera{Eigen::Isometry3d::Identity()};
    auto corners_opt = getArmorCornersInImageOpt(armor, stamp, pose_camera);
    if (!corners_opt.has_value()) {
      continue;
    }
    const Eigen::Quaterniond q_camera{pose_camera.rotation()};
    armor.position = {.x = pose_camera.translation().x(),
                      .y = pose_camera.translation().y(),
                      .z = pose_camera.translation().z()};
    armor.orientation = {.x = q_camera.x(),
                         .y = q_camera.y(),
                         .z = q_camera.z(),
                         .w = q_camera.w()};
    const auto &corners = corners_opt.value();
    armor.left_light.bottom = {.x = corners.at(0).x, .y = corners.at(0).y};
    armor.left_light.top = {.x = corners.at(1).x, .y = corners.at(1).y};
    armor.right_light.top = {.x = corners.at(2).x, .y = corners.at(2).y};
    armor.right_light.bottom = {.x = corners.at(3).x, .y = corners.at(3).y};
    const auto center_x = (corners.at(0).x + corners.at(1).x + corners.at(2).x +
                           corners.at(3).x) /
                          4.0;
    const auto center_y = (corners.at(0).y + corners.at(1).y + corners.at(2).y +
                           corners.at(3).y) /
                          4.0;
    armor.distance_to_image_center = std::hypot(center_x - cx, center_y - cy);
    armor.confidence = 1.0F;
    visible_armors.emplace_back(armor);
  }
  return visible_armors;
}

void auto_aim::Robot::publishArmors() {
  if (atom_contrl_.hide_armors.load()) {
    armor_pub_.loan()
        .and_then([&](iox::popo::Sample<msgs::Armor, msgs::Header> &sample) {
          sample.getUserHeader().frame_id = {
              iox::TruncateToCapacity, config_.camera_frame_id.c_str()};
          sample.getUserHeader().stamp_ns = tools::getTimeNowNanoSec();
          sample->heart_beat = true;
          sample.publish();
          LOG_DEBUG(logger_, "heart_beat published");
        })
        .or_else([&](auto) { LOG_ERROR(logger_, "loan shm failure!"); });
    return;
  }
  const auto now_point = std::chrono::system_clock::now();
  const auto now = tools::chronoPointToNanoSec(now_point);
  auto armors = this->getArmorMsgsFromStatus(now_point);
  for (auto &&armor : armors) {
    armor_pub_.loan()
        .and_then([&](iox::popo::Sample<msgs::Armor, msgs::Header> &sample) {
          sample.getUserHeader().frame_id = {
              iox::TruncateToCapacity, config_.camera_frame_id.c_str()};
          sample.getUserHeader().stamp_ns = now;
          sample->armor_color = armor.armor_color;
          sample->armor_type = armor.armor_type;
          sample->distance_to_image_center = armor.distance_to_image_center;
          sample->position = armor.position;
          sample->orientation = armor.orientation;
          sample->left_light = armor.left_light;
          sample->right_light = armor.right_light;
          sample->confidence = armor.confidence;
          sample->heart_beat = false;
          sample->key_frame = true;
          sample.publish();
        })
        .or_else([&](auto) { LOG_ERROR(logger_, "loan shm failure!"); });
  }
  LOG_DEBUG(logger_, "{} armor(s) published", armors.size());
  return;
}

void auto_aim::Robot::resetStatus() {
  std::scoped_lock lk{status_mtx_};
  this->status_.position = Eigen::Vector2d::Zero();
  this->status_.linear_velocity = Eigen::Vector2d::Zero();
  this->status_.linear_acceleration = Eigen::Vector2d::Zero();
  this->status_.yaw = 0;
  this->status_.vel_yaw = 0;
  this->status_.acc_yaw = 0;
  this->last_update_stamp_ = std::chrono::system_clock::now();
  LOG_DEBUG(logger_, "robot status reset.");
  return;
}

void auto_aim::Robot::updateContrlPhysic() {
  std::scoped_lock lk{status_mtx_};
  auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::system_clock::now() - last_update_stamp_)
                .count();
  this->last_update_stamp_ = std::chrono::system_clock::now();
  auto [direction, spin, const_speed_spin] = std::tuple{
      atom_contrl_.traction_direction.load(), atom_contrl_.spin_status.load(),
      atom_contrl_.const_speed_spin.load()};
  auto [ap, bp] = std::pair{
      std::fabs(direction.x + direction.y) >= 1e-8
          ? (Eigen::Vector2d{direction.x, direction.y}.normalized() *
             config_.power_linear_acceleration)
                .eval()
          : Eigen::Vector2d::Zero().eval(),
      spin.has_value() ? 2.0 * (static_cast<double>(spin.value()) - 0.5) *
                             config_.power_angular_acceleration
                       : 0.};
  auto [af, bf] =
      std::pair{std::fabs(status_.linear_velocity.norm()) >= 1e-8
                    ? (-status_.linear_velocity.normalized() *
                       config_.drag_linear_acceleration)
                          .eval()
                    : Eigen::Vector2d::Zero().eval(),
                std::fabs(status_.vel_yaw) > 1e-8
                    ? -status_.vel_yaw / std::fabs(status_.vel_yaw) *
                          config_.drag_angular_acceleration
                    : 0.};
  if (status_.linear_velocity.norm() <= 1e-1)
    status_.linear_velocity = Eigen::Vector2d::Zero();
  if (std::abs(status_.vel_yaw) <= 0.06)
    status_.vel_yaw = 0;

  status_.linear_acceleration = ap + af;
  status_.position = status_.position + status_.linear_velocity * dt +
                     0.5 * status_.linear_acceleration * std::pow(dt, 2);
  status_.linear_velocity =
      status_.linear_velocity + status_.linear_acceleration * dt;
  status_.acc_yaw = const_speed_spin ? 0 : bp + bf;
  status_.yaw = status_.yaw + status_.vel_yaw * dt +
                0.5 * status_.acc_yaw * std::pow(dt, 2);
  status_.vel_yaw = status_.vel_yaw + status_.acc_yaw * dt;
  LOG_TRACE_L1(logger_, "status updated.");
  return;
}

auto_aim::RobotStatus auto_aim::Robot::getState() {
  std::scoped_lock lk{status_mtx_};
  return status_;
};
