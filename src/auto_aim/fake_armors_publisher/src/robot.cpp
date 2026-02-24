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
#include "types/Basic.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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
      atom_contrl_(contrl) {
  LOG_INFO(logger_, "start robot.");
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
    // BUG: 资源释放有问题
  }};
}

std::vector<msgs::Armor> auto_aim::Robot::getArmorMsgsFromStatus() {
  std::scoped_lock lk{status_mtx_};
  std::vector<msgs::Armor> armors;
  for (auto &&i : std::array{0, 1, 2, 3}) {
    auto [r, dz] = i % 2 == 0 ? std::pair{config_.r1, 0.}
                              : std::pair{config_.r2, config_.dz};
    auto armor_yaw =
        tools::limitRadian(status_.yaw + i * std::numbers::pi / 2.);
    LOG_TRACE_L1(logger_, "armor yaw {}", armor_yaw);
    Eigen::Quaterniond rot{
        tools::rpyToQuaterniond({0, tools::angle2Radian(-15), armor_yaw})};
    armors.emplace_back(
        msgs::Armor{.armor_type = static_cast<int>(config_.armor_type),
                    .armor_color = static_cast<int>(config_.armor_color),
                    .distance_to_image_center = 0,
                    .position =
                        {
                            .x = status_.position.x() + std::cos(armor_yaw) * r,
                            .y = status_.position.y() + std::sin(armor_yaw) * r,
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
    std::ranges::sort(armors, {}, [](const msgs::Armor &a) {
      return a.position.x * a.position.x + a.position.y * a.position.y;
    });
    Eigen::Vector2d view_dir = status_.position.head<2>().normalized();
    auto is_side_facing = [&](const msgs::Armor &armor) {
      double yaw = Eigen::Quaterniond{armor.orientation.w, armor.orientation.x,
                                      armor.orientation.y, armor.orientation.z}
                       .matrix()
                       .eulerAngles(0, 1, 2)(2);
      Eigen::Vector2d normal{std::cos(yaw), std::sin(yaw)};
      double cos_incidence = std::abs(view_dir.dot(normal));
      return cos_incidence < config_.facing_armor_cos_incidence;
    };
    if (is_side_facing(armors[1])) {
      armors.erase(armors.begin() + 1, armors.end());
    } else {
      armors.erase(armors.begin() + 2, armors.end());
    }
  }
  return armors;
}

void auto_aim::Robot::publishArmors() {
  if (atom_contrl_.hide_armors.load())
    return;
  auto armors = this->getArmorMsgsFromStatus();
  for (auto &&armor : armors) {
    armor_pub_.loan()
        .and_then([&](iox::popo::Sample<msgs::Armor, msgs::Header> &sample) {
          sample.getUserHeader().frame_id = {iox::TruncateToCapacity,
                                             config_.pub_conf.frame_id.c_str()};
          sample.getUserHeader().stamp_ns = tools::getTimeNowNanoSec();
          sample->armor_color = armor.armor_color;
          sample->armor_type = armor.armor_type;
          sample->distance_to_image_center = armor.distance_to_image_center;
          sample->position = armor.position;
          sample->orientation = armor.orientation;
          sample.publish();
          LOG_DEBUG(logger_, "armor published");
        })
        .or_else([&](auto) { LOG_ERROR(logger_, "loan shm failure!"); });
  }
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
