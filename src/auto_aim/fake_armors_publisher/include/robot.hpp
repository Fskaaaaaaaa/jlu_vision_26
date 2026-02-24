#pragma once

#include "configs.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "quill/Logger.h"
#include "types/Basic.hpp"
#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace auto_aim {

struct RobotStatus {
  Eigen::Vector2d position;
  Eigen::Vector2d linear_velocity;
  Eigen::Vector2d linear_acceleration;
  double yaw;
  double vel_yaw;
  double acc_yaw;
}; // 这个就是机器人自己维护的了

struct RobotContrl {
  std::atomic<types::Vector2d> traction_direction;
  std::atomic<std::optional<bool>> spin_status;
  // 不旋转、顺时针、逆时针
  std::atomic<bool> const_speed_spin;
  std::atomic<bool> hide_armors;
};

class Robot {
public:
  Robot(quill::Logger *logger, const RobotConfig &config, RobotContrl &contrl);
  RobotStatus getState();
  void resetStatus();

private:
  std::vector<msgs::Armor> getArmorMsgsFromStatus();
  void publishArmors();
  void updateContrlPhysic();

  quill::Logger *logger_;
  RobotConfig config_;
  std::jthread physic_pub_thread_;
  std::chrono::system_clock::time_point last_update_stamp_;
  iox::popo::Publisher<msgs::Armor, msgs::Header> armor_pub_;
  RobotStatus status_;
  std::mutex status_mtx_;
  RobotContrl &atom_contrl_;
};

} // namespace auto_aim
