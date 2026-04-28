#pragma once

#include "configs.hpp"
#include "hardware/cam_info_listener.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "transform/tf_listener.hpp"
#include "types/CameraInfo.hpp"
#include "quill/Logger.h"
#include "types/Basic.hpp"
#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
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
  std::optional<std::array<cv::Point2d, 4>>
  getArmorCornersInImageOpt(const msgs::Armor &armor,
                            const std::chrono::system_clock::time_point &stamp,
                            Eigen::Isometry3d &pose_camera);
  std::vector<msgs::Armor>
  getArmorMsgsFromStatus(const std::chrono::system_clock::time_point &stamp);
  void publishArmors();
  void updateContrlPhysic();

  quill::Logger *logger_;
  RobotConfig config_;
  std::jthread physic_pub_thread_;
  std::chrono::system_clock::time_point last_update_stamp_;
  iox::popo::Publisher<msgs::Armor, msgs::Header> armor_pub_;
  fast_tf::detail::transform_buffer tf_buffer_;
  tf::TransformListener tf_listener_;
  hardware::CameraInfoListener cam_info_listener_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
  int view_width_px_;
  int view_height_px_;
  RobotStatus status_;
  std::mutex status_mtx_;
  RobotContrl &atom_contrl_;
};

} // namespace auto_aim
