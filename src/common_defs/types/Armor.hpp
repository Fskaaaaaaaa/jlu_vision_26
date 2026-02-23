#pragma once

#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "types/ArmorType.hpp"
#include "types/EnemyColor.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include <Eigen/Dense>

#include <chrono>

namespace types {
struct Armor {
  Armor() = default;
  Armor(const iox::popo::Sample<const msgs::Armor, const msgs::Header> &sample);
  Eigen::Vector3d getRpy() const;
  // msgs::Armor toMsg();
  std::chrono::system_clock::time_point stamp;
  std::string frame_id;
  ArmorType type;
  EnemyColor color;
  double distance_to_image_center;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  float confidence;
  bool key_frame;  // NOTE:是否为关键帧（由pca和ba共同判断）
  bool heart_beat; // NOTE:是否为用来驱动tracker的心跳空数据
};

} // namespace types
