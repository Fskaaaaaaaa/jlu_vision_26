#pragma once

#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"
#include "types/ArmorType.hpp"
#include "types/EnemyColor.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include <Eigen/Dense>

#include <array>
#include <chrono>

namespace types {
enum class ArmorPointPosition {
  LeftBottom,
  LeftTop,
  RightTop,
  RightBottom,
};
struct Armor {
  Armor() = default;
  Armor(const iox::popo::Sample<const msgs::Armor, const msgs::Header> &sample);
  Eigen::Vector3d getRpy() const;
  std::chrono::system_clock::time_point stamp;
  std::string frame_id;
  ArmorType type;
  EnemyColor color;
  double distance_to_image_center;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  // HACK: 为了精简和避免和detector子类添加的灯条类重复使用了array
  // 内部丢失了哪个位姿的点的语义，使用枚举cast当index避免访问时犯糖
  std::array<Eigen::Vector2d, 4> key_points;
  float confidence;
  bool key_frame;  // NOTE:是否为关键帧（由pca和ba共同判断）
  bool heart_beat; // NOTE:是否为用来驱动tracker的心跳空数据
};

} // namespace types

// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(types::Armor)
