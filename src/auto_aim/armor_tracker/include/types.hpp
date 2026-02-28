// Copyright (c) 2026 GuGuGaAaaaa. All Rights Reserved.
#pragma once

#include "types/Armor.hpp"
#include "types/ArmorType.hpp"

#include <Eigen/Core>
#include <gtsam/geometry/Rot2.h>

namespace auto_aim {

// NOTE:
// 四个装甲板时的索引
//            ^x
//            |              ^
//            2              |
//            |  /->rb       | center_yaw方向
// y<-----3---+---1-----
//            |->ra
//            0
//            |
// 三个装甲板时同理（都是均分圆周后由-x轴开始逆时针旋转）
// Robot在13方向上使用dz，outpost在1使用dz_a，在2使用dz_b
enum class ArmorIndex {
  _0 = 0,
  _1 = 1,
  _2 = 2,
  _3 = 3,
};

class ArmorException : public std::exception {
public:
  explicit ArmorException(const std::string &msg) : message_(msg) {}
  const char *what() const noexcept override { return message_.c_str(); }

private:
  std::string message_;
};

enum TrackStatus {
  Lost,
  TempLost,
  Tracking,
};

struct Armor;

struct TargetStatus {
  types::ArmorType type;
  // for robot
  double radius_a;
  double radius_b;
  double dz;
  // for outpost
  double radius;
  double dz_a;
  double dz_b;
  // center status
  Eigen::Vector3d center_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_velocity = Eigen::Vector3d::Zero();
  double center_yaw = 0;
  double center_vyaw = 0;
  TrackStatus track_status = TrackStatus::Lost;
  std::uint64_t k = 0;
  TargetStatus predict(double dt) const;
  std::vector<Armor> armors() const;
};

// NOTE: 装甲板类: 能从center_pos+yaw+r1/r2dz+armor_index构造position和yaw
// 用在装甲板因子和弹道解算里
// 另外弹道解算不要参数化了。给上限约束死了不说还很别扭()
struct Armor : public types::Armor {
  Armor() = default;
  Armor(const types::Armor &armor);
  // NOTE: 给装甲板因子的两个构造函数。ArmorIndex与半径AB不匹配时抛出异常
  // HACK: 统一了装甲板类型。但是用在装甲板因子里时大部分信息都是冗余的
  Armor(const Eigen::Vector3d &center_pos, double center_yaw, double radius_a,
        ArmorIndex armor_index);
  Armor(const Eigen::Vector3d &center_pos, double center_yaw, double radius_b,
        double dz, ArmorIndex armor_index);

  // NOTE: 只初始化position和yaw字段，小心UB
  static Armor fromTargetStatus(const TargetStatus &status,
                                ArmorIndex armor_index);
  static Armor fromRobot(const Eigen::Vector3d &center_pos, double center_yaw,
                         double radius_a, double radius_b, double dz,
                         ArmorIndex armor_index);
  static Armor fromOutpost(const Eigen::Vector3d &center_pos, double center_yaw,
                           double radius, double dz_a, double dz_b,
                           ArmorIndex armor_index);
  static Armor frmoBase(const Eigen::Vector3d &center_pos, double center_yaw,
                        ArmorIndex armor_index);

  // Eigen::Vector3d position;
  gtsam::Rot2 yaw;
  ArmorIndex index;

  // std::chrono::system_clock::time_point stamp;
  // std::string frame_id;
  // ArmorType type;
  // EnemyColor color;
  // double distance_to_image_center;
  // Eigen::Quaterniond orientation; NOTE: 保存原始四元数用于重投影debug
  // float confidence;
  // bool key_frame; // 是否为关键帧（由pca和ba共同判断）
};

} // namespace auto_aim
