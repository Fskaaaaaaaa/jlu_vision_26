#pragma once

#include "msgs/BuffBlade.hpp"
#include "msgs/Header.hpp"
#include "types/BuffBladeType.hpp"
#include "types/EnemyColor.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include "opencv2/core/types.hpp"
#include <Eigen/Dense>
#include <gtsam/geometry/Rot2.h>
#include <opencv2/core.hpp>

namespace auto_buff {

// NOTE:
//                ^ z
//                |
//                0
//                |
//        4       |        1
//                |
// <--------------x
// y
//           3          2
enum class BuffBladeIndex {
  _0 = 0,
  _1,
  _2,
  _3,
  _4,
};

// inline const std::vector<cv::Point3f> SINGLE_BLADE_OBJ_POINTS{
//     cv::Point3f(0, 0, 0) / 1000,         cv::Point3f(0, -541.5, 186) / 1000,
//     cv::Point3f(0, -858.5, 160) / 1000,  cv::Point3f(0, -858.5, -160) / 1000,
//     cv::Point3f(0, -541.5, -186) / 1000,
// };
// NOTE: 将风车的世界点从朝右改为朝上的了，更符合index定义
// Rune object points
// r_tag, bottom_left, top_left, top_right, bottom_right
inline const std::vector<cv::Point3f> BUFF_BLADE_OBJ_POINTS{
    cv::Point3f(0, 0, 0) / 1000,        cv::Point3f(0, 186, 541.5) / 1000,
    cv::Point3f(0, 160, 858.5) / 1000,  cv::Point3f(0, -160, 858.5) / 1000,
    cv::Point3f(0, -186, 541.5) / 1000,
};

enum class BuffPointPosition {
  Center,
  BottomRight,
  TopRight,
  TopLeft,
  BottomLeft,
};

struct BuffBladePoints {
  cv::Point2f r_center;
  cv::Point2f bottom_right;
  cv::Point2f top_right;
  cv::Point2f top_left;
  cv::Point2f bottom_left;
};

// 从msg构造，用于接受信息
struct BuffBlade {
  BuffBlade() = default;
  BuffBlade(const iox::popo::Sample<const msgs::BuffBlade, const msgs::Header>
                &sample);
  types::EnemyColor color;
  types::BuffBladeType type;
  float confidence;
  BuffBladePoints points;
  // NOTE: 有需要的随时再添加
};

// 只保留位置信息
struct BuffBladePositionRoll {
  Eigen::Vector3d position;
  gtsam::Rot2 roll;
  Eigen::Vector3d getHitPosition();
};

// 添加了角点信息
struct BuffBladePositionRollPoints : BuffBladePositionRoll {
  BuffBladePoints points;
};

struct BuffState {
  Eigen::Vector3d position;
};

} // namespace auto_buff
