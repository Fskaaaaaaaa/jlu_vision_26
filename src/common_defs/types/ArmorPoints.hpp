#pragma once

#include "types/ArmorType.hpp"

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

#include <vector>

namespace types {

namespace points {

[[deprecated]]
constexpr double ARMOR_HEIGHT = 125e-3;       // m
constexpr double BIG_ARMOR_HEIGHT = 127e-3;   // m
constexpr double SMALL_ARMOR_HEIGHT = 125e-3; // m
constexpr double TINY_ARMOR_HEIGHT = 100e-3;  // m
constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m
constexpr double TINY_ARMOR_WIDTH = 129e-3;   // m

inline double getArmorWidth(types::ArmorType type) {
  switch (type) {
  case types::ArmorType::Base:
    return TINY_ARMOR_WIDTH;
  case types::ArmorType::Outpost:
    return TINY_ARMOR_WIDTH;
  case types::ArmorType::One:
    return BIG_ARMOR_WIDTH;
  default:
    return SMALL_ARMOR_WIDTH;
  }
}

inline double getArmorHeight(types::ArmorType type) {
  switch (type) {
  case types::ArmorType::Base:
    return TINY_ARMOR_HEIGHT;
  case types::ArmorType::Outpost:
    return TINY_ARMOR_HEIGHT;
  case types::ArmorType::One:
    return BIG_ARMOR_HEIGHT;
  default:
    return SMALL_ARMOR_HEIGHT;
  }
}

// NOTE: 从左下角开始按顺时针顺序
//  坐标：x 向前，y 向左，z 向上
inline const std::vector<cv::Point3f> CV_BIG_ARMOR_POINTS{
    {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};
inline const std::vector<cv::Point3f> CV_SMALL_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};
inline const std::vector<cv::Point3f> CV_TINY_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};
inline const std::vector<cv::Point3f> &getArmorPointsCV(::types::ArmorType t) {
  switch (t) {
  case ::types::ArmorType::One:
    return CV_BIG_ARMOR_POINTS;
  case ::types::ArmorType::Two:
    return CV_SMALL_ARMOR_POINTS;
  case ::types::ArmorType::Three:
    return CV_SMALL_ARMOR_POINTS;
  case ::types::ArmorType::Four:
    return CV_SMALL_ARMOR_POINTS;
  case ::types::ArmorType::Sentry:
    return CV_SMALL_ARMOR_POINTS;
  case ::types::ArmorType::Outpost:
    return CV_TINY_ARMOR_POINTS;
  case ::types::ArmorType::Base:
    return CV_TINY_ARMOR_POINTS;
  default:
    return CV_SMALL_ARMOR_POINTS; // NOTE:非装甲板情况也不能空着，默认返回小装甲板的点
  }
}

inline const std::vector<Eigen::Vector3d> EG_BIG_ARMOR_POINTS{
    {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};
inline const std::vector<Eigen::Vector3d> EG_SMALL_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};
inline const std::vector<Eigen::Vector3d> EG_TINY_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
};
inline const std::vector<Eigen::Vector3d> &
getArmorPointsEG(::types::ArmorType t) {
  switch (t) {
  case ::types::ArmorType::One:
    return EG_BIG_ARMOR_POINTS;
  case ::types::ArmorType::Two:
    return EG_SMALL_ARMOR_POINTS;
  case ::types::ArmorType::Three:
    return EG_SMALL_ARMOR_POINTS;
  case ::types::ArmorType::Four:
    return EG_SMALL_ARMOR_POINTS;
  case ::types::ArmorType::Sentry:
    return EG_SMALL_ARMOR_POINTS;
  case ::types::ArmorType::Outpost:
    return EG_TINY_ARMOR_POINTS;
  case ::types::ArmorType::Base:
    return EG_TINY_ARMOR_POINTS;
  default:
    return EG_SMALL_ARMOR_POINTS;
  }
}

} // namespace points

} // namespace types
