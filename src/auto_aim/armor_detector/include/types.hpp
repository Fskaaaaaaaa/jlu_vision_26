#pragma once

#include "types/Armor.hpp"
#include "types/EnemyColor.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <array>
#include <vector>

namespace auto_aim {

struct LightBar : public cv::RotatedRect {
  LightBar() = default;
  // 传统算法构造函数
  explicit LightBar(const std::vector<cv::Point> &contour);
  // 神经网络关键点构造函数
  explicit LightBar(const cv::Point2f top, const cv::Point2f bottom,
                    types::EnemyColor color = types::EnemyColor::Extinguished);
  static constexpr auto ratio_length_width = 6.7;
  types::EnemyColor color;
  cv::Point2f top;
  cv::Point2f bottom;
  double length;
  double width;
  cv::Point2f axis;
  float tilt_angle;
};

struct Armor : public ::types::Armor {
  Armor() = default;
  Armor(const LightBar &l1, const LightBar &l2);
  Armor(const std::array<cv::Point2f, 4> &key_points, float confidence,
        types::ArmorType armor_type, types::EnemyColor color);
  // 别忘了pnp后初始化distance_to_image_center字段

  LightBar left_light;
  LightBar right_light;
  cv::Point2f center;

  //   std::chrono::system_clock::time_point stamp;
  //   std::string frame_id;
  //   ArmorType type;
  //   EnemyColor color;
  //   double distance_to_image_center;
  //   Eigen::Quaterniond orientation;
  //   Eigen::Vector3d position;
};
} // namespace auto_aim
