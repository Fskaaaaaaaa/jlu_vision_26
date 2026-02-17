#include "types.hpp"

#include <numeric>

auto_aim::LightBar::LightBar(const std::vector<cv::Point> &contour)

    : cv::RotatedRect(cv::minAreaRect(contour)),
      color(types::EnemyColor::Extinguished) {
  // NOTE: 使用轮廓质心代替对角线交点来初始化旋转矩形中心，理论上更稳定
  center = std::accumulate(contour.begin(), contour.end(), cv::Point2f(0, 0),
                           [n = static_cast<float>(contour.size())](
                               const cv::Point2f &a, const cv::Point &b) {
                             return a + cv::Point2f(b.x, b.y) / n;
                           });
  cv::Point2f p[4];
  this->points(p);
  std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) {
    return a.y < b.y;
  });
  top = (p[0] + p[1]) / 2;
  bottom = (p[2] + p[3]) / 2;
  length = cv::norm(top - bottom);
  width = cv::norm(p[0] - p[1]);
  axis = top - bottom;
  axis = axis / cv::norm(axis);
  // Calculate the tilt angle
  // The angle is the angle between the light bar and the horizontal line
  tilt_angle =
      std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
  tilt_angle = tilt_angle / std::numbers::pi * 180;
}

auto_aim::LightBar::LightBar(const cv::Point2f top, const cv::Point2f bottom,
                             types::EnemyColor color)
    : color(color), top(top), bottom(bottom), length(cv::norm(top - bottom)),
      width(length / ratio_length_width), axis((top - bottom) / length) {
  // HACK:感觉还是组合更合适，但先这样吧
  static_cast<cv::RotatedRect &>(*this) = cv::RotatedRect{
      (top + bottom) / 2.,
      {static_cast<float>(width), static_cast<float>(length)},
      static_cast<float>(std::atan2(axis.y, axis.x) / std::numbers::pi * 180)};
  // XXX: 注意检查下anlge,貌似不对劲
  tilt_angle =
      std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
  tilt_angle /= std::numbers::pi * 180;
};

auto_aim::Armor::Armor(const LightBar &l1, const LightBar &l2) {
  if (l1.center.x < l2.center.x) {
    left_light = l1, right_light = l2;
  } else {
    left_light = l2, right_light = l1;
  }
  center = (left_light.center + right_light.center) / 2;
}

// NOTE:
// 关键点顺序
// 0 1
// 3 2
auto_aim::Armor::Armor(const std::array<cv::Point2f, 4> &key_points,
                       float confidence, types::ArmorType armor_type,
                       types::EnemyColor color)
    : left_light(key_points.at(0), key_points.at(3)),
      right_light(key_points.at(1), key_points.at(2)),
      center((left_light.center + right_light.center) / 2.) {
  this->confidence = confidence;
  this->type = armor_type;
  this->color = color;
}
