#pragma once

#include "types/Armor.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace auto_aim {

struct Lightbar {
  std::size_t id;
  types::Color color;
  cv::Point2f center;
  cv::Point2f top;
  cv::Point2f bottom;
  cv::Point2f top2bottom;
  std::vector<cv::Point2f> points;
  double angle;
  double angle_error;
  double length;
  double width;
  double ratio;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect &rotated_rect, std::size_t id);
  Lightbar() = default;
};

struct Armor : public ::types::Armor {};
} // namespace auto_aim
