#pragma once

#include "colors.hpp"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace tools {

inline void drawPoints(cv::Mat &img, const std::vector<cv::Point> &points,
                       const cv::Scalar &color = Color::bgr::RED,
                       int thickness = 2) {
  std::vector<std::vector<cv::Point>> contours = {points};
  cv::drawContours(img, contours, -1, color, thickness);
}

inline void drawPoints(cv::Mat &img, const std::vector<cv::Point2f> &points,
                       const cv::Scalar &color = Color::bgr::RED,
                       int thickness = 2) {
  std::vector<cv::Point> int_points(points.begin(), points.end());
  drawPoints(img, int_points, color, thickness);
}

} // namespace tools
