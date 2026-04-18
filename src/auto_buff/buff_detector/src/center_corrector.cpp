#include "center_corrector.hpp"
#include "math/cross_point.hpp"

#include "configs.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "quill/LogMacros.h"

#include <opencv2/imgproc.hpp>
#include <vector>

auto_buff::CorrectorConfig auto_buff::CenterCorrector::config_ =
    ConfigManager::instance()->configs().corrector_config;

void auto_buff::CenterCorrector::correctRunes(const cv::Mat &image,
                                              std::vector<RuneObject> &runes) {
  if (runes.size() == 0)
    return;

  std::sort(runes.begin(), runes.end(),
            [](auto &a, auto &b) { return a.box.area() > b.box.area(); });
  float maxArea = runes.front().box.area();
  float minArea = maxArea * config_.rune_size_tolerance;
  runes.erase(std::remove_if(runes.begin(), runes.end(),
                             [minArea](const RuneObject &obj) {
                               return obj.box.area() < minArea;
                             }),
              runes.end());

  cv::Point2f center;
  float center_x = 0, center_y = 0, weight = 0;
  if (CenterCorrector::getCenterpoint(image, runes, center)) {
    // LOG_INFO(ConfigManager::instance()->logger(), "x:{} y:{}", center.x, center.y);
    center_x += center.x * config_.center_weight;
    center_y += center.y * config_.center_weight;
    weight += config_.center_weight;
  }
  for (auto &rune : runes) {
    center_x += rune.points.center.x * rune.prob;
    center_y += rune.points.center.y * rune.prob;
    weight += rune.prob;
  }
  center_x = center_x / weight;
  center_y = center_y / weight;
  for (auto &rune : runes) {
    rune.points.center.x = center_x;
    rune.points.center.y = center_y;
  }
}

bool auto_buff::CenterCorrector::getCenterpoint(const cv::Mat &image,
                                                std::vector<RuneObject> runes,
                                                cv::Point2f &center) {
  cv::Mat roi;
  int corner_x, corner_y;
  switch (runes.size()) {
  case 0:
    return false;
  case 1: {
    float blade_center_x =
        runes[0].points.bottom_left.x + runes[0].points.bottom_right.x +
        runes[0].points.top_left.x + runes[0].points.top_right.x;
    blade_center_x /= 4;
    float blade_center_y =
        runes[0].points.bottom_left.y + runes[0].points.bottom_right.y +
        runes[0].points.top_left.y + runes[0].points.top_right.y;
    blade_center_y /= 4;
    Eigen::Vector2f direction(runes[0].points.center.x - blade_center_x,
                              runes[0].points.center.y - blade_center_y);
    direction.normalize();
    direction.x() = config_.roi_size * config_.skewing * direction.x();
    direction.y() = config_.roi_size * config_.skewing * direction.y();
    int x1 = static_cast<int>(runes[0].points.center.x + direction.x() -
                              config_.roi_size / 2.0);
    int y1 = static_cast<int>(runes[0].points.center.y + direction.y() -
                              config_.roi_size / 2.0);
    cv::Rect roi_rect = cv::Rect(x1, y1, config_.roi_size, config_.roi_size) &
                        cv::Rect(0, 0, image.cols, image.rows);
    corner_x = roi_rect.x;
    corner_y = roi_rect.y;
    roi = image(roi_rect);
    break;
  }
  case 2: {
    float blade_center_x_1 =
        runes[0].points.bottom_left.x + runes[0].points.bottom_right.x +
        runes[0].points.top_left.x + runes[0].points.top_right.x;
    blade_center_x_1 /= 4;
    float blade_center_y_1 =
        runes[0].points.bottom_left.y + runes[0].points.bottom_right.y +
        runes[0].points.top_left.y + runes[0].points.top_right.y;
    blade_center_y_1 /= 4;

    float blade_center_x_2 =
        runes[1].points.bottom_left.x + runes[1].points.bottom_right.x +
        runes[1].points.top_left.x + runes[1].points.top_right.x;
    blade_center_x_2 /= 4;
    float blade_center_y_2 =
        runes[1].points.bottom_left.y + runes[1].points.bottom_right.y +
        runes[1].points.top_left.y + runes[1].points.top_right.y;
    blade_center_y_2 /= 4;
    Eigen::Vector2f cross_point;
    tools::getCrossPoint(
        Eigen::Vector2f(blade_center_x_1, blade_center_y_1),
        Eigen::Vector2f(runes[0].points.center.x, runes[0].points.center.y),
        Eigen::Vector2f(blade_center_x_2, blade_center_y_2),
        Eigen::Vector2f(runes[1].points.center.x, runes[1].points.center.y),
        cross_point);
    cv::Rect roi_rect =
        cv::Rect(static_cast<int>(cross_point.x() - config_.roi_size / 2.0),
                 static_cast<int>(cross_point.y() - config_.roi_size / 2.0),
                 static_cast<int>(config_.roi_size),
                 static_cast<int>(config_.roi_size)) &
        cv::Rect(0, 0, image.cols, image.rows);
    corner_x = roi_rect.x;
    corner_y = roi_rect.y;
    roi = image(roi_rect);
    break;
    }
  default: {
    std::vector<cv::Point2f> points;
    for (auto &rune : runes)
      points.push_back(rune.points.center);
    cv::RotatedRect rect = cv::minAreaRect(points);
    cv::Rect roi_rect =
        cv::Rect(static_cast<int>(rect.center.x - config_.roi_size / 2.0),
                 static_cast<int>(rect.center.y - config_.roi_size / 2.0),
                 static_cast<int>(config_.roi_size),
                 static_cast<int>(config_.roi_size)) &
        cv::Rect(0, 0, image.cols, image.rows);
    corner_x = roi_rect.x;
    corner_y = roi_rect.y;
    roi = image(roi_rect);
  }
    break;
  }
//   LOG_INFO(ConfigManager::instance()->logger(), "{} {}", roi.cols, roi.rows);
  cv::Mat gray_img;
  cv::cvtColor(roi, gray_img, cv::COLOR_BGR2GRAY);
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
  cv::morphologyEx(binary_img, binary_img, cv::MORPH_OPEN, kernel);
  cv::dilate(binary_img, binary_img, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_NONE);

  std::vector<cv::RotatedRect> rects;
  for (auto &contour : contours) {
    auto rect = cv::minAreaRect(contour);
    float w = rect.size.width;
    float h = rect.size.height;
    if (w < 1e-6 || h < 1e-6)
        continue;
    float ratio = std::max(w, h) / std::min(w, h);
    if (fabs(ratio - 1.0f) < config_.center_shape_tolerance)
        rects.push_back(rect);
  }
  if (rects.size() == 0)
    return false;
  std::sort(rects.begin(), rects.end(),
            [](auto &a, auto &b) { return a.size.area() > b.size.area(); });
  auto &center_rect = rects[0];
  center_rect.center.x += corner_x;
  center_rect.center.y += corner_y;

  center.x = center_rect.center.x;
  center.y = center_rect.center.y;

  cv::Mat image_copy;
  image.copyTo(image_copy);

  cv::Point2f pts[4];
  center_rect.points(pts);

  for (int i = 0; i < 4; i++) {
      cv::line(image_copy, pts[i], pts[(i+1)%4], cv::Scalar(0,255,0), 2);
  }

  cv::circle(image_copy, center_rect.center, 3, cv::Scalar(255, 0, 0), -1);
  cv::resize(
      image_copy, image_copy,
      cv::Size(image_copy.size().width / 2, image_copy.size().height / 2));
  cv::imshow("image_copy", image_copy);

  return true;
}