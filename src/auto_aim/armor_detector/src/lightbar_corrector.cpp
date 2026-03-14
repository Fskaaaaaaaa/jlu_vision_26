// Copyright (C) FYT Vision Group. All rights reserved.
// Modified by GuGuGaGa
#include "lightbar_corrector.hpp"
#include "types.hpp"

#include "quill/LogMacros.h"

#include <algorithm>
#include <array>
#include <execution>
#include <functional>
#include <numeric>
#include <optional>

auto_aim::LightCornerCorrector::LightCornerCorrector(
    quill::Logger *logger, const LightCornerCorrectorConfig &config)
    : logger_(logger), config_(config) {}

bool auto_aim::LightCornerCorrector::correctCorners(Armor &armor,
                                                    const cv::Mat &gray_img) {
  auto process_lightbar = [&](LightBar &lightbar) -> bool {
    if (lightbar.width <= this->config_.pass_optimize_lightbar_width) {
      LOG_DEBUG(logger_, "[pca]: lightbar width too short!");
      return false;
    }
    auto axis = this->findSymmetryAxis(gray_img, lightbar);
    if (!axis.has_value()) {
      LOG_DEBUG(logger_, "[pca]: axis finding failed!");
      return false;
    }
    lightbar.center = axis->centroid;
    lightbar.axis = axis->direction;
    auto top = this->findCorner(gray_img, lightbar, axis.value(),
                                FindCornerOrder::Top);
    auto bottom = this->findCorner(gray_img, lightbar, axis.value(),
                                   FindCornerOrder::Bottom);
    lightbar.pca_ok = top.has_value() && bottom.has_value();
    if (lightbar.pca_ok) {
      lightbar.top = top.value();
      lightbar.bottom = bottom.value();
    }
    return lightbar.pca_ok;
  };

  // HACK: 为了避免“只优化缩短近灯条导致pnpyaw错误”，只保留两边都优化成功的情景
  // 稍微并发下减少下延迟。串行一次矫正10ms太吓人了。
  std::array<LightBar, 2> copys{armor.right_light, armor.left_light};
  auto success =
      std::transform_reduce(std::execution::par, copys.begin(), copys.end(),
                            true, std::logical_and<>(), process_lightbar);
  if (success) {
    armor.right_light = copys.at(0);
    armor.left_light = copys.at(1);
  }
  return success;
}

std::optional<auto_aim::SymmetryAxis>
auto_aim::LightCornerCorrector::findSymmetryAxis(const cv::Mat &gray_img,
                                                 const LightBar &light) {
  // Scale the bounding box
  cv::Rect light_box = light.boundingRect();
  light_box.x -= light_box.width * config_.padding_scale;
  light_box.y -= light_box.height * config_.padding_scale;
  light_box.width += light_box.width * config_.padding_scale * 2;
  light_box.height += light_box.height * config_.padding_scale * 2;

  // Check boundary
  light_box.x = std::max(light_box.x, 0);
  light_box.x = std::min(light_box.x, gray_img.cols - 1);
  light_box.y = std::max(light_box.y, 0);
  light_box.y = std::min(light_box.y, gray_img.rows - 1);
  light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
  light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);

  // Get normalized light image
  cv::Mat roi = gray_img(light_box);
  float mean_val = cv::mean(roi)[0];
  // NOTE: early return 非灯条的情况（过暗）
  if (mean_val <= config_.lightbar_min_mean_brightness) {
    LOG_DEBUG(logger_, "[pca]: lightbar too dark!");
    return std::nullopt;
  }
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, config_.normalize_max_brightness, cv::NORM_MINMAX);

  // Calculate the centroid
  cv::Moments moments = cv::moments(roi, false);
  cv::Point2f centroid =
      cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00) +
      cv::Point2f(light_box.x, light_box.y);

  // Initialize the PointCloud
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; i++) {
    for (int j = 0; j < roi.cols; j++) {
      for (int k = 0; k < std::round(roi.at<float>(i, j)); k++) {
        points.emplace_back(cv::Point2f(j, i));
      }
    }
  }
  cv::Mat points_mat = cv::Mat(points).reshape(1);

  // PCA (Principal Component Analysis)
  auto pca = cv::PCA(points_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // Get the symmetry axis
  cv::Point2f axis = cv::Point2f(pca.eigenvectors.at<float>(0, 0),
                                 pca.eigenvectors.at<float>(0, 1));

  // Normalize the axis
  axis /= cv::norm(axis);

  if (axis.y > 0) {
    axis = -axis;
  }

  return SymmetryAxis{
      .centroid = centroid, .direction = axis, .mean_val = mean_val};
}

std::optional<cv::Point2f> auto_aim::LightCornerCorrector::findCorner(
    const cv::Mat &gray_img, const LightBar &light, const SymmetryAxis &axis,
    FindCornerOrder order) {

  auto is_in_image = [&gray_img](const cv::Point &point) -> bool {
    return point.x >= 0 && point.x < gray_img.cols && point.y >= 0 &&
           point.y < gray_img.rows;
  };

  auto get_distance = [](float x0, float y0, float x1, float y1) -> float {
    return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
  };

  int oper = order == FindCornerOrder::Top ? 1 : -1;
  float L = light.length;
  float dx = axis.direction.x * oper;
  float dy = axis.direction.y * oper;

  std::vector<cv::Point2f> candidates;

  // Select multiple corner candidates and take the average as the final corner
  int n = light.width - 2;
  int half_n = std::round(n / 2);
  for (int i = -half_n; i <= half_n; i++) {
    float x0 = axis.centroid.x + L * config_.search_start_ratio * dx + i;
    float y0 = axis.centroid.y + L * config_.search_start_ratio * dy;

    cv::Point2f prev = cv::Point2f(x0, y0);
    cv::Point2f corner = cv::Point2f(x0, y0);
    float max_brightness_diff = 0;
    bool has_corner = false;
    for (float x = x0 + dx, y = y0 + dy;
         get_distance(x, y, x0, y0) <
         L * (config_.search_end_ratio - config_.search_start_ratio);
         x += dx, y += dy) {
      cv::Point2f cur = cv::Point2f(x, y);
      if (!is_in_image(cv::Point(cur))) {
        break;
      }
      float brightness_diff =
          gray_img.at<uchar>(prev) - gray_img.at<uchar>(cur);
      // 打擂台筛选最大梯度，重点在后半段判断mean
      if (brightness_diff > max_brightness_diff &&
          gray_img.at<uchar>(prev) > axis.mean_val) {
        max_brightness_diff = brightness_diff;
        corner = prev;
        has_corner = true;
      }
      prev = cur;
    }
    if (has_corner) {
      candidates.emplace_back(corner);
    }
  }
  if (candidates.empty()) {
    LOG_DEBUG(logger_, "[pca]: find corner failed!");
    return std::nullopt;
  }
  cv::Point2f result =
      std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
  return result / static_cast<float>(candidates.size());
}
