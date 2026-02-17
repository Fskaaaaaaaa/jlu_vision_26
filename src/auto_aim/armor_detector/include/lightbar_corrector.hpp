// Maintained by Shenglin Qin, Chengfu Zou
// Modified by F
// Copyright (C) FYT Vision Group. All rights reserved.
#pragma once

#include "configs.hpp"
#include "quill/Logger.h"
#include "types.hpp"

#include <opencv2/opencv.hpp>
#include <optional>

namespace auto_aim {

struct SymmetryAxis {
  cv::Point2f centroid;
  cv::Point2f direction;
  float mean_val; // 平均亮度
};

class LightCornerCorrector {
public:
  LightCornerCorrector(quill::Logger *logger,
                       const LightCornerCorrectorConfig &config);
  bool correctCorners(Armor &armor, const cv::Mat &gray_img);

private:
  enum class FindCornerOrder {
    Top,
    Bottom,
  };
  std::optional<SymmetryAxis> findSymmetryAxis(const cv::Mat &gray_img,
                                               const LightBar &light);
  std::optional<cv::Point2f> findCorner(const cv::Mat &gray_img,
                                        const LightBar &light,
                                        const SymmetryAxis &axis,
                                        FindCornerOrder order);
  quill::Logger *logger_;
  LightCornerCorrectorConfig config_;
};

} // namespace auto_aim
