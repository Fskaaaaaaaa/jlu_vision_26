// Copyright (c) 2026 GuGuGaGa. All Rights Reserved.
#pragma once

#include "configs.hpp"
#include "opencv2/core/mat.hpp"
#include "quill/Logger.h"
#include "types.hpp"
#include "types/CameraInfo.hpp"

#include "opencv2/core/types.hpp"

#include <optional>
#include <vector>
namespace auto_aim {

struct PnPResult {
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  std::vector<double> project_errors;
};

class PnPSolver {
public:
  PnPSolver(quill::Logger *logger, const PnPConfig &config,
            const types::CameraInfo &cam_info);
  // NOTE:有解时返回ippe的双解，第一个是被选择的解，被写入到引用的装甲板里
  std::optional<PnPResult> solvePnP(Armor &armor) const;
  float calculateDistanceToCenter(const cv::Point2f &image_point) const;

  // NOTE: 用于调试，绘制pnp的结果，绿色圈是被选择的结果，红圈是被抛弃的结果
  void drawFrameAxes(const PnPResult &pnp_result, cv::Mat &image) const;

private:
  void sortPnPResult(const Armor &armor, PnPResult &pnp_result) const;
  // double getReprojectionError(const std::vector<cv::Point2f> &image_points,
  //                             const cv::Mat &rvec, const cv::Mat &tvec,
  //                             types::ArmorType armor_type) const;

  quill::Logger *logger_;
  PnPConfig config_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
};
} // namespace auto_aim
