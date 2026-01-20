#pragma once

#include "msgs/CameraInfo.hpp"
#include "msgs/Header.hpp"
#include "opencv2/core/mat.hpp"

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>

#include <iceoryx_posh/popo/sample.hpp>
namespace types {
struct CameraInfo {
  CameraInfo(const iox::popo::Sample<const msgs::CameraInfo, const msgs::Header>
                 &sample);
  CameraInfo() = default;
  Eigen::Matrix3d getCameraEigenMatrix();
  int view_width_px;
  int view_height_px;
  cv::Mat camera_matrix = cv::Mat_<double>(3, 3);
  cv::Mat distortion_coefficients = cv::Mat_<double>(1, 5);
};
} // namespace types
