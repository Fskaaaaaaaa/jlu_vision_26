#include "types/CameraInfo.hpp"
#include "Eigen/src/Core/Matrix.h"

#include <cstring>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

types::CameraInfo::CameraInfo(
    const iox::popo::Sample<const msgs::CameraInfo, const msgs::Header> &sample)
    : view_width_px(sample->view_width_px),
      view_height_px(sample->view_height_px) {
  std::memcpy(this->camera_matrix.data, sample->camera_matrix.data(),
              camera_matrix.elemSize() * 9);
  std::memcpy(this->distortion_coefficients.data,
              sample->distortion_coefficients.data(),
              distortion_coefficients.elemSize() * 5);
}

Eigen::Matrix3d types::CameraInfo::getCameraEigenMatrix() {
  Eigen::Matrix3d K;
  cv::cv2eigen(this->camera_matrix, K);
  return K;
}
