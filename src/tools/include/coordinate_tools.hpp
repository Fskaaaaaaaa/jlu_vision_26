#pragma once

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <opencv2/core.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_ultra_tools {

geometry_msgs::msg::Pose xyzRpyToPose(const Eigen::Vector3d &xyz,
                                      const Eigen::Vector3d &rpy);

std::pair<cv::Mat, cv::Mat>
poseToTvecRvec(const geometry_msgs::msg::Pose &pose);

std::array<double, 3> orientationToRpy(const geometry_msgs::msg::Quaternion &q);

std::pair<std::array<double, 3>, std::array<double, 3>>
poseToXyzRpy(const geometry_msgs::msg::Pose &pose);

std::pair<std::array<double, 3>, std::array<double, 3>>
xyzaToXyzRpy(std::array<double, 4> xyza, double roll, double pitch);

std::pair<Eigen::Vector3d, Eigen::Vector3d>
xyzaToXyzRpy(Eigen::Vector4d xyza, double roll, double pitch);
// std::array<double, 3> orientationToRpy(const geometry_msgs::msg::Quaternion
// &q);
Eigen::Vector4d poseToXyza(const geometry_msgs::msg::Pose &pose);

} // namespace rm_ultra_tools
