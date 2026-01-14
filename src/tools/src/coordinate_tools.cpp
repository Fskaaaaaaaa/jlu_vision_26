#include "rm_ultra_tools/coordinate_tools.hpp"

#include <array>
#include <opencv2/opencv.hpp>

namespace rm_ultra_tools {

geometry_msgs::msg::Pose xyzRpyToPose(const Eigen::Vector3d &xyz,
                                      const Eigen::Vector3d &rpy) {
  geometry_msgs::msg::Pose p;
  p.position.x = xyz.x();
  p.position.y = xyz.y();
  p.position.z = xyz.z();
  tf2::Quaternion q;
  q.setRPY(rpy.x(), rpy.y(), rpy.z());
  p.orientation = tf2::toMsg(q);
  return p;
}

std::pair<cv::Mat, cv::Mat>
poseToTvecRvec(const geometry_msgs::msg::Pose &pose) {
  cv::Mat rvec, tvec;
  const auto &q = pose.orientation;
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 R_tf(tf_q);
  cv::Mat R_cv(3, 3, CV_64F);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      R_cv.at<double>(i, j) = R_tf[i][j];
  cv::Rodrigues(R_cv, rvec);
  const auto &t = pose.position;
  tvec = (cv::Mat_<double>(3, 1) << t.x, t.y, t.z);
  return {tvec, rvec};
}

std::array<double, 3>
orientationToRpy(const geometry_msgs::msg::Quaternion &q) {
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

std::pair<std::array<double, 3>, std::array<double, 3>>
poseToXyzRpy(const geometry_msgs::msg::Pose &pose) {
  auto rpy = orientationToRpy(pose.orientation);
  return {{{pose.position.x, pose.position.y, pose.position.z}}, rpy};
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
xyzaToXyzRpy(Eigen::Vector4d xyza, double roll, double pitch) {
  return {{xyza.x(), xyza.y(), xyza.z()}, {roll, pitch, xyza.w()}};
}

std::pair<std::array<double, 3>, std::array<double, 3>>
xyzaToXyzRpy(std::array<double, 4> xyza, double roll, double pitch) {
  return {std::array{xyza[0], xyza[1], xyza[2]},
          std::array{roll, pitch, xyza[3]}};
}

Eigen::Vector4d poseToXyza(const geometry_msgs::msg::Pose &pose) {
  auto [xyz, rpy] = poseToXyzRpy(pose);
  auto &[x, y, z] = xyz;
  auto yaw = rpy[2];
  return {x, y, z, yaw};
}
} // namespace rm_ultra_tools
