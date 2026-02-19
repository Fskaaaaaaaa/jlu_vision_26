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

inline void drawText(cv::Mat &img, const std::string &text,
                     const cv::Point &point,
                     const cv::Scalar &color = Color::bgr::RED,
                     double font_scale = 1.0, int thickness = 2) {
  cv::putText(img, text, point, cv::FONT_HERSHEY_SIMPLEX, font_scale, color,
              thickness);
}

} // namespace tools

// class ArmorDrawer {
// public:
//   ArmorDrawer(const std::array<double, 9> &camera_matrix,
//               const std::vector<double> &distortion_coefficients);
//   // TODO 可视化应该把坐标变换封装好，构造传入buffer就好
//   // void drawArmor(cv::Mat &img,
//   //                const geometry_msgs::msg::Pose &armor_pose_in_world,
//   //                const cv::Scalar &color = Color::RED,
//   //                const rclcpp::Time &time = rclcpp::Time(0),
//   //                bool draw_big_armor = false);
//   // void drawArmor(cv::Mat &img, const Eigen::Vector3d &armor_xyz,
//   //                const Eigen::Vector3d &armor_rpy,
//   //                const cv::Scalar &color = Color::RED,
//   //                const rclcpp::Time &time = rclcpp::Time(0),
//   //                bool draw_big_armor = false);
//   //
// private:
//   std::string camera_optical_frame_;
//   std::string odom_;
//   cv::Mat camera_matrix_;
//   cv::Mat dist_coeffs_;
//
//   static constexpr double LIGHTBAR_LENGTH = 56e-3;    // m
//   static constexpr double BIG_ARMOR_WIDTH = 230e-3;   // m
//   static constexpr double SMALL_ARMOR_WIDTH = 135e-3; // m
//
//   const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
//       {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
//       {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
//       {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
//       {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
//   const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
//       {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
//       {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
//       {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
//       {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
// };
//
// } // namespace tools
