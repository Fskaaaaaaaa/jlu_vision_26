// #pragma once
//
// #include "colors.hpp"
//
// #include <Eigen/Dense>
// #include <array>
// #include <opencv2/core.hpp>
// #include <yaml_config_loader.hpp>
//
// namespace tools {
//
// void draw_points(cv::Mat &img, const std::vector<cv::Point> &points,
//                  const cv::Scalar &color, int thickness = 2);
// void draw_points(cv::Mat &img, const std::vector<cv::Point2f> &points,
//                  const cv::Scalar &color, int thickness = 2);
//
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
