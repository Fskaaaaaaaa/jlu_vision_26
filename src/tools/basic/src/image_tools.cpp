// #include "rm_ultra_tools/image_tools.hpp"
// #include "rm_ultra_tools/coordinate_tools.hpp"
//
// #include <cmath>
// #include <opencv2/opencv.hpp>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/time.hpp>
// #include <tf2/LinearMath/Matrix3x3.hpp>
// #include <tf2/LinearMath/Quaternion.hpp>
// #include <tf2/convert.hpp>
// #include <tf2/time.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_ros/buffer_interface.h>
// #include <vector>
//
// namespace rm_ultra_tools {
//
// void drawPoints(cv::Mat &img, const std::vector<cv::Point> &points,
//                 const cv::Scalar &color, int thickness) {
//   std::vector<std::vector<cv::Point>> contours = {points};
//   cv::drawContours(img, contours, -1, color, thickness);
// }
//
// void drawPoints(cv::Mat &img, const std::vector<cv::Point2f> &points,
//                 const cv::Scalar &color, int thickness) {
//   std::vector<cv::Point> int_points(points.begin(), points.end());
//   drawPoints(img, int_points, color, thickness);
// }
//
// ArmorDrawer::ArmorDrawer(const std::array<double, 9> &camera_matrix,
//                          const std::vector<double> &distortion_coefficients,
//                          std::shared_ptr<tf2_ros::Buffer> tf2_buffer,
//                          const std::string name_of_camera_optical_frame,
//                          const std::string name_of_odom)
//     : camera_optical_frame_(name_of_camera_optical_frame),
//     odom_(name_of_odom),
//       camera_matrix_(
//           cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data()))
//               .clone()),
//       dist_coeffs_(cv::Mat(1, 5, CV_64F,
//                            const_cast<double
//                            *>(distortion_coefficients.data()))
//                        .clone()),
//       tf2_buffer_(tf2_buffer) {}
//
// void ArmorDrawer::drawArmor(cv::Mat &img,
//                             const geometry_msgs::msg::Pose &pose_in_world,
//                             const cv::Scalar &color, const rclcpp::Time
//                             &time, bool is_big_armor) {
//   geometry_msgs::msg::Pose pose_in_cam;
//   if (time == rclcpp::Time(0)) {
//     try {
//       auto transform = tf2_buffer_->lookupTransform(camera_optical_frame_,
//                                                     odom_,
//                                                     tf2::TimePointZero);
//       tf2::doTransform(pose_in_world, pose_in_cam, transform);
//     } catch (const tf2::ExtrapolationException &ex) {
//       RCLCPP_ERROR(rclcpp::get_logger("drawArmor"),
//                    "Error while transforming %s", ex.what());
//       return;
//     }
//   } else {
//     try {
//       auto transform =
//           tf2_buffer_->lookupTransform(camera_optical_frame_, odom_, time);
//       tf2::doTransform(pose_in_world, pose_in_cam, transform);
//     } catch (const tf2::ExtrapolationException &ex) {
//       RCLCPP_ERROR(rclcpp::get_logger("drawArmor"),
//                    "Error while transforming %s", ex.what());
//       return;
//     }
//   }
//   auto [tvec, rvec] = poseToTvecRvec(pose_in_cam);
//   std::vector<cv::Point2f> image_points;
//   auto objpoints = is_big_armor ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
//   cv::projectPoints(objpoints, rvec, tvec, camera_matrix_, dist_coeffs_,
//                     image_points);
//   drawPoints(img, image_points, color, 2);
// }
//
// void ArmorDrawer::drawArmor(cv::Mat &img, const Eigen::Vector3d &armor_xyz,
//                             const Eigen::Vector3d &armor_rpy,
//                             const cv::Scalar &color, const rclcpp::Time
//                             &time, bool draw_big_armor) {
//   // auto color = std::fabs(armor_rpy.z()) >= 90.0 ? Color::BLUE :
//   Color::RED;
//   // 背面是蓝色，正面是红色
//   auto pose = xyzRpyToPose(armor_xyz, armor_rpy);
//   this->drawArmor(img, pose, color, time, draw_big_armor);
//   return;
// }
//
// } // namespace rm_ultra_tools
