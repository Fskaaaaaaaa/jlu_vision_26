#include "pnp_solver.hpp"
#include "math/angle_tools.hpp"
#include "types/ArmorPoints.hpp"

#include "opencv2/calib3d.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "quill/LogMacros.h"
#include "types/ArmorType.hpp"
#include <numbers>
#include <opencv2/core/eigen.hpp>

#include <optional>
#include <vector>

auto_aim::PnPSolver::PnPSolver(quill::Logger *logger, const PnPConfig &config,
                               const types::CameraInfo &cam_info)
    : logger_(logger), config_(config),
      camera_matrix_(cam_info.camera_matrix.clone()),
      distortion_coefficients_(cam_info.distortion_coefficients.clone()) {}

std::optional<auto_aim::PnPResult>
auto_aim::PnPSolver::solvePnP(Armor &armor) const {
  const auto &object_points = types::points::getArmorPointsCV(
      (armor.type == types::ArmorType::Outpost &&
       config_.outpost_is_small_armor)
          ? types::ArmorType::Three
          : armor.type);
  std::vector<cv::Point2f> image_points{
      {armor.left_light.bottom},
      {armor.left_light.top},
      {armor.right_light.top},
      {armor.right_light.bottom},
  };
  PnPResult result;
  if (config_.use_generic_mode) {
    int solutions = cv::solvePnPGeneric(
        object_points, image_points, camera_matrix_, distortion_coefficients_,
        result.rvecs, result.tvecs, false, cv::SOLVEPNP_IPPE, cv::noArray(),
        cv::noArray(), result.project_errors);
    if (solutions <= 0)
      return std::nullopt;
    sortPnPResult(armor, result);
  } else {
    result.tvecs.resize(1);
    result.rvecs.resize(1);
    bool success = cv::solvePnP(object_points, image_points, camera_matrix_,
                                distortion_coefficients_, result.rvecs.at(0),
                                result.tvecs.at(0), false, cv::SOLVEPNP_IPPE);
    if (!success)
      return std::nullopt;
  }
  armor.position.x() = result.tvecs.at(0).at<double>(0);
  armor.position.y() = result.tvecs.at(0).at<double>(1);
  armor.position.z() = result.tvecs.at(0).at<double>(2);
  cv::Mat R_cv;
  cv::Rodrigues(result.rvecs.at(0), R_cv);
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  cv::cv2eigen(R_cv, R);
  armor.orientation = Eigen::Quaterniond(R);
  return result;
}

void auto_aim::PnPSolver::sortPnPResult(const Armor &armor,
                                        PnPResult &pnp_result) const {
  auto error0 = pnp_result.project_errors.at(0);
  auto error1 = pnp_result.project_errors.at(1);
  auto ratio = error1 / error0;
  if (ratio > config_.project_error_ratio_thres) {
    LOG_DEBUG(logger_, "[sortPnPResult]: large reprojection error ratio: {}.",
              ratio);
    return;
  }
  cv::Mat &rvec0 = pnp_result.rvecs.at(0);
  cv::Mat &rvec1 = pnp_result.rvecs.at(1);
  cv::Mat &tvec0 = pnp_result.tvecs.at(0);
  cv::Mat &tvec1 = pnp_result.tvecs.at(1);
  cv::Mat R0_cv, R1_cv;
  cv::Rodrigues(rvec0, R0_cv);
  cv::Rodrigues(rvec1, R1_cv);
  Eigen::Matrix3d R0, R1;
  cv::cv2eigen(R0_cv, R0);
  cv::cv2eigen(R1_cv, R1);
  // NOTE: 从z朝前的相机系拧到z朝上的ROS系
  Eigen::Matrix3d R_camera_to_ros = Eigen::Matrix3d::Zero();
  R_camera_to_ros << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Vector3d rpy0 = tools::rotationMatrixToRPY(R_camera_to_ros * R0);
  Eigen::Vector3d rpy1 = tools::rotationMatrixToRPY(R_camera_to_ros * R1);
  auto roll0 = tools::radian2Angle(tools::limitRadian(
      rpy0(0), {-std::numbers::pi / 2, std::numbers::pi / 2}));
  if (roll0 > config_.roll_thres_degree) {
    LOG_DEBUG(logger_, "[sortPnPResult]: large roll0: {}.", roll0);
    return;
  }
  double l_angle = tools::radian2Angle(
      std::atan2(armor.left_light.axis.y, armor.left_light.axis.x));
  double r_angle = tools::radian2Angle(
      std::atan2(armor.right_light.axis.y, armor.right_light.axis.x));
  double angle = (l_angle + r_angle) / 2;
  double armor_angle = angle + 90.;
  if (armor.type == types::ArmorType::Outpost)
    armor_angle = -armor_angle;
  // NOTE: 因为装甲板是向内倾斜的（前哨相反）
  // 如果装甲板左倾（angle > 0），选择Yaw为负的解
  // 如果装甲板右倾（angle < 0），选择Yaw为正的解
  if ((armor_angle > 0 && rpy0[2] > 0 && rpy1[2] < 0) ||
      (armor_angle < 0 && rpy0[2] < 0 && rpy1[2] > 0)) {
    std::swap(rvec0, rvec1);
    std::swap(tvec0, tvec1);
    LOG_DEBUG(logger_,
              "[sortPnPResult]: armor_angle: {}, second PnP solution selected.",
              armor_angle);
  }
}

float auto_aim::PnPSolver::calculateDistanceToCenter(
    const cv::Point2f &image_point) const {
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

void auto_aim::PnPSolver::drawFrameAxes(const PnPResult &pnp_result,
                                        cv::Mat &image) const {
  std::vector<cv::Point2f> image_points;
  const auto &object_points =
      types::points::getArmorPointsCV(pnp_result.armor_type);
  if (config_.use_generic_mode) {
    cv::drawFrameAxes(image, camera_matrix_, distortion_coefficients_,
                      pnp_result.rvecs.at(1), pnp_result.tvecs.at(1),
                      config_.frame_axes_length);
  }
  cv::drawFrameAxes(image, camera_matrix_, distortion_coefficients_,
                    pnp_result.rvecs.at(0), pnp_result.tvecs.at(0),
                    config_.frame_axes_length);
}
