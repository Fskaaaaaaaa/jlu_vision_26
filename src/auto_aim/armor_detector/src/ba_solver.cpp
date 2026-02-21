// Copyright (c) 2026 L. All Rights Reserved.
#include "ba_solver.hpp"
#include "math/angle_tools.hpp"
#include "quill/LogMacros.h"
#include "types/ArmorPoints.hpp"

#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/core/types.hpp"
#include <Eigen/Dense>
#include <exception>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cstdlib>
#include <vector>

auto_aim::ProjectionFactor::ProjectionFactor(
    const gtsam::SharedNoiseModel &model, const gtsam::Key &key,
    const cv::Point3f &obj_point, const cv::Point2f &image_point,
    const cv::Mat &camera_matrix, const cv::Mat &distortion_coefficients)
    : auto_aim::ProjectionFactor::Base(model, key), obj_point_(obj_point),
      img_point_(image_point), camera_matrix_(camera_matrix),
      distortion_coefficients_(distortion_coefficients) {
  Eigen::Matrix3d R_camera_to_ros = Eigen::Matrix3d::Zero();
  R_camera_to_ros << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  R_ros_to_camera_ = R_camera_to_ros.inverse();
}

gtsam::Vector
auto_aim::ProjectionFactor::evaluateError(const gtsam::Pose3 &pose,
                                          gtsam::OptionalMatrixType H) const {
  Eigen::Matrix3d rmat_in_ros = pose.rotation().matrix();
  Eigen::Matrix3d rmat_in_camera = R_ros_to_camera_ * rmat_in_ros;
  Eigen::Vector3d translation_in_camera = R_ros_to_camera_ * pose.translation();
  cv::Mat rmat_in_camera_cv;
  cv::eigen2cv(rmat_in_camera, rmat_in_camera_cv);
  cv::Mat rvec, tvec;
  cv::Rodrigues(rmat_in_camera_cv, rvec);
  cv::eigen2cv(translation_in_camera, tvec);
  std::vector<cv::Point2f> reproj_points;
  cv::projectPoints(std::vector{obj_point_}, rvec, tvec, camera_matrix_,
                    distortion_coefficients_, reproj_points);
  // HACK: 这里最好用GTSAM的投影方法实现。
  auto reproj_point = reproj_points.at(0);
  auto error = reproj_point - img_point_;
  return Eigen::Vector2d{error.x, error.y};
}

auto_aim::BASolver::BASolver(quill::Logger *logger, const BaConfig &config,
                             const types::CameraInfo &cam_info)
    : logger_(logger), config_(config), camera_matrix_(cam_info.camera_matrix),
      distortion_coefficients_(cam_info.distortion_coefficients) {}

bool auto_aim::BASolver::optimizeArmorPose(Armor &armor) const {
  using namespace gtsam::symbol_shorthand;
  try {
    // 获得ros系内装甲板的初始状态
    Eigen::Matrix3d R_camera_to_ros = Eigen::Matrix3d::Zero();
    R_camera_to_ros << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Matrix3d R_in_ros = R_camera_to_ros * armor.orientation;
    Eigen::Vector3d pose = R_camera_to_ros * armor.position;
    Eigen::Vector3d rpy = tools::rotationMatrixToRPY(R_in_ros);
    gtsam::Vector6 sigmas;
    auto initial_pose =
        gtsam::Pose3(gtsam::Rot3::RzRyRx(rpy.x(), rpy.y(), rpy.z()), pose);

    // 添加先验约束
    sigmas << config_.prior_noise.roll, config_.prior_noise.pitch,
        config_.prior_noise.yaw, config_.prior_noise.xyz,
        config_.prior_noise.xyz, config_.prior_noise.xyz;
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    gtsam::NonlinearFactorGraph graph;
    graph.addPrior(X(0), initial_pose, prior_noise);

    // 添加四个关键点的投影因子
    std::vector<cv::Point2f> img_points{
        {armor.left_light.bottom},
        {armor.left_light.top},
        {armor.right_light.top},
        {armor.right_light.bottom},
    };
    const auto &obj_points = types::points::getArmorPointsCV(armor.type);
    gtsam::SharedDiagonal measurement_noise =
        gtsam::noiseModel::Diagonal::Sigmas(
            gtsam::Vector2::Constant(config_.measurement_noise_px_xy));
    for (int i = 0; i < img_points.size(); i++) {
      graph.emplace_shared<ProjectionFactor>(
          measurement_noise, X(0), obj_points.at(i), img_points.at(i),
          camera_matrix_, distortion_coefficients_);
    }

    // 使用LM进行优化
    gtsam::Values initial_value;
    initial_value.insert(X(0), initial_pose);
    auto result =
        gtsam::LevenbergMarquardtOptimizer(graph, initial_value).optimize();
    if (config_.print_result)
      result.print();

    // 获取优化结果
    auto optimized = result.at<gtsam::Pose3>(X(0));
    Eigen::Matrix3d R_ros_to_camera = R_camera_to_ros.inverse();
    Eigen::Matrix3d optimized_R =
        R_ros_to_camera * optimized.rotation().matrix();
    Eigen::Vector3d optimized_position =
        R_ros_to_camera * optimized.translation();
    armor.position = optimized_position;
    armor.orientation = Eigen::Quaterniond{optimized_R};
    return true;
  } catch (const std::exception &e) {
    LOG_ERROR(logger_, "[ba_solver]: Error: {}.", e.what());
    return false;
  } catch (...) {
    LOG_ERROR(logger_, "[ba_solver]: Unkown error.");
    return false;
  }
}
