#include "ba_solver.hpp"
#include "math/angle_tools.hpp"
#include "quill/LogMacros.h"
#include "types/ArmorPoints.hpp"

#include "opencv2/core/eigen.hpp"
#include "opencv2/core/types.hpp"
#include <Eigen/Dense>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <cstdlib>
#include <exception>
#include <vector>

auto_aim::ProjectionFactor::ProjectionFactor(
    const gtsam::SharedNoiseModel &model, gtsam::Key key,
    const gtsam::Point3 &obj_point, const gtsam::Point2 &measurement,
    const gtsam::Cal3DS2 &K)
    : Base(model, key), point_(obj_point), measurement_(measurement), K_(K) {
  Eigen::Matrix3d R_camera_to_ros;
  R_camera_to_ros << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  R_ros_to_camera_ = R_camera_to_ros.inverse();
}

gtsam::Vector
auto_aim::ProjectionFactor::evaluateError(const gtsam::Pose3 &pose,
                                          gtsam::OptionalMatrixType H) const {

  const gtsam::Pose3 T_ros_cam(gtsam::Rot3(R_ros_to_camera_),
                               gtsam::Point3(0, 0, 0));
  gtsam::Matrix66 H_compose;
  gtsam::Pose3 T_armor_cam = T_ros_cam.compose(pose, H_compose);
  gtsam::Matrix36 H_transform;
  gtsam::Point3 p_cam = T_armor_cam.transformFrom(point_, H_transform);
  if (p_cam.z() <= 1e-6) {
    if (H)
      *H = gtsam::Matrix::Zero(2, 6);
    return gtsam::Vector2::Zero();
  }
  gtsam::Matrix23 H_norm;
  gtsam::Point2 pn =
      gtsam::PinholeCamera<gtsam::Cal3DS2>::Project(p_cam, H_norm);
  if (H_norm.rows() != 2 || H_norm.cols() != 3) {
    if (H)
      *H = gtsam::Matrix::Zero(2, 6);
    return gtsam::Vector2::Zero();
  }
  gtsam::Matrix22 H_calib;
  gtsam::Point2 proj = K_.uncalibrate(pn, {}, H_calib);
  if (H_calib.rows() != 2) {
    if (H)
      *H = gtsam::Matrix::Zero(2, 6);
    return gtsam::Vector2::Zero();
  }
  if (H) {
    *H = H_calib * H_norm * H_transform * H_compose;
  }
  return proj - measurement_;
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
    Eigen::Matrix3d R_in_ros = R_camera_to_ros * armor.orientation.matrix();
    Eigen::Vector3d pose = R_camera_to_ros * armor.position;
    Eigen::Vector3d rpy = tools::rotationMatrixToRPY(R_in_ros);
    gtsam::Vector6 sigmas;
    auto initial_pose =
        gtsam::Pose3(gtsam::Rot3::RzRyRx(rpy.z(), rpy.y(), rpy.x()), pose);

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
      const auto &obj = obj_points.at(i);
      const auto &img = img_points.at(i);
      graph.emplace_shared<ProjectionFactor>(
          measurement_noise, X(0), gtsam::Point3{obj.x, obj.y, obj.z},
          gtsam::Point2{img.x, img.y},
          gtsam::Cal3DS2{
              camera_matrix_.at<double>(0, 0),
              camera_matrix_.at<double>(1, 1),
              camera_matrix_.at<double>(0, 1),
              camera_matrix_.at<double>(0, 2),
              camera_matrix_.at<double>(1, 2),
              distortion_coefficients_.at<double>(0),
              distortion_coefficients_.at<double>(1),
              distortion_coefficients_.at<double>(2),
              distortion_coefficients_.at<double>(3),
          });
    }

    // 使用LM进行优化
    gtsam::Values initial_value;
    initial_value.insert(X(0), initial_pose);
    gtsam::LevenbergMarquardtParams params;
    if (config_.print_result)
      params.setVerbosity("TERMINATION");
    auto result =
        gtsam::LevenbergMarquardtOptimizer(graph, initial_value, params)
            .optimize();
    // BUG: 因子定义有问题，优化器没有正常迭代

    // 获取优化结果
    auto optimized = result.at<gtsam::Pose3>(X(0));

    Eigen::Matrix3d R_ros_to_camera = R_camera_to_ros.inverse();
    Eigen::Matrix3d optimized_R =
        R_ros_to_camera *
        tools::rpyToQuaterniond(optimized.rotation().rpy()).matrix();
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
