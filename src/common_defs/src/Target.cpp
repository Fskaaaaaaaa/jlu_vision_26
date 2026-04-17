#include "types/Target.hpp"
#include "msgs/Header.hpp"
#include "msgs/KinematicFunctions.hpp"
#include "msgs/Target.hpp"

#include <numbers>
#include <tuple>
#include <vector>

[[deprecated]]
types::Target::Target(
    const iox::popo::Sample<const msgs::Target, const msgs::Header> &sample)

    // XXX: 这里不应该包含Header部分
    : frame_id(std::string{sample.getUserHeader().frame_id.c_str()}),
      type(static_cast<types::TargetType>(sample->type)) {
  auto limit_radian = [](double angle,
                         std::pair<double, double> range = {-std::numbers::pi,
                                                            std::numbers::pi}) {
    const double low = range.first;
    const double high = range.second;
    const double width = high - low;
    angle = std::fmod(angle - low, width);
    if (angle < 0.0)
      angle += width;   // 处理负数 fmod
    return angle + low; // 回到 (low, high]
  };
  auto rpy_to_quaterniond = [](const Eigen::Vector3d &rpy_angle) {
    Eigen::AngleAxisd roll(rpy_angle.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(rpy_angle.y(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(rpy_angle.z(), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q{yaw * pitch * roll};
    q.normalize();
    return q;
  };
  if (type == TargetType::Base) {
    auto param = sample->parametric_func.get<msgs::BaseKinematicFunc>();
    Eigen::Vector3d translation{
        param->center_xyz.x,
        param->center_xyz.y,
        param->center_xyz.z,
    };
    Eigen::Vector3d rotation{
        param->center_rpy.roll,
        param->center_rpy.pitch,
        param->center_rpy.yaw,
    };
    Eigen::Isometry3d center_pose{Eigen::Isometry3d::Identity()};
    center_pose.pretranslate(translation);
    center_pose.rotate(rpy_to_quaterniond(rotation));
    this->center_pose = center_pose;
    this->kinematic_function =
        [center_pose](double) -> std::vector<Eigen::Isometry3d> {
      return {center_pose};
    };
  } else if (type == TargetType::SmallArmorRobot ||
             type == TargetType::BigArmorRobot) {
    auto param = sample->parametric_func.get<msgs::RobotKinematicFunc>();
    Eigen::Vector3d center_translation{
        param->center_xyz.x,
        param->center_xyz.y,
        param->center_xyz.z,
    };
    Eigen::Vector3d linear_vel{
        param->linear_velocity.x,
        param->linear_velocity.y,
        param->linear_velocity.z,
    };
    Eigen::Vector3d center_rotation{
        param->center_rpy.roll,
        param->center_rpy.pitch,
        param->center_rpy.yaw,
    };
    Eigen::Vector3d angular_vel{
        param->angular_velocity.roll,
        param->angular_velocity.pitch,
        param->angular_velocity.yaw,
    };
    auto [r1, r2, dz] = std::tuple{
        param->radius1,
        param->radius2,
        param->dz,
    };
    Eigen::Isometry3d center_pose{Eigen::Isometry3d::Identity()};
    center_pose.pretranslate(center_translation);
    center_pose.rotate(rpy_to_quaterniond(center_rotation));
    this->center_pose = center_pose;
    this->kinematic_function =
        [center_translation, center_rotation, angular_vel, r1, r2, dz,
         limit_radian,
         rpy_to_quaterniond](double dt) -> std::vector<Eigen::Isometry3d> {
      std::vector<Eigen::Isometry3d> results;
      for (auto &&i : std::array{0, 1, 2, 3}) {
        auto [current_r, curretn_dz] =
            i % 2 == 0 ? std::pair{r1, 0.} : std::pair{r2, dz};
        auto armor_yaw =
            limit_radian(center_rotation.z() + i * std::numbers::pi / 2.);
        Eigen::Vector3d armor_translation{
            center_translation.x() + std::cos(armor_yaw) * current_r,
            center_translation.y() + std::sin(armor_yaw) * current_r,
            center_translation.z() + curretn_dz,
        };
        Eigen::Isometry3d armor_position{Eigen::Isometry3d::Identity()};
        armor_position.pretranslate(armor_translation);
        armor_position.rotate(rpy_to_quaterniond(
            {0, -15. / 360. * 2 * std::numbers::pi, armor_yaw}));
        // NOTE: 这里假设敌方地盘始终与地面平行，故锁定装甲板角度为支架的15度
        results.emplace_back(armor_position);
      }
      return results;
    };
  } else if (type == TargetType::Outpost) {
    // TODO
  } else if (type == TargetType::SmallBuff) {
  } else if (type == TargetType::BigBuff) {
  }
}
