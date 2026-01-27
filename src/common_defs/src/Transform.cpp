// Copyright (c) 2026 F. All Rights Reserved.
#include "types/Transform.hpp"
#include "confs/Transform.hpp"

#include <numbers>

types::Transform::Transform(
    const iox::popo::Sample<const msgs::Transform, const msgs::Header>
        &sample) {
  this->parent_frame_id = sample.getUserHeader().frame_id.c_str();
  this->child_frame_id = sample->child_frame_id.c_str();
  this->stamp = std::chrono::time_point<std::chrono::system_clock>{
      std::chrono::nanoseconds{sample.getUserHeader().stamp_ns}};
  this->translation.x() = sample->translation.x;
  this->translation.y() = sample->translation.y;
  this->translation.z() = sample->translation.z;
  this->rotation.x() = sample->rotation.x;
  this->rotation.y() = sample->rotation.y;
  this->rotation.z() = sample->rotation.z;
  this->rotation.w() = sample->rotation.w;
}

Eigen::Isometry3d types::Transform::getIsometry3d() {
  Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
  I.prerotate(this->rotation);
  I.translate(this->translation); // 旋转后的坐标系作为基准来移动
  return I;
}

confs::Vector4d confs::Transform::getQuaterniond() {
  Eigen::AngleAxisd roll_angle(rpy_angle.roll * 2 * std::numbers::pi / 360,
                               Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(rpy_angle.pitch * 2 * std::numbers::pi / 360,
                                Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(rpy_angle.yaw * 2 * std::numbers::pi / 360,
                              Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q{yaw_angle * pitch_angle * roll_angle};
  q.normalize();
  return {.x = q.x(), .y = q.y(), .z = q.z(), .w = q.w()};
}
