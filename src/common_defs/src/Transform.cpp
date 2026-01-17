// Copyright (c) 2026 F. All Rights Reserved.
#include "types/Transform.hpp"

types::Transform::Transform(
    const iox::popo::Sample<const msgs::Transform, const msgs::Header>
        &sample) {
  this->parent_frame_id = sample.getUserHeader().frame_id.c_str();
  this->child_frame_id = sample->child_frame_id.c_str();
  this->stamp = std::chrono::time_point<std::chrono::system_clock>{
      std::chrono::nanoseconds{sample.getUserHeader().stamp_ns}};
  this->tvec.x() = sample->translate.x;
  this->tvec.y() = sample->translate.y;
  this->tvec.z() = sample->translate.z;
  this->quaterniond.x() = sample->quaterniond.x;
  this->quaterniond.y() = sample->quaterniond.y;
  this->quaterniond.z() = sample->quaterniond.z;
  this->quaterniond.w() = sample->quaterniond.w;
}

Eigen::Isometry3d types::Transform::getIsometry3d() {
  Eigen::Isometry3d I = Eigen::Isometry3d::Identity();
  I.prerotate(this->quaterniond);
  I.translate(this->tvec); // 旋转后的坐标系作为基准来移动
  return I;
}
