#include "msgs/Armor.hpp"
#include "types/Armor.hpp"
#include "types/EnemyColor.hpp"

types::Armor::Armor(
    const iox::popo::Sample<const msgs::Armor, const msgs::Header> &sample) {
  // this->stamp = std::chrono::time_point<std::chrono::system_clock>{
  //     std::chrono::nanoseconds{sample.getUserHeader().stamp_ns}};
  this->stamp = std::chrono::time_point<std::chrono::system_clock>(
      std::chrono::duration_cast<std::chrono::system_clock::duration>(
          std::chrono::nanoseconds(sample.getUserHeader().stamp_ns)));
  this->frame_id = std::string{sample.getUserHeader().frame_id.c_str()};

  this->distance_to_image_center = sample->distance_to_image_center;
  this->type = static_cast<types::ArmorType>(sample->armor_type);
  this->color = static_cast<types::EnemyColor>(sample->armor_color);

  this->orientation.w() = sample->orientation.w;
  this->orientation.x() = sample->orientation.x;
  this->orientation.y() = sample->orientation.y;
  this->orientation.z() = sample->orientation.z;
  this->position.x() = sample->position.x;
  this->position.y() = sample->position.y;
  this->position.z() = sample->position.z;

  // XXX 好丑陋的方式
  this->key_points.at(static_cast<int>(types::ArmorPointPosition::LeftBottom))
      .x() = sample->left_light.bottom.x;
  this->key_points.at(static_cast<int>(types::ArmorPointPosition::LeftBottom))
      .y() = sample->left_light.bottom.y;
  this->key_points.at(static_cast<int>(types::ArmorPointPosition::LeftTop))
      .x() = sample->left_light.top.x;
  this->key_points.at(static_cast<int>(types::ArmorPointPosition::LeftTop))
      .y() = sample->left_light.top.y;
  this->key_points.at(static_cast<int>(types::ArmorPointPosition::RightTop))
      .x() = sample->right_light.top.x;
  this->key_points.at(static_cast<int>(types::ArmorPointPosition::RightTop))
      .y() = sample->right_light.top.y;
  this->key_points.at(static_cast<int>(types::ArmorPointPosition::RightBottom))
      .x() = sample->right_light.bottom.x;
  this->key_points.at(static_cast<int>(types::ArmorPointPosition::RightBottom))
      .y() = sample->right_light.bottom.y;

  this->confidence = sample->confidence;
  this->key_frame = sample->key_frame;
  this->heart_beat = sample->heart_beat;
}

Eigen::Vector3d types::Armor::getRpy() const {
  // External RPY (fixed axes) from rotation matrix:
  // R = Rz(yaw) * Ry(pitch) * Rx(roll)
  const Eigen::Matrix3d R = this->orientation.matrix();
  const double yaw = std::atan2(R(1, 0), R(0, 0));
  const double pitch = std::atan2(-R(2, 0), std::hypot(R(2, 1), R(2, 2)));
  const double roll = std::atan2(R(2, 1), R(2, 2));
  return {roll, pitch, yaw};
}
