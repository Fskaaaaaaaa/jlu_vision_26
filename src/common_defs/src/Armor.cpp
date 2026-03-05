#include "msgs/Armor.hpp"
#include "types/Armor.hpp"
#include "types/EnemyColor.hpp"

types::Armor::Armor(
    const iox::popo::Sample<const msgs::Armor, const msgs::Header> &sample) {
  this->stamp = std::chrono::time_point<std::chrono::system_clock>{
      std::chrono::nanoseconds{sample.getUserHeader().stamp_ns}};
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
  this->confidence = sample->confidence;
  this->key_frame = sample->key_frame;
  this->heart_beat = sample->heart_beat;
}

Eigen::Vector3d types::Armor::getRpy() const {
  Eigen::Vector3d ypr = this->orientation.matrix().eulerAngles(2, 1, 0);
  return {ypr.z(), ypr.y(), ypr.x()};
}
