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
}

Eigen::Vector3d types::Armor::getRpy() const {
  return this->orientation.matrix().eulerAngles(0, 1, 2);
}

// msgs::Armor types::Armor::toMsg() {
//   return {.armor_type = static_cast<int>(this->type),
//           .armor_color = static_cast<int>(this->color),
//           .distance_to_image_center = this->distance_to_image_center,
//           .position =
//               {
//                   this->position.x(),
//                   this->position.y(),
//                   this->position.z(),
//               },
//           .orientation = {
//               this->orientation.x(),
//               this->orientation.y(),
//               this->orientation.z(),
//               this->orientation.w(),
//           }};
// }
