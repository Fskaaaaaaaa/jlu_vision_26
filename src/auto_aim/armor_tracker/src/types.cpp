// Copyright (c) 2026 I Love CCB. All Rights Reserved.
#include "types.hpp"
#include <utility>

auto_aim::Armor::Armor(const types::Armor &armor) {
  this->stamp = armor.stamp;
  this->frame_id = armor.frame_id;
  this->type = armor.type;
  this->color = armor.color;
  this->distance_to_image_center = armor.distance_to_image_center;
  this->orientation = armor.orientation;
  this->position = armor.position;
  this->confidence = armor.confidence;
  this->key_frame = armor.key_frame;
  this->yaw = armor.getRpy()(2);
}

auto_aim::Armor::Armor(const Eigen::Vector3d &center_pos, double center_yaw,
                       double radius_a, ArmorIndex armor_index) {
  if (armor_index == ArmorIndex::_1 || armor_index == ArmorIndex::_3)
    throw ArmorException(
        "Invalid armor index: cannot be _1 or _3 in radius_a constructor");
  auto between_angle = (2 * std::numbers::pi) / 4;
  auto armor_yaw = center_yaw + static_cast<int>(armor_index) * between_angle;
  auto armor_x = center_pos.x() - radius_a * std::cos(armor_yaw);
  auto armor_y = center_pos.y() - radius_a * std::sin(armor_yaw);
  this->position = Eigen::Vector3d{armor_x, armor_y, center_pos.z()};
  this->yaw = armor_yaw;
}

auto_aim::Armor::Armor(const Eigen::Vector3d &center_pos, double center_yaw,
                       double radius_b, double dz, ArmorIndex armor_index) {
  if (armor_index == ArmorIndex::_0 || armor_index == ArmorIndex::_2)
    throw ArmorException(
        "Invalid armor index: cannot be _0 or _2 in radius_b_dz constructor");
  auto between_angle = (2 * std::numbers::pi) / 4;
  auto armor_yaw = center_yaw + static_cast<int>(armor_index) * between_angle;
  auto armor_x = center_pos.x() - radius_b * std::cos(armor_yaw);
  auto armor_y = center_pos.y() - radius_b * std::sin(armor_yaw);
  this->position = Eigen::Vector3d{armor_x, armor_y, center_pos.z() + dz};
  this->yaw = armor_yaw;
}

auto_aim::Armor auto_aim::Armor::fromRobot(const Eigen::Vector3d &center_pos,
                                           double center_yaw, double radius_a,
                                           double radius_b, double _dz,
                                           ArmorIndex armor_index) {
  auto between_angle = (2 * std::numbers::pi) / 4;
  auto armor_yaw = center_yaw + static_cast<int>(armor_index) * between_angle;
  auto [r, dz] =
      (armor_index == ArmorIndex::_0 || armor_index == ArmorIndex::_2)
          ? std::pair{radius_a, 0.0}
          : std::pair{radius_b, _dz};
  auto armor_x = center_pos.x() - radius_b * std::cos(armor_yaw);
  auto armor_y = center_pos.y() - radius_b * std::sin(armor_yaw);
  Armor armor;
  armor.position = Eigen::Vector3d{armor_x, armor_y, center_pos.z() + dz};
  armor.yaw = armor_yaw;
  return armor;
}

auto_aim::Armor auto_aim::Armor::fromOutpost(const Eigen::Vector3d &center_pos,
                                             double center_yaw, double radius,
                                             double dz_a, double dz_b,
                                             ArmorIndex armor_index) {
  if (armor_index == ArmorIndex::_3)
    throw ArmorException{
        "Invalid armor index: cannot be _3 in fromOutpost constructor"};
  auto between_angle = (2 * std::numbers::pi) / 3;
  auto armor_yaw = center_yaw + static_cast<int>(armor_index) * between_angle;
  auto armor_x = center_pos.x() - radius * std::cos(armor_yaw);
  auto armor_y = center_pos.y() - radius * std::sin(armor_yaw);
  auto dz = armor_index == ArmorIndex::_0   ? 0.0
            : armor_index == ArmorIndex::_1 ? dz_a
                                            : dz_b;
  Armor armor;
  armor.position = Eigen::Vector3d{armor_x, armor_y, center_pos.z() + dz};
  armor.yaw = armor_yaw;
  return armor;
}
