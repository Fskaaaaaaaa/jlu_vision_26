#pragma once

#include "msgs/Armor.hpp"
#include "msgs/Header.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <gtsam/inference/Symbol.h>

namespace types {
enum class ArmorType {
  One,
  Two,
  Three,
  Four,
  Sentry,
  Outpost,
  Base,
  Negative,
};
enum class ArmorColor {
  Red,
  Blue,
  Extinguished,
};
struct Armor {
  Armor(const iox::popo::Sample<const msgs::Armor, const msgs::Header> &sample);
  Eigen::Vector3d getRpy();
  std::chrono::system_clock::time_point stamp;
  std::string frame_id;
  ArmorType type;
  ArmorColor color;
  double distance_to_image_center;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
};
} // namespace types
