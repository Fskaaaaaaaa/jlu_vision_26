// Copyright (c) 2026 GuGuGaGa!!!!! All Rights Reserved.
#pragma once

#include "types/Armor.hpp"

#include "quill/core/LogLevel.h"

#include <array>
#include <string>

namespace auto_aim {
struct RobotPhysicsConfig {
  double r1;
  double r2;
  double dz;
  double traction;
  double drag_coefficient;
  double gravity;
  types::ArmorType armor_type;
  types::ArmorColor armor_color;
};

struct ArmorsPublisherConfig {
  std::array<std::string, 3> service_instance_event;
  int publish_interval_ms;
};

struct ArmorsPublisherConfigs {
  RobotPhysicsConfig robot_conf;
  ArmorsPublisherConfig pub_conf;
  quill::LogLevel log_level;
};
} // namespace auto_aim
