// Copyright (c) 2026 GuGuGaGa!!!!! All Rights Reserved.
#pragma once

#include "types/Armor.hpp"

#include "quill/core/LogLevel.h"

#include <array>
#include <string>

namespace auto_aim {

struct ArmorsPublisherConfig {
  std::array<std::string, 3> service_instance_event;
  std::string frame_id;
};

struct RobotConfig {
  double r1;
  double r2;
  double dz;
  double drag_linear_acceleration;
  double drag_angular_acceleration;
  double power_linear_acceleration;
  double power_angular_acceleration;
  int time_step_ms;
  bool hidden_invisible_armors;
  double facing_armor_cos_incidence;
  types::ArmorType armor_type;
  types::ArmorColor armor_color;
  std::array<double, 2> camera_pos;
  ArmorsPublisherConfig pub_conf;
};

struct ArmorsPublisherConfigs {
  RobotConfig robot_conf;
  quill::LogLevel log_level;
};
} // namespace auto_aim
