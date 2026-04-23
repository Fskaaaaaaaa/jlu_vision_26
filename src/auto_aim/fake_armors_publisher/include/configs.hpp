// Copyright (c) 2026 GuGuGaGa!!!!! All Rights Reserved.
#pragma once

#include "types/Armor.hpp"
#include "types/EnemyColor.hpp"

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
  double tf_query_tolerance_ms;
  types::ArmorType armor_type;
  types::EnemyColor armor_color;
  std::string camera_name;
  std::string camera_frame_id;
  std::string odom_frame_id;
  std::array<double, 2> camera_pos;
  ArmorsPublisherConfig pub_conf;
};

struct ArmorsPublisherConfigs {
  RobotConfig robot_conf;
  quill::LogLevel log_level;
};
} // namespace auto_aim
