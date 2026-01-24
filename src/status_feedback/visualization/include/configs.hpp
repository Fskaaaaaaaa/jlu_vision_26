// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

#include <array>
#include <quill/core/LogLevel.h>
#include <string>
#include <vector>
namespace fb {
struct CoordGeometryConfig {
  double frame_size = 1.;
  std::vector<std::string> frame_ids = {"map", "odom", "gimbal", "camera",
                                        "camera_optical"};
  std::string static_frame_id = "map";
  int tf_query_time_tolerance_ms = 100;
  bool draw_frame_trajectory = true;
  std::string trajectory_frame_id = "odom";
  bool draw_camera_visualization = true;
  std::string camera_frame_id = "camera_optical";
  std::string camera_name = "camera_optical";
};

struct ArmorGeometryConfig {
  std::string path_to_armor_stl;
  std::array<std::string, 3> service_instance_event;
};

struct VisualizationConfigs {
  CoordGeometryConfig coord_conf;
  ArmorGeometryConfig armor_conf;
  // CameraGeometryConfig cam_conf;
  // ArmorDrawerConfig armor_conf;
  // BuffDrawerConfig buff_conf;
  // BulletDrawerConfig bullet_conf;

  quill::LogLevel log_level = quill::LogLevel::Info;
  int render_interval_ms;
};
} // namespace fb
