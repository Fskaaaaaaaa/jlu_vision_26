#pragma once

#include "confs/Basic.hpp"

#include "quill/core/LogLevel.h"
#include <array>
#include <string>

namespace tf {
struct ImuIntegratorConfig {
  double gravity;
  std::array<double, 3> accel_noise;
  std::array<double, 3> gyro_noise;
  std::array<double, 3> accel_bias_rw_sigma;
  std::array<double, 3> gyro_bias_rw_sigma;
  std::array<double, 3> vel_noise;
  double integration_noise;
  double prior_noise_rpy;
  double prior_noise_xyz;
  double bias_noise;
  int update_interval_ms;
  bool use_imu_rpy;
};
struct TransformAggregatorConfigs {
  quill::LogLevel log_level;
  std::string imu_name;
  std::string map_frame_id;
  confs::RpyAngle imu_initial_angle;
  confs::Vector3d imu_initial_trans;
  confs::Vector3d imu_initial_vel;
  ImuIntegratorConfig imu_conf;
};
} // namespace tf
