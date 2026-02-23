#pragma once

#include "confs/IceoryxServiceDescription.hpp"
#include "quill/core/LogLevel.h"
namespace auto_aim {
struct TargetConfig {
  double yaw_error_weight; // 匹配装甲板时，yaw误差的权重
  double yaw_factor_noise;
  double yaw_prior_noise;
  struct {
    double x;
    double y;
    double z;
  } translation_factor_noise;
  struct {
    double x;
    double y;
    double z;
  } translation_prior_noise;
  struct {
    double vx;
    double vy;
    double vz;
  } velocity_factor_noise;

  struct {
    double lost_threshold_sec; // 超时重置因子图的阈值
    double default_radius_a;
    double default_radius_b;
    double default_dz;
    struct {
      double x;
      double y;
      double z;
      double yaw;
    } ra_factor_noise;
    struct {
      double x;
      double y;
      double z;
      double yaw;
    } rbdz_factor_noise;
    double ra_prior_noise;
    double rb_prior_noise;
    double dz_prior_noise;
    double vyaw_factor_noise;
  } robot;

  struct {
    double lost_threshold_sec; // 超时重置因子图的阈值
    double default_radius;
    double default_dz_a;
    double default_dz_b;
    double vyaw_factor_noise;
  } outpost;
};

struct TrajectoryConfig {
  double g = 9.8;
  double k = 0.01903;
  double length_gimbal_to_barrel = 0.107;
  double time_step = 0.0004;
  int max_pitch_iterate_count = 100;
  double min_pitch_error = 0.008;
  double gimbal_pitch_min_degree = -5.0;
  double gimbal_pitch_max_degree = 30.0;
  double half_search_range_degree = 5.0;
  double max_fly_time = 1.0;
  double default_fly_time = 0.1; // 默认假设弹丸飞行10ms,应该用不到
  int max_aim_iterate_count = 20;
  double aim_ok_error_m = 0.005;
};

struct AimerConfig {
  TrajectoryConfig trajectory_conf;
};

struct PlannerConfig {};

struct TrackerConfigs {
  quill::LogLevel log_level;
  bool publish_target_armors;
  bool show_target_armors_reproj_result;
  std::string camera_name;
  std::string camera_frame_id; // 这两个用在tf上
  std::string odom_frame_id;
  confs::IceoryxServiceDescription armors_topic;
  confs::IceoryxServiceDescription serial_topic;
  confs::IceoryxServiceDescription target_armors_publish_topic;
  TargetConfig target_conf;
  AimerConfig aimer_conf;
  PlannerConfig planner_conf;
};
} // namespace auto_aim
