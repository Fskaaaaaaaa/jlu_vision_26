#pragma once

#include "confs/IceoryxServiceDescription.hpp"
#include "quill/core/LogLevel.h"

#include <array>

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
  int max_aim_iterate_count = 20;
  double aim_ok_error_m = 0.005;
};

struct PlannerConfig {
  double dt_sec;                    // MPC更新的间隔时间
  double fail_polling_interval_sec; // 非ontask时轮询task的时间间隔
  int trajectory_half_horizon;      // 生成的瞄准轨迹一半在过去，一半在未来
  bool use_analytical_solution;     // 是否使用解析解
  bool iterative_fly_time;          // 是否考虑子弹飞行时敌人的运动
  int shoot_offset; // 预判几个MPC帧，在iterative_fly_time时应当为0
  double yaw_offset;
  double pitch_offset;
  double fire_thresh;
  double max_yaw_acc;
  std::array<double, 2> Q_yaw;
  double R_yaw;
  double max_pitch_acc;
  std::array<double, 2> Q_pitch;
  double R_pitch;
  double max_bullet_speed;
  double default_bullet_speed;
  double min_bullet_speed;
  TrajectoryConfig trajectory_conf;
};

struct TrackerConfigs {
  quill::LogLevel log_level;
  bool debug_mode;
  bool publish_target_armors;
  double tf_query_tolerance_ms;
  std::string camera_name;
  std::string camera_frame_id; // 这两个用在tf上
  std::string odom_frame_id;
  confs::IceoryxServiceDescription armors_sub_topic;
  confs::IceoryxServiceDescription armors_pub_topic;
  confs::IceoryxServiceDescription serial_topic;
  TargetConfig target_conf;
  PlannerConfig planner_conf;
};
} // namespace auto_aim
