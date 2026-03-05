#pragma once

#include "confs/Basic.hpp"
#include "confs/IceoryxServiceDescription.hpp"

#include "math/ballistic_trajectory.hpp"
#include "quill/core/LogLevel.h"

#include <array>

namespace auto_aim {

struct RobotConfig {
  double max_match_distance_m;
  double max_match_yaw_diff_degree;
  double lost_threshold_sec; // 超时重置因子图的阈值
  // 先验噪声
  double yaw_prior_noise;
  double vyaw_prior_noise;
  confs::Vector3d translation_prior_noise;
  confs::Vector3d velocity_prior_noise;
  // 因子噪声
  double yaw_factor_noise;
  double vyaw_factor_noise;
  confs::Vector3d translation_factor_noise;
  confs::Vector3d velocity_factor_noise;
  double radius_prior_noise;
  double default_radius;
  double radius_min;
  double radius_max;
  double dz_prior_noise;
  double default_dz;
  confs::Vector4d obs_factor_noise;
};

struct OutpostConfig {
  double lost_threshold_sec; // 超时重置因子图的阈值
  double default_radius;
  double default_dz_a;
  double default_dz_b;
};

struct TrajectoryConfig {
  tools::ballistic::BallisticConfig ballistic_conf;
  int max_aim_iterate_count = 20;
  int max_aim_switch_armor_count;
  double aim_ok_error_m = 0.005;
};

struct PlannerConfig {
  double dt_sec;                    // MPC更新的间隔时间
  double fail_polling_interval_sec; // 非ontask时轮询task的时间间隔
  int trajectory_half_horizon;      // 生成的瞄准轨迹一半在过去，一半在未来
  bool rk45_yaw0;
  bool iterative_yaw0;
  bool rk45_traj;      // 是否使用解析解
  bool iterative_traj; // 是否考虑子弹飞行时敌人的运动
  int shoot_offset;    // 预判几个MPC帧，在iterative_fly_time时应当为0
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
  bool plot_info;
  bool always_on_task;
  bool show_image;
  double tf_query_tolerance_ms;
  std::string camera_name;
  std::string camera_frame_id; // 这两个用在tf上
  std::string odom_frame_id;
  confs::IceoryxServiceDescription armors_sub_topic;
  confs::IceoryxServiceDescription serial_topic;
  RobotConfig robot_conf;
  PlannerConfig planner_conf;
};
} // namespace auto_aim
