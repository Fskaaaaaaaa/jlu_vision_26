#pragma once

#include "confs/Basic.hpp"
#include "confs/IceoryxServiceDescription.hpp"
#include "math/ballistic_trajectory.hpp"
#include "quill/core/LogLevel.h"

namespace auto_buff {

struct BuffBladeMatchConfig {
  double max_match_distance_m;
  double max_match_roll_diff_degree;
};

// 扇叶到风车本体的约束因子和重投影因子的噪声
struct BuffBladeNoiseConfig {
  confs::Vector2d pixel_error;
  confs::Vector3d position_noise_m;
  double roll_noise_degree;
};

// 风车运动学一致的噪声
struct BuffCenterNoiseConfig {
  confs::Vector3d position_consistency_noise_m;
  double roll_noise_degree;
  double vroll_noise_rad;
  confs::Vector3d position_prior_noise_m;
  double roll_prior_noise_degree;
  double vroll_prior_noise_rad;
};

struct SmallBuffConfig {
  BuffBladeMatchConfig match_conf;
  BuffBladeNoiseConfig blade_conf;
  BuffCenterNoiseConfig center_conf;
  double lost_threshold_sec;
};

struct BigBuffConfig {};

struct TrajectoryConfig {
  tools::ballistic::BallisticConfig ballistic_conf;
  int max_aim_iterate_count;
  double aim_ok_error_m;
  int blade_select_change_count_thres; // 经过几帧的不一样更改叶(有可能是nullopt)
  double fail_polling_interval_sec;
  double min_bullet_speed;
  double max_bullet_speed;
  double default_bullet_speed;
  bool iterative_fly_time;
};

struct TrackerConfigs {
  quill::LogLevel log_level;
  bool always_on_task_small_buff;
  bool always_on_task_big_buff;
  bool plot_info;
  bool show_image;
  bool erase_if_not_key_frame;
  double cmd_pub_dt_sec;
  double tf_query_tolerance_ms;
  std::string camera_name;
  std::string odom_frame_id;
  std::string camera_frame_id; // 这两个用在tf上
  confs::IceoryxServiceDescription buff_blade_topic;
  confs::IceoryxServiceDescription serial_topic;
  SmallBuffConfig small_buff_conf;
  // BigBuffConfig big_buff_conf;
  TrajectoryConfig trajectory_conf;
};

}; // namespace auto_buff
