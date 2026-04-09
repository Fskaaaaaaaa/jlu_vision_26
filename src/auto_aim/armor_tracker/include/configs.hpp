#pragma once

#include "confs/Basic.hpp"
#include "confs/IceoryxServiceDescription.hpp"
#include "math/ballistic_trajectory.hpp"

#include "quill/core/LogLevel.h"

#include <array>

namespace auto_aim {

struct ArmorObservationNoiseConfig {
  confs::Vector2d pixel_error;
  double tangential_error_m;
  double radial_error_m;
  double height_error_m;
  double yaw_error_rad;
};

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
  ArmorObservationNoiseConfig armor_observation_noise;
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
  double aim_ok_error_m = 0.005;
  double armor_switch_facing_degree_diff_thres;
  // 保持当前选板的当前选板与最正对选板的面对角差的最大阈值
  double iterative_max_facing_angle; // 迭代时允许的最大面对角（否则视为失败）
  double flytime0_distance_offset;   // 用于解算选板预判的飞行时间，一般为负数
};

struct FireControllerConfig {
  double norm_dispersion_yaw_degree; // 两个方向上散布的标准差
  double norm_dispersion_pitch_degree;
  double min_fire_thres_yaw_degree;
  double min_fire_thres_pitch_degree;
};

struct PlannerConfig {
  double dt_sec;                    // MPC更新的间隔时间
  double fail_polling_interval_sec; // 非ontask时轮询task的时间间隔
  int trajectory_half_horizon;      // 生成的瞄准轨迹一半在过去，一半在未来
  bool rk45_yaw0;
  bool iterative_yaw0;
  bool rk45_traj;                // 是否使用解析解
  bool iterative_traj;           // 是否考虑子弹飞行时敌人的运动
  bool enable_aim_center;        // 理论上不需要，电控没调好yaw时启用
  bool consider_gimbal_response; // 用响应曲线(而不是规划曲线)做火控
  double aim_center_vyaw_thres_high;
  double aim_center_vyaw_thres_low;
  double aim_center_distance_high;
  double aim_center_distance_low; // 近到一定程度后就得瞄中心了，不然响应跟不上
  int shoot_offset; // 预判几个MPC帧，在iterative_fly_time时应当为0
  double yaw_offset;
  double pitch_offset;
  int predict_offset_ms; // 补偿拨盘响应延迟
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
  FireControllerConfig fire_ctrl_conf;
};

struct TrackerConfigs {
  quill::LogLevel log_level;
  bool plot_info;
  bool always_on_task;
  bool show_image;
  bool erase_if_not_key_frame;
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
