#pragma once

#include "confs/CameraParams.hpp"
#include "confs/IceoryxServiceDescription.hpp"
#include "types/EnemyColor.hpp"

#include "quill/core/LogLevel.h"

#include <string>

namespace auto_aim {

struct YOLOConfig {
  std::string device;
  std::string model_path;
  bool use_latency_performancemode; // NOTE: 注意MT时设成false
  float score_thresh = 0.7;         // 网络输出是否进入候选
  float nms_iou_thresh = 0.3;       // 候选之间是否合并
  float accept_thresh = 0.8;        // 是否作为最终目标
};
enum class YOLOVersion {
  YOLOv5,
  YOLOv8,
  YOLO11,
};

struct LightParams {
  // width / height
  double min_ratio;
  double max_ratio;
  // vertical angle
  double max_angle;
  // judge color
  int color_diff_thresh;
};
struct ArmorParams {
  double min_light_ratio;
  // light pairs distance
  double min_small_center_distance;
  double max_small_center_distance;
  double min_large_center_distance;
  double max_large_center_distance;
  // horizontal angle
  double max_angle;
};
struct TraditionalConfig {
  int binary_thres;
  types::EnemyColor detect_color;
  LightParams light_params;
  ArmorParams armor_params;
};

struct LightCornerCorrectorConfig {
  int pass_optimize_lightbar_width = 3;
  float normalize_max_brightness = 25.;
  float lightbar_min_mean_brightness = 0.; // 用于排除yolo语义补全的棍母灯条
  float padding_scale = 0.07;
  float search_start_ratio = 0.8 / 2.;
  float search_end_ratio = 1.2 / 2.;
};

struct PnPConfig {
  bool use_generic_mode;
  double project_error_ratio_thres;
  double roll_thres_degree;
  float frame_axes_length;
  int frame_axes_circle_radius;
  int frame_axes_circle_thickness;
};
struct BaConfig {
  // TODO
};

struct DetectorConfigs {
  quill::LogLevel detector_log_level;
  quill::LogLevel solver_log_level;
  confs::CameraParams camera_params;
  // NOTE: 注意模式切换回自瞄时，要给相机发送更改参数的请求
  bool show_detect_result_image;
  bool show_pnp_result_image;
  bool use_muti_thread;
  int queue_size;
  confs::IceoryxServiceDescription image_topic;
  confs::IceoryxServiceDescription camera_param_topic;
  confs::IceoryxServiceDescription armors_topic;
  YOLOVersion yolo_version;
  YOLOConfig yolo_conf;
  TraditionalConfig trad_conf;
  LightCornerCorrectorConfig pca_conf;
};

} // namespace auto_aim
