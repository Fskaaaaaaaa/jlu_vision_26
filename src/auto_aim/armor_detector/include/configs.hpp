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
  bool outpost_is_small_armor;
};

struct DetectorConfigs {
  quill::LogLevel log_level;
  confs::CameraParams camera_params;
  // NOTE: 注意模式切换回自瞄时，要给相机发送更改参数的请求
  bool use_muti_thread;
  int queue_size;
  bool detect_when_idle;
  bool use_pca;
  bool show_detect_result;   // detector直出的结果,红色
  bool show_optimize_result; // PCA、BA后的结果，绿色
  bool show_pnp_result;      // 绘制PNP两个解的坐标轴
  bool plot_pnp_result;      // 将PNP结果发到PlotJuggler
  bool all_is_three;
  bool step_by_step_debug;
  types::EnemyColor default_enemy_color;
  bool update_enemy_color; // 是否从裁判系统更新敌方颜色
  std::string camera_name; // NOTE: 这些话题都是固定的，只要相机名称就够了
  confs::IceoryxServiceDescription armors_topic;
  YOLOVersion yolo_version;
  YOLOConfig yolo_conf;
  LightCornerCorrectorConfig pca_conf;
  PnPConfig pnp_conf;
};

} // namespace auto_aim
