#pragma once
#include "quill/Logger.h"
#include "single.hpp"
#include "confs/IceoryxServiceDescription.hpp"
// #include "basic/logger.hpp"

#include "confs/CameraParams.hpp"
#include "types/EnemyColor.hpp"
#include "quill/core/LogLevel.h"

#include <string>

namespace auto_buff {
constexpr char APP_NAME[] = "buff_detector";
constexpr char DEFAULT_CONFIG_PATH[] = "configs/auto_buff/buff_detector.yaml";
constexpr char DEFAULT_LOG_PATH[] = "logs/auto_buff";

enum class YOLOVersion {
  YOLOv5,
  YOLOv8,
  YOLO11,
};

struct YOLOConfig {
  std::string device;
  std::string model_path;
  bool use_latency_performancemode; // NOTE: 注意MT时设成false
  float threshold;
  int top_k;
  float nms_threshold;
  float merge_conf_error;
  float merge_min_iou;
};

struct RuneDetectorConfigs {
  quill::LogLevel log_level;
  confs::CameraParams camera_params;

  bool detect_when_idle;
  bool debug_mode;
  bool step_by_step_debug;

  
  types::EnemyColor default_enemy_color;
  std::string camera_name;
  confs::IceoryxServiceDescription runes_topic;
  YOLOConfig yolo_config;
};

class ConfigManager : public Single<ConfigManager> {
  friend class Single<ConfigManager>;
protected:
  ConfigManager();
  ~ConfigManager();

public:
  //初始化
  void init(const std::string config_path,
            const std::string log_path);
  void init();
  void setDebugMode();
  //获取配置和logger
  quill::Logger* logger();
  const RuneDetectorConfigs& configs();

private:
  void check_init();

private:
  quill::Logger* logger_;
  RuneDetectorConfigs configs_;
  bool is_init_ = false;
};
}
