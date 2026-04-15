#pragma once
#include "quill/Logger.h"
#include "single.hpp"
// #include "basic/logger.hpp"

#include "confs/CameraParams.hpp"
#include "types/EnemyColor.hpp"
// #include "confs/IceoryxServiceDescription.hpp"
#include "quill/core/LogLevel.h"

#include <string>

namespace auto_buff {
constexpr char APP_NAME[] = "rune_detector";
constexpr char DEFAULT_CONFIG_PATH[] = "configs/auto_buff/rune_detector.yaml";
constexpr char DEFAULT_LOG_PATH[] = "logs/auto_buff";

struct RuneDetectorConfigs {
  quill::LogLevel log_level;
  confs::CameraParams camera_params;

  bool detect_when_idle;
  
  types::EnemyColor default_enemy_color;
  std::string camera_name;
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
