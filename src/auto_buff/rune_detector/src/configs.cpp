#include "configs.hpp"
#include "basic/logger.hpp"

#include <rfl.hpp>
#include <rfl/yaml/read.hpp>
#include "quill/LogMacros.h"

#include <fstream>
#include <iostream>

extern const char APP_NAME[];

auto_buff::ConfigManager::ConfigManager() = default;
auto_buff::ConfigManager::~ConfigManager() = default;

void auto_buff::ConfigManager::init(const std::string config_path,
                                    const std::string log_path) {
  std::ifstream ifs(config_path);
  if (!ifs) {
    std::cerr << "Invalid config path!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  auto configs_opt = rfl::yaml::read<auto_buff::RuneDetectorConfigs>(ifs);
  if (!configs_opt.has_value()) {
    std::cerr << "Configuration parsing error!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  configs_ = configs_opt.value();
  logger_ = tools::initAndGetLogger(APP_NAME, configs_.log_level, log_path);
  LOG_INFO(logger_, "config manager initialized!");
  is_init_ = true;
}

void auto_buff::ConfigManager::init() {
  init(DEFAULT_CONFIG_PATH, DEFAULT_LOG_PATH);
  LOG_ERROR(logger_, "use default config init!");
}

void auto_buff::ConfigManager::check_init() {
  if (!is_init_)
    init();
}
