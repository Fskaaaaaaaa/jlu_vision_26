// Copyright (c) 2026 Mian. All Rights Reserved.
#include "basic/logger.hpp"
#include "configs.hpp"
#include "detector_node.hpp"

#include <cxxopts.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iox/signal_watcher.hpp>
#include <quill/Backend.h>
#include <quill/LogMacros.h>
#include <quill/backend/ThreadUtilities.h>
#include <rfl.hpp>
#include <rfl/yaml/read.hpp>

#include <fstream>
#include <string>

constexpr char APP_NAME[] = "armor_detector";

int main(int argc, char *argv[]) {
  cxxopts::Options options(
      APP_NAME, "Armor detector node. Subscribe image and task mode, publish "
                "detected aromrs on topic {armors, detector, data}. Send "
                "camera param change request when get on task.");
  options.add_options()("c,config", "Path of config yaml file",
                        cxxopts::value<std::string>()->default_value(
                            "configs/auto_aim/armor_detector.yaml"))(
      "l,log", "Path of log dir",
      cxxopts::value<std::string>()->default_value("logs/armor_detector"))(
      "h,help", "Print usage.");
  auto result = options.parse(argc, argv);
  if (result.count("help")) {
    std::cout << options.help() << std::endl;
    std::exit(EXIT_SUCCESS);
  }
  auto config_path = result["config"].as<std::string>();
  auto log_path = result["log"].as<std::string>();
  std::ifstream ifs(config_path);
  if (!ifs) {
    std::cerr << "Invalid config path!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  auto configs_opt = rfl::yaml::read<auto_aim::DetectorConfigs>(ifs);
  if (!configs_opt.has_value()) {
    std::cerr << "Configuration parsing error!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  auto_aim::DetectorConfigs configs = configs_opt.value();

  auto *logger = tools::initAndGetLogger(APP_NAME, configs.log_level, log_path);
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);
  auto_aim::DetectorNode node{logger, configs};
  iox::waitForTerminationRequest();
  return 0;
}
