// Copyright (c) 2026 FengHongrui. All Rights Reserved.
#include "basic/logger.hpp"
#include "configs.hpp"
#include "tracker_node.hpp"

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

constexpr char APP_NAME[] = "armor_tracker";

int main(int argc, char *argv[]) {
  cxxopts::Options options(
      APP_NAME,
      "Armor Tracker Node, subscribe armors published by detector, track "
      "targets and plan aims. publish aimcommand to serial node.");
  options.add_options()("c,config", "Path of config yaml file",
                        cxxopts::value<std::string>()->default_value(
                            "configs/auto_aim/armor_tracker.yaml"))(
      "l,log", "Path of log dir",
      cxxopts::value<std::string>()->default_value("logs/armor_tracker"))(
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
  auto configs_opt = rfl::yaml::read<auto_aim::TrackerConfigs>(ifs);
  if (!configs_opt.has_value()) {
    std::cerr << "Configuration parsing error!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  auto_aim::TrackerConfigs configs = configs_opt.value();

  auto *logger = tools::initAndGetLogger(APP_NAME, configs.log_level, log_path);
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);
  auto_aim::TrackerNode node{logger, configs};
  iox::waitForTerminationRequest();
  return 0;
}
