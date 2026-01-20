// Copyright (c) 2026 F. All Rights Reserved.
#include "basic/logger.hpp"
#include "configs.hpp"
#include "imu.hpp"

#include <chrono>
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
#include <thread>

constexpr char APP_NAME[] = "imu_node";

int main(int argc, char *argv[]) {
  cxxopts::Options options(
      APP_NAME,
      "publish imu data on topic {\"imu_data\", \"imu_name\",\"data\"}");
  options.add_options()("c,config", "Path of config yaml file",
                        cxxopts::value<std::string>()->default_value(
                            "configs/hardware/imu.yaml"))(
      "l,log", "Path of log dir",
      cxxopts::value<std::string>()->default_value("logs/imu"))("h,help",
                                                                "Print usage.");
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
  auto configs_opt = rfl::yaml::read<hardware::ImuConfigs>(ifs);
  if (!configs_opt.has_value()) {
    std::cerr << "Configuration parsing error!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  hardware::ImuConfigs configs = configs_opt.value();

  auto *logger = tools::initAndGetLogger(APP_NAME, configs.log_level, log_path);
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);
  hardware::Imu imu{logger, configs};
  while (!iox::hasTerminationRequested()) {
    imu.publishImuData();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(configs.imu_data_publish_interval_ms));
  }
  return 0;
}
