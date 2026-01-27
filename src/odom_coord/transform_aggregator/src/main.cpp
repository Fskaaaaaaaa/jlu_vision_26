// Copyright (c) 2026 F. All Rights Reserved.
#include "basic/logger.hpp"
#include "configs.hpp"
#include "transform_aggregator.hpp"

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

constexpr char APP_NAME[] = "tf_agg";

int main(int argc, char *argv[]) {
  cxxopts::Options options(
      APP_NAME,
      "tf_agg subcribe {imu_data, imu_name, data}, publish transform "
      "from map to imu and the vel of imu. it also subcribe {imu_control, "
      "imu_name, reset} to reset translation of this transform.");
  options.add_options()("c,config", "Path of config yaml file",
                        cxxopts::value<std::string>()->default_value(
                            "configs/odom_coord/tf_agg.yaml"))(
      "l,log", "Path of log dir",
      cxxopts::value<std::string>()->default_value("logs/tf_agg"))(
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
  auto configs_opt = rfl::yaml::read<tf::TransformAggregatorConfigs>(ifs);
  if (!configs_opt.has_value()) {
    std::cerr << "Configuration parsing error!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  tf::TransformAggregatorConfigs configs = configs_opt.value();

  auto *logger = tools::initAndGetLogger(APP_NAME, configs.log_level, log_path);
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);
  tf::TransformAggregator transform_aggregator{logger, configs};
  iox::waitForTerminationRequest();
  return 0;
}
