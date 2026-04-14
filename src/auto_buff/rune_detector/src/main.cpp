#include "basic/logger.hpp"
#include "rune_detector_node.hpp"
#include "configs.hpp"

// #include <cstdlib>
#include <cxxopts.hpp>
// #include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <ostream>
#include <rfl.hpp>
#include <rfl/yaml/read.hpp>

#include <string>
#include <fstream>
#include <iostream>

constexpr char APP_NAME[] = "rune_detector";

int main(int argc, char *argv[]) {
    cxxopts::Options options(APP_NAME, "I'm too lazy to write it.");
    options.add_options()("c,config", "Path of config yaml file",
                          cxxopts::value<std::string>()->default_value(
                              "configs/auto_buff/rune_detector.yaml"))(
        "l,log", "Path of log dir",
        cxxopts::value<std::string>()->default_value("logs/auto_buff"))(
        "h,help", "Print usage.")("d,debug", "Enable debug mode.");
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
    auto configs_opt = rfl::yaml::read<auto_buff::RuneDetectorConfigs>(ifs);
    if (!configs_opt.has_value()) {
      std::cerr << "Configuration parsing error!" << std::endl;
      std::exit(EXIT_FAILURE);
    }

    auto configs = configs_opt.value();
    if (!configs_opt.has_value()) {
      std::cerr << "Configuration parsing error!" << std::endl;
      std::exit(EXIT_FAILURE);
    }

    auto *logger = tools::initAndGetLogger(APP_NAME, configs.log_level, log_path);

    auto_buff::RuneDetectorNode node{logger, configs};
    
    while (true) {
    }
    return 0;
}