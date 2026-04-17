#include "buff_detector_node.hpp"
#include "configs.hpp"

#include <cxxopts.hpp>

#include <string>
#include <iostream>

int main(int argc, char *argv[]) {
    //把默认路径移到了config里
    cxxopts::Options options(auto_buff::APP_NAME, "I'm too lazy to write it.");
    options.add_options()("c,config", "Path of config yaml file",
                          cxxopts::value<std::string>()->default_value(
                              auto_buff::DEFAULT_CONFIG_PATH))(
        "l,log", "Path of log dir",
        cxxopts::value<std::string>()->default_value(auto_buff::DEFAULT_LOG_PATH))(
        "h,help", "Print usage.")("d,debug", "Enable debug mode.");
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
      std::cout << options.help() << std::endl;
      std::exit(EXIT_SUCCESS);
    }
    auto config_path = result["config"].as<std::string>();
    auto log_path = result["log"].as<std::string>();
    /*
    我觉得main函数里的东西太多了，
    遂加入一些小巧思，把config的初始化加到ConfigManager里，
    其它类初始化时自己去ConfigManager里读。
    */
    iox::runtime::PoshRuntime::initRuntime(auto_buff::APP_NAME);

    auto_buff::ConfigManager::instance()->init(config_path, log_path);
    return auto_buff::DetectorNode::instance()->run();;
}