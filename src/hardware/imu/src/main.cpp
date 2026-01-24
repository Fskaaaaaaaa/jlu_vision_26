// Copyright (c) 2026 F. All Rights Reserved.
#include "basic/logger.hpp"
#include "configs.hpp"
#include "imu.hpp"

#include <chrono>
#include <cxxopts.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iceoryx_posh/runtime/posh_runtime_single_process.hpp>
#include <iox/signal_watcher.hpp>
#include <quill/Backend.h>
#include <quill/LogMacros.h>
#include <quill/backend/ThreadUtilities.h>
#include <rfl.hpp>
#include <rfl/yaml/read.hpp>

#include <fstream>
#include <string>
#include <thread>

// clang-format off
// BUG:   SSSSSSSSSSSSSSS hhhhhhh               iiii          tttt           !!! BBBBBBBBBBBBBBBBB                                          !!! 
// BUG: SS:::::::::::::::Sh:::::h              i::::i      ttt:::t          !!:!!B::::::::::::::::B                                        !!:!!
// BUG: S:::::SSSSSS::::::Sh:::::h               iiii       t:::::t          !:::!B::::::BBBBBB:::::B                                       !:::!
// BUG: S:::::S     SSSSSSSh:::::h                          t:::::t          !:::!BB:::::B     B:::::B                                      !:::!
// BUG: S:::::S             h::::h hhhhh       iiiiiiittttttt:::::ttttttt    !:::!  B::::B     B:::::Buuuuuu    uuuuuu     ggggggggg   ggggg!:::!
// BUG: S:::::S             h::::hh:::::hhh    i:::::it:::::::::::::::::t    !:::!  B::::B     B:::::Bu::::u    u::::u    g:::::::::ggg::::g!:::!
// BUG:  S::::SSSS          h::::::::::::::hh   i::::it:::::::::::::::::t    !:::!  B::::BBBBBB:::::B u::::u    u::::u   g:::::::::::::::::g!:::!
// BUG:   SS::::::SSSSS     h:::::::hhh::::::h  i::::itttttt:::::::tttttt    !:::!  B:::::::::::::BB  u::::u    u::::u  g::::::ggggg::::::gg!:::!
// BUG:     SSS::::::::SS   h::::::h   h::::::h i::::i      t:::::t          !:::!  B::::BBBBBB:::::B u::::u    u::::u  g:::::g     g:::::g !:::!
// BUG:        SSSSSS::::S  h:::::h     h:::::h i::::i      t:::::t          !:::!  B::::B     B:::::Bu::::u    u::::u  g:::::g     g:::::g !:::!
// BUG:             S:::::S h:::::h     h:::::h i::::i      t:::::t          !!:!!  B::::B     B:::::Bu::::u    u::::u  g:::::g     g:::::g !!:!!
// BUG:             S:::::S h:::::h     h:::::h i::::i      t:::::t    tttttt !!!   B::::B     B:::::Bu:::::uuuu:::::u  g::::::g    g:::::g  !!! 
// BUG: SSSSSSS     S:::::S h:::::h     h:::::hi::::::i     t::::::tttt:::::t     BB:::::BBBBBB::::::Bu:::::::::::::::uug:::::::ggggg:::::g      
// BUG: S::::::SSSSSS:::::S h:::::h     h:::::hi::::::i     tt::::::::::::::t !!! B:::::::::::::::::B  u:::::::::::::::u g::::::::::::::::g  !!! 
// BUG: S:::::::::::::::SS  h:::::h     h:::::hi::::::i       tt:::::::::::tt!!:!!B::::::::::::::::B    uu::::::::uu:::u  gg::::::::::::::g !!:!!
// BUG:  SSSSSSSSSSSSSSS    hhhhhhh     hhhhhhhiiiiiiii         ttttttttttt   !!! BBBBBBBBBBBBBBBBB       uuuuuuuu  uuuu    gggggggg::::::g  !!! 
// BUG:                                                                                                                             g:::::g      
// BUG:                                                                                                                 gggggg      g:::::g      
// BUG:                                                                                                                 g:::::gg   gg:::::g      
// BUG:                                                                                                                  g::::::ggg:::::::g      
// BUG:                                                                                                                   gg:::::::::::::g       
// BUG:                                                                                                                     ggg::::::ggg         
// BUG:                                                                                                                        gggggg
// clang-format on
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic
// BUG: 有概率imu串口关不掉/后台程序残留导致重启时iox重名panic

constexpr char APP_NAME[] = "imu";

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
