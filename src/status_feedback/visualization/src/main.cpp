// Copyright (c) 2026 F. All Rights Reserved.
#include "basic/logger.hpp"
#include "configs.hpp"
#include "coord_geometry.hpp"
#include "open3d/geometry/Geometry3D.h"

#include <memory>
#include <open3d/Open3D.h>

#include <cxxopts.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <quill/Backend.h>
#include <quill/LogMacros.h>
#include <quill/backend/ThreadUtilities.h>
#include <rfl.hpp>
#include <rfl/yaml/read.hpp>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <thread>
#include <unordered_map>

constexpr char APP_NAME[] = "rmviz3d";

int main(int argc, char *argv[]) {
  cxxopts::Options options(
      APP_NAME, "rmviz3d is an app for robot self-aiming visualization.");
  options.add_options()("c,config", "Path of config yaml file",
                        cxxopts::value<std::string>()->default_value(
                            "configs/status_feedback/rmviz3d.yaml"))(
      "h,help", "Print usage.");
  auto result = options.parse(argc, argv);
  if (result.count("help")) {
    std::cout << options.help() << std::endl;
    std::exit(EXIT_SUCCESS);
  }
  auto config_path = result["config"].as<std::string>();
  std::ifstream ifs(config_path);
  if (!ifs) {
    std::cerr << "Invalid config path!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  auto config_opt = rfl::yaml::read<fb::VisualizationConfigs>(ifs);
  if (!config_opt.has_value()) {
    std::cerr << "Configuration parsing error!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  fb::VisualizationConfigs config = config_opt.value();

  auto *logger = tools::initAndGetLogger(APP_NAME, config.log_level);
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);

  open3d::visualization::Visualizer visualizer;
  visualizer.CreateVisualizerWindow(APP_NAME, 1920, 1080);

  std::unordered_map<std::string, std::shared_ptr<open3d::geometry::Geometry3D>>
      geometrys;
  fb::CoordGeometry coord_geometry{logger, config.coord_conf, geometrys};

  for (auto &&[name, geometry] : geometrys) {
    visualizer.AddGeometry(geometry);
  }
  while (visualizer.PollEvents()) {
    coord_geometry.update(geometrys);
    for (auto &&[name, geometry] : geometrys) {
      visualizer.UpdateGeometry(geometry);
    }
    visualizer.UpdateRender();
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
  return 0;
}
