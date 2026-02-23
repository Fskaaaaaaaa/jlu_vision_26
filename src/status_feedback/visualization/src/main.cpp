// Copyright (c) 2026 F. All Rights Reserved.
#include "armor_geometry.hpp"
#include "basic/logger.hpp"
#include "configs.hpp"
#include "coord_geometry.hpp"
#include "target_geometry.hpp"

#include <open3d/Open3D.h>

#include <cxxopts.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <quill/Backend.h>
#include <quill/LogMacros.h>
#include <quill/backend/ThreadUtilities.h>
#include <rfl.hpp>
#include <rfl/yaml/read.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <thread>

constexpr char APP_NAME[] = "rmviz3d";

int main(int argc, char *argv[]) {
  cxxopts::Options options(
      APP_NAME, "rmviz3d is an app for robot self-aiming visualization.");
  options.add_options()("c,config", "Path of config yaml file",
                        cxxopts::value<std::string>()->default_value(
                            "configs/status_feedback/rmviz3d.yaml"))(
      "l,log", "Path of log dir",
      cxxopts::value<std::string>()->default_value("logs/rmviz3d"))(
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
  auto config_opt = rfl::yaml::read<fb::VisualizationConfigs>(ifs);
  if (!config_opt.has_value()) {
    std::cerr << "Configuration parsing error!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  fb::VisualizationConfigs config = config_opt.value();

  auto *logger = tools::initAndGetLogger(APP_NAME, config.log_level, log_path);
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);

  open3d::visualization::Visualizer visualizer;
  visualizer.CreateVisualizerWindow(APP_NAME, 1920, 1080);

  std::set<std::shared_ptr<open3d::geometry::Geometry3D>> geometry_ptrs;
  fb::CoordGeometry coord_geometry{logger, config.coord_conf, geometry_ptrs};
  fb::ArmorGeometry arrmor_geometry{logger, config.armor_conf};
  // fb::TargetGeometry target_geometry{logger, config.target_conf};

  for (auto &&geometry : geometry_ptrs)
    visualizer.AddGeometry(geometry);

  while (visualizer.PollEvents()) {
    auto gps_before_update = geometry_ptrs;
    // NOTE: 各个组在这里update
    arrmor_geometry.update(geometry_ptrs);
    coord_geometry.update(geometry_ptrs);
    // target_geometry.update(geometry_ptrs);

    // 添加新增的
    std::set<std::shared_ptr<open3d::geometry::Geometry3D>> increases;
    std::ranges::set_difference(geometry_ptrs, gps_before_update,
                                std::inserter(increases, increases.begin()));
    for (auto &&increased_gp : increases)
      visualizer.AddGeometry(increased_gp);

    // 去掉删除的
    std::set<std::shared_ptr<open3d::geometry::Geometry3D>> decreases;
    std::ranges::set_difference(gps_before_update, geometry_ptrs,
                                std::inserter(decreases, decreases.begin()));
    for (auto &&decreased_gp : decreases)
      visualizer.RemoveGeometry(decreased_gp);

    // 对现有的进行更新
    for (auto &&geometry : geometry_ptrs)
      visualizer.UpdateGeometry(geometry);
    LOG_TRACE_L1(logger, "render geometry vector size: {}",
                 geometry_ptrs.size());
    visualizer.UpdateRender();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(config.render_interval_ms));
  }
  return 0;
}
