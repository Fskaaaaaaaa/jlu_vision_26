// clang-format off
// BUG: | |  | |                (_)           | |       (_) |                        | (_)           | |
// BUG: | |  | | __ _ _ __ _ __  _ _ __   __ _| | __   ___| |__   ___    ___ ___   __| |_ _ __   __ _| |
// BUG: | |/\| |/ _` | '__| '_ \| | '_ \ / _` | | \ \ / / | '_ \ / _ \  / __/ _ \ / _` | | '_ \ / _` | |
// BUG: \  /\  / (_| | |  | | | | | | | | (_| |_|  \ V /| | |_) |  __/ | (_| (_) | (_| | | | | | (_| |_|
// BUG:  \/  \/ \__,_|_|  |_| |_|_|_| |_|\__, (_)   \_/ |_|_.__/ \___|  \___\___/ \__,_|_|_| |_|\__, (_)
// BUG:                                   __/ |                                                  __/ |  
// BUG:                                  |___/                                                  |___/

// clang-format on
#include "basic/logger.hpp"
#include "configs.hpp"
#include "robot.hpp"

#include <atomic>
#include <cxxopts.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <iceoryx_posh/runtime/posh_runtime.hpp>
#include <iox/signal_watcher.hpp>
#include <optional>
#include <ostream>
#include <quill/Backend.h>
#include <quill/LogMacros.h>
#include <quill/backend/ThreadUtilities.h>
#include <rfl.hpp>
#include <rfl/yaml/read.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

constexpr char APP_NAME[] = "fake_ap";
using namespace ftxui;
int main(int argc, char *argv[]) {
  cxxopts::Options options(APP_NAME, "fake armor detector result publisher.");
  options.add_options()("c,config", "Path of config yaml file",
                        cxxopts::value<std::string>()->default_value(
                            "configs/auto_aim/fake_ap.yaml"))(
      "l,log", "Path of log dir",
      cxxopts::value<std::string>()->default_value("logs/fake_ap"))(
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
  auto configs_opt = rfl::yaml::read<auto_aim::ArmorsPublisherConfigs>(ifs);
  if (!configs_opt.has_value()) {
    std::cerr << "Configuration parsing error!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  auto_aim::ArmorsPublisherConfigs configs = configs_opt.value();
  auto *logger = tools::initAndGetLogger(APP_NAME, configs.log_level, log_path);
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);

  constexpr int box_w = 25;
  constexpr int box_h = 25;
  constexpr int cx = box_w / 2;
  constexpr int cy = box_h / 2;
  constexpr float radius = cx;
  int knob_x = cx;
  int knob_y = cy;
  bool dragging = false;
  auto_aim::RobotContrl contrl;
  auto_aim::Robot robot{logger, configs.robot_conf, contrl};
  auto screen = ScreenInteractive::TerminalOutput();

  Component joystick = Renderer([&]() -> Element {
    std::vector<std::string> grid(box_h, std::string(box_w, ' '));
    for (int x = 0; x < box_w; ++x) {
      grid[0][x] = '-';
      grid[box_h - 1][x] = '-';
    }
    for (int y = 0; y < box_h; ++y) {
      grid[y][0] = '|';
      grid[y][box_w - 1] = '|';
    }
    grid[0][0] = grid[0][box_w - 1] = grid[box_h - 1][0] =
        grid[box_h - 1][box_w - 1] = '+';
    grid[cy][cx] = '+';
    grid[knob_y][knob_x] = '*';
    Elements lines;
    for (auto &row : grid)
      lines.push_back(text(row));
    Elements joystick_with_values;
    joystick_with_values.push_back(vbox(std::move(lines)));
    Elements values;
    values.push_back(
        text("x = " + std::to_string(contrl.traction_direction.load().x)));
    values.push_back(
        text("y = " + std::to_string(contrl.traction_direction.load().y)));
    values.push_back(text(
        "robot pose = " +
        (std::ostringstream{} << robot.getState().position.transpose()).str()));
    values.push_back(
        text("vel : " + (std::ostringstream{}
                         << robot.getState().linear_velocity.transpose())
                            .str()));
    values.push_back(
        text("acc : " + (std::ostringstream{}
                         << robot.getState().linear_acceleration.transpose())
                            .str()));
    values.push_back(text("yaw : " + std::to_string(robot.getState().yaw)));
    values.push_back(
        text("vyaw : " + std::to_string(robot.getState().vel_yaw)));
    values.push_back(
        text("ayaw : " + std::to_string(robot.getState().acc_yaw)));
    values.push_back(text(" "));
    values.push_back(text("Drag the joystick to move robot"));
    values.push_back(text("Prase r to reset robot"));
    values.push_back(text("Prase s to start/stop spin"));
    values.push_back(text("Prase space to change spin direction"));
    joystick_with_values.push_back(vbox(std::move(values)));
    return hbox(std::move(joystick_with_values));
  });
  joystick |= CatchEvent([&](Event e) {
    if (e == Event::Character('r')) {
      robot.resetStatus();
      LOG_DEBUG(logger, "robot reset!");
      return true;
    }
    if (e == Event::Character('s')) {
      if (contrl.spin_status.load().has_value()) {
        contrl.spin_status.store(std::nullopt);
        LOG_DEBUG(logger, "robot stop spin!");
      } else {
        contrl.spin_status.store(true);
        LOG_DEBUG(logger, "robot start spin!");
      }
      return true;
    }
    if (e == Event::Character(' ')) {
      if (!contrl.spin_status.load().has_value())
        return false;
      contrl.spin_status.store(!contrl.spin_status.load().value());
      return true;
    }
    if (!e.is_mouse())
      return false;
    auto m = e.mouse();
    if (m.button == Mouse::Left && m.motion == Mouse::Pressed) {
      dragging = true;
      return true;
    }
    if (dragging && m.motion == Mouse::Moved) {
      int x = std::clamp(m.x, 1, box_w - 2);
      int y = std::clamp(m.y, 1, box_h - 2);
      float dx = x - cx;
      float dy = y - cy;
      float len = std::sqrt(dx * dx + dy * dy);
      if (len > radius) {
        dx *= radius / len;
        dy *= radius / len;
      }
      knob_x = static_cast<int>(cx + dx);
      knob_y = static_cast<int>(cy + dy);
      contrl.traction_direction.store({dx / radius, -dy / radius});
      screen.PostEvent(Event::Custom);
      return true;
    }
    if (m.motion == Mouse::Released) {
      dragging = false;
      knob_x = cx;
      knob_y = cy;
      contrl.traction_direction.store({0, 0});
      screen.PostEvent(Event::Custom);
      return true;
    }
    return false;
  });
  screen.Loop(joystick);
}
