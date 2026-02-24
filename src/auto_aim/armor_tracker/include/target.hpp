// Copyright (c) 2026 Fr. All Rights Reserved.
#pragma once

#include "configs.hpp"
#include "types.hpp"

#include "quill/Logger.h"
#include "types/ArmorType.hpp"
#include <Eigen/Core>
#include <atomic>
#include <gtsam/nonlinear/ISAM2.h>

#include <chrono>
#include <mutex>
#include <optional>

namespace auto_aim {

class Target {
public:
  Target(quill::Logger *logger, const TargetConfig &config,
         types::ArmorType type);
  // NOTE: tacker_node每帧都传入接受到的所有装甲板，筛选由Target内部自己维护
  std::optional<TargetStatus>
  track(const std::vector<types::Armor> &armors,
        const std::chrono::system_clock::time_point &stamp);
  // NOTE: armor的stamp是detector由image的header转发来的，即是图像采集时间点

  std::pair<std::optional<TargetStatus>, std::chrono::system_clock::time_point>
  getStatusStamp() const;

  types::ArmorType type_;

private:
  void reset();
  void initStatus(const Eigen::Vector3d &armor_pos, double armor_yaw);
  // NOTE: 比较观测和预测，选择误差最小的
  ArmorIndex matchArmor(const Armor &armor, double dt_sec);
  std::optional<TargetStatus> updateRobot(const std::vector<Armor> &armors,
                                          double dt);
  std::optional<TargetStatus> updateOutpost(const std::vector<Armor> &armors,
                                            double dt) {
    // TODO
    return {};
  };
  std::optional<TargetStatus> updateBase(const std::vector<Armor> &armors);

  quill::Logger *logger_;
  TargetConfig config_;
  std::atomic<bool> inited_;
  // NOTE:plan线程调用getStatusStamp只读方法，保证写入时设锁就可以线程安全
  mutable std::mutex status_mtx_;
  TargetStatus status_;
  std::chrono::system_clock::time_point stamp_last_update_;
  std::uint64_t k_;
  gtsam::ISAM2 isam2_;
};

} // namespace auto_aim
