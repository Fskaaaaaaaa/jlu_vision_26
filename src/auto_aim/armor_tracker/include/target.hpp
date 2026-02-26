// Copyright (c) 2026 Fr. All Rights Reserved.
#pragma once

#include "configs.hpp"
#include "types.hpp"

#include "quill/Logger.h"
#include "types/ArmorType.hpp"
#include <Eigen/Core>
#include <atomic>
#include <cstdint>
#include <gtsam/nonlinear/ISAM2.h>

#include <chrono>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <mutex>
#include <optional>

namespace auto_aim {

class Target {
public:
  Target(quill::Logger *logger, const TargetConfig &config,
         types::ArmorType type);
  // NOTE: tacker_node每帧都传入接受到的所有装甲板，筛选由Target内部自己维护
  //  k、status、stamp_last_update的写入只在这个方法里且并发读取只有getStatuStamp。
  //  故只要这两处设锁就是线程安全的了
  std::optional<TargetStatus>
  track(const std::vector<types::Armor> &armors,
        const std::chrono::system_clock::time_point &stamp);
  // NOTE: armor的stamp是detector由image的header转发来的，即是图像采集时间点

  std::pair<std::optional<TargetStatus>, std::chrono::system_clock::time_point>
  getStatusStamp() const;

  types::ArmorType type_;

private:
  void reset();
  void addMotionValuesFactors(gtsam::Values &values,
                              gtsam::NonlinearFactorGraph &graph, double dt,
                              std::uint64_t k) const;
  // NOTE: 注意这个方法本身就是threadsafe的，再设锁会死锁
  void initStatus(const Eigen::Vector3d &armor_pos, double armor_yaw);
  // NOTE: 比较观测和预测，选择误差最小的
  std::pair<ArmorIndex, double> matchArmor(const Armor &armor,
                                           double dt_sec) const;
  // NOTE: 不更新除了isam以外的成员，返回的opt表示是否更新成功
  // statue、k和stamp的更新放到track方法里完成（即是当k非0返回null时reset）
  std::optional<TargetStatus> updateRobot(const std::vector<Armor> &armors,
                                          double dt) const;
  std::optional<TargetStatus> updateOutpost(const std::vector<Armor> &armors,
                                            double dt) const {
    // TODO
    return {};
  };
  std::optional<TargetStatus>
  updateBase(const std::vector<Armor> &armors) const;

  quill::Logger *logger_;
  TargetConfig config_;
  // NOTE:plan线程调用getStatusStamp只读方法，保证写入时设锁就可以线程安全
  mutable std::mutex status_mtx_;
  TargetStatus status_;
  std::chrono::system_clock::time_point stamp_last_update_;
  // 将k=0视为系统未初始化的标志（首帧）
  std::uint64_t k_;
  mutable gtsam::ISAM2 isam2_;
};

} // namespace auto_aim
