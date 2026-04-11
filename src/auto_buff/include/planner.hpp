#pragma once

#include "math/ballistic_trajectory.hpp"
#include "msgs/AimCommand.hpp"
#include "msgs/GimbalInfo.hpp"

#include <Eigen/Core>
#include <quill/Logger.h>

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "tiny_api.hpp"

struct BuffTrackedTarget {
    int track_index{-1};
    bool predicted{false};
    int model_type{0};
    double fly_time{0.0};
    Eigen::Vector3d position_source{Eigen::Vector3d::Zero()};
    Eigen::Vector3d position_gimbal{Eigen::Vector3d::Zero()};
    Eigen::Vector3d position_odom{Eigen::Vector3d::Zero()};
};

struct BuffTargetSnapshot {
    std::chrono::system_clock::time_point stamp{};
    std::string frame_id;
    std::vector<BuffTrackedTarget> targets;

    [[nodiscard]] bool empty() const { return targets.empty(); }
};

struct BuffAimTrack {
    BuffTrackedTarget current;
    std::optional<BuffTrackedTarget> predicted;
};

struct BuffAimSolution {
    double yaw{0.0};
    double pitch{0.0};
    double fly_time{0.0};
};

struct BuffTrajectoryReference {
    Eigen::MatrixXd state_reference;
    int fallback_sample_count{0};
};

class BuffPlanner {
public:
    explicit BuffPlanner(quill::Logger* logger);

    msgs::AimCommand plan(const BuffTargetSnapshot& snapshot,
                          const msgs::GimbalInfo& gimbal_info);

private:
    [[nodiscard]] std::optional<BuffAimTrack>
    selectTarget(const BuffTargetSnapshot& snapshot);
    [[nodiscard]] std::optional<BuffAimSolution>
    solveAim(const Eigen::Vector3d& position_gimbal, double bullet_speed_mps,
             bool use_rk45) const;
    [[nodiscard]] BuffTrajectoryReference
    buildTrajectoryReference(const BuffAimTrack& track,
                             const msgs::GimbalInfo& gimbal_info,
                             double bullet_speed_mps) const;

    static double wrapAngle(double angle_rad);
    static double resolveBulletSpeed(double measured_speed);

    quill::Logger* logger_;
    tools::ballistic::BallisticTrajectorySolver solver_;
    TinySolver* yaw_solver_{nullptr};
    TinySolver* pitch_solver_{nullptr};
    int trajectory_horizon_{0};
    unsigned int bullet_id_{0};
    std::optional<int> preferred_track_index_;
};

// 声明大符跟踪的规划器接口，姑且用于负责目标选择、弹道解算和瞄准轨迹生成。
