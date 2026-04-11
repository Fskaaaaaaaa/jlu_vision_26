#pragma once

#include "iceoryx_channel.hpp"
#include "planner.hpp"

#include "msgs/AimCommand.hpp"
#include "msgs/GimbalInfo.hpp"
#include "msgs/Header.hpp"
#include "msgs/TaskMode.hpp"
#include "transform/tf_listener.hpp"
#include "types/TaskMode.hpp"

#include <iceoryx_posh/popo/publisher.hpp>
#include <iceoryx_posh/popo/subscriber.hpp>
#include <quill/Logger.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

class BuffTrackerNode {
public:
    explicit BuffTrackerNode(quill::Logger* logger);

private:
    void run();
    void pollTaskMode();
    void pollGimbalInfo();
    void pollDetectorResults();
    void publishAimCommand(const msgs::AimCommand& cmd);
    [[nodiscard]] bool isOnBuffTask() const;
    [[nodiscard]] bool isTargetFresh() const;
    std::optional<BuffTargetSnapshot> transformPacket(
        const TargetCoordsPacket& packet,
        const msgs::Header& header) const;
    void updateSnapshot(std::optional<BuffTargetSnapshot> snapshot);
    void logTransformFailure(const std::string& source_frame_id,
                             const std::string& error_message) const;

    quill::Logger* logger_;
    ResultSubscriber result_sub_;
    iox::popo::Subscriber<msgs::TaskMode, msgs::Header> task_mode_sub_;
    iox::popo::Subscriber<msgs::GimbalInfo, msgs::Header> gimbal_info_sub_;
    iox::popo::Publisher<msgs::AimCommand, msgs::Header> aimcommand_pub_;
    fast_tf::detail::transform_buffer tf_buffer_;
    tf::TransformListener tf_listener_;
    BuffPlanner planner_;
    types::TaskMode current_task_mode_{types::TaskMode::Idle};
    msgs::GimbalInfo latest_gimbal_info_{0, 0, 0, 0, 0, 22};
    mutable std::mutex snapshot_mtx_;
    std::optional<BuffTargetSnapshot> latest_snapshot_;
    mutable std::chrono::steady_clock::time_point last_tf_log_time_{};
    mutable std::string last_tf_error_message_;
    std::jthread loop_thread_;
};

// 声明 buff_tracker 节点，负责订阅检测结果和云台状态、做坐标变换并发布瞄准指令。
