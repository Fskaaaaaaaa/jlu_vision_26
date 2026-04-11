#include "tracker_node.hpp"

#include "basic/time_tools.hpp"
#include "iox/signal_watcher.hpp"
#include "parameter.hpp"

#include <Eigen/Geometry>
#include <iceoryx_posh/popo/sample.hpp>
#include <quill/LogMacros.h>

#include <cstring>
#include <stdexcept>
#include <thread>

namespace {

const char *taskModeName(types::TaskMode mode) {
  switch (mode) {
  case types::TaskMode::Idle:
    return "Idle";
  case types::TaskMode::Armor:
    return "Armor";
  case types::TaskMode::SmallBuff:
    return "SmallBuff";
  case types::TaskMode::BigBuff:
    return "BigBuff";
  default:
    return "Unknown";
  }
}

Eigen::Vector3d transformPoint(const Eigen::Isometry3d &transform,
                               const Eigen::Vector3d &point) {
  return transform * point;
}

void appendTarget(std::vector<BuffTrackedTarget> &targets, bool valid,
                  int track_index, bool predicted, int model_type,
                  double fly_time, float x, float y, float z,
                  const Eigen::Isometry3d &odom_from_source,
                  const Eigen::Isometry3d &gimbal_from_source) {
  if (!valid) {
    return;
  }

  BuffTrackedTarget target;
  target.track_index = track_index;
  target.predicted = predicted;
  target.model_type = model_type;
  target.fly_time = fly_time;
  target.position_source = Eigen::Vector3d{x, y, z};
  target.position_odom =
      transformPoint(odom_from_source, target.position_source);
  target.position_gimbal =
      transformPoint(gimbal_from_source, target.position_source);
  targets.push_back(target);
}

bool isHeartbeatPacket(const TargetCoordsPacket &packet) {
  return packet.heart_beat > 0 || (packet.observed_target_count == 0 &&
                                   packet.predicted_target_count == 0);
}

} // namespace

BuffTrackerNode::BuffTrackerNode(quill::Logger *logger)
    : logger_(logger), task_mode_sub_(make_service_desc(par::TASK_MODE_SERVICE,
                                                        par::TASK_MODE_INSTANCE,
                                                        par::TASK_MODE_EVENT)),
      gimbal_info_sub_(make_service_desc(par::GIMBAL_INFO_SERVICE,
                                         par::GIMBAL_INFO_INSTANCE,
                                         par::GIMBAL_INFO_EVENT)),
      aimcommand_pub_(make_service_desc(par::AIM_COMMAND_SERVICE,
                                        par::AIM_COMMAND_INSTANCE,
                                        par::AIM_COMMAND_EVENT)),
      tf_listener_(logger_, tf_buffer_), planner_(logger_) {
  task_mode_sub_.subscribe();
  gimbal_info_sub_.subscribe();
  aimcommand_pub_.offer();
  if (!tf_listener_.init()) {
    throw std::runtime_error("unable to init tf listener");
  }
  loop_thread_ = std::jthread([this] { run(); });
}

void BuffTrackerNode::run() {
  while (!iox::hasTerminationRequested()) {
    pollTaskMode();
    pollGimbalInfo();
    pollDetectorResults();

    msgs::AimCommand cmd{};
    if (isOnBuffTask() && isTargetFresh()) {
      std::optional<BuffTargetSnapshot> snapshot;
      {
        std::scoped_lock lk(snapshot_mtx_);
        snapshot = latest_snapshot_;
      }
      if (snapshot.has_value()) {
        cmd = planner_.plan(*snapshot, latest_gimbal_info_);
      }
    } else {
      cmd.control = false;
    }

    publishAimCommand(cmd);
    std::this_thread::sleep_for(
        std::chrono::duration<double>(par::TRACKER_IDLE_SLEEP_SEC));
  }
}

void BuffTrackerNode::pollTaskMode() {
  auto previous_mode = current_task_mode_;
  task_mode_sub_.take()
      .and_then([&](const iox::popo::Sample<const msgs::TaskMode,
                                            const msgs::Header> &sample) {
        current_task_mode_ = static_cast<types::TaskMode>(sample->mode);
      })
      .or_else([&](auto) {});
  while (task_mode_sub_.take().and_then(
      [&](const iox::popo::Sample<const msgs::TaskMode, const msgs::Header>
              &sample) {
        current_task_mode_ = static_cast<types::TaskMode>(sample->mode);
      })) {
  }
  if (current_task_mode_ != previous_mode) {
    LOG_INFO(logger_, "task mode -> {}", taskModeName(current_task_mode_));
  }
}

void BuffTrackerNode::pollGimbalInfo() {
  gimbal_info_sub_.take()
      .and_then([&](const iox::popo::Sample<const msgs::GimbalInfo,
                                            const msgs::Header> &sample) {
        latest_gimbal_info_ = *sample;
      })
      .or_else([&](auto) {});
  while (gimbal_info_sub_.take().and_then(
      [&](const iox::popo::Sample<const msgs::GimbalInfo, const msgs::Header>
              &sample) { latest_gimbal_info_ = *sample; })) {
  }
}

void BuffTrackerNode::pollDetectorResults() {
  TargetCoordsPacket packet;
  msgs::Header header;
  while (result_sub_.take(packet, header)) {
    updateSnapshot(transformPacket(packet, header));
  }
}

void BuffTrackerNode::publishAimCommand(const msgs::AimCommand &cmd) {
  aimcommand_pub_.loan()
      .and_then([&](iox::popo::Sample<msgs::AimCommand, msgs::Header> &sample) {
        sample.getUserHeader().frame_id = {iox::cxx::TruncateToCapacity,
                                           par::ODOM_FRAME_ID,
                                           std::strlen(par::ODOM_FRAME_ID)};
        sample.getUserHeader().stamp_ns = tools::getTimeNowNanoSec();
        sample->control = cmd.control;
        // TODO:
        // sample->fire = cmd.fire;
        sample->target_yaw = cmd.target_yaw;
        sample->target_pitch = cmd.target_pitch;
        sample->yaw = cmd.yaw;
        sample->yaw_vel = cmd.yaw_vel;
        sample->yaw_acc = cmd.yaw_acc;
        sample->pitch = cmd.pitch;
        sample->pitch_vel = cmd.pitch_vel;
        sample->pitch_acc = cmd.pitch_acc;
        sample->bullet_id = cmd.bullet_id;
        sample.publish();
      })
      .or_else(
          [&](auto) { LOG_WARNING(logger_, "Fail to publish AimCommand!"); });
}

bool BuffTrackerNode::isOnBuffTask() const {
  return current_task_mode_ == types::TaskMode::SmallBuff ||
         current_task_mode_ == types::TaskMode::BigBuff;
}

bool BuffTrackerNode::isTargetFresh() const {
  std::scoped_lock lk(snapshot_mtx_);
  if (!latest_snapshot_.has_value()) {
    return false;
  }
  const auto age =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::system_clock::now() - latest_snapshot_->stamp)
          .count();
  return age <= par::TRACKER_LOST_THRESHOLD_SEC &&
         !latest_snapshot_->targets.empty();
}

std::optional<BuffTargetSnapshot>
BuffTrackerNode::transformPacket(const TargetCoordsPacket &packet,
                                 const msgs::Header &header) const {
  const std::string source_frame_id = header.frame_id.c_str();
  if (source_frame_id.empty()) {
    return std::nullopt;
  }

  const auto stamp = tools::nanoSecToChronoPoint(header.stamp_ns);
  if (isHeartbeatPacket(packet)) {
    BuffTargetSnapshot snapshot;
    snapshot.stamp = stamp;
    snapshot.frame_id = source_frame_id;
    return snapshot;
  }

  try {
    const Eigen::Isometry3d odom_from_source =
        tf_buffer_.get(par::ODOM_FRAME_ID, source_frame_id, stamp,
                       std::chrono::nanoseconds{static_cast<int64_t>(
                           par::TF_QUERY_TOLERANCE_MS * 1e6)});
    const Eigen::Isometry3d gimbal_from_source =
        tf_buffer_.get(par::GIMBAL_FRAME_ID, source_frame_id, stamp,
                       std::chrono::nanoseconds{static_cast<int64_t>(
                           par::TF_QUERY_TOLERANCE_MS * 1e6)});

    BuffTargetSnapshot snapshot;
    snapshot.stamp = stamp;
    snapshot.frame_id = source_frame_id;
    appendTarget(snapshot.targets, packet.valid0 > 0.5f, 0, false,
                 packet.model_type0, 0.0, packet.x0, packet.y0, packet.z0,
                 odom_from_source, gimbal_from_source);
    appendTarget(snapshot.targets, packet.pred_valid0 > 0.5f, 0, true,
                 packet.model_type0, packet.fly_time0, packet.pred_x0,
                 packet.pred_y0, packet.pred_z0, odom_from_source,
                 gimbal_from_source);
    appendTarget(snapshot.targets, packet.valid1 > 0.5f, 1, false,
                 packet.model_type1, 0.0, packet.x1, packet.y1, packet.z1,
                 odom_from_source, gimbal_from_source);
    appendTarget(snapshot.targets, packet.pred_valid1 > 0.5f, 1, true,
                 packet.model_type1, packet.fly_time1, packet.pred_x1,
                 packet.pred_y1, packet.pred_z1, odom_from_source,
                 gimbal_from_source);
    return snapshot;
  } catch (const std::exception &e) {
    logTransformFailure(source_frame_id, e.what());
    return std::nullopt;
  }
}

void BuffTrackerNode::updateSnapshot(
    std::optional<BuffTargetSnapshot> snapshot) {
  std::scoped_lock lk(snapshot_mtx_);
  latest_snapshot_ = std::move(snapshot);
}

void BuffTrackerNode::logTransformFailure(
    const std::string &source_frame_id,
    const std::string &error_message) const {
  const auto now = std::chrono::steady_clock::now();
  if (error_message != last_tf_error_message_ ||
      now - last_tf_log_time_ >= std::chrono::seconds(1)) {
    LOG_ERROR(logger_, "tf from {} to {} or {} failed: {}", source_frame_id,
              par::ODOM_FRAME_ID, par::GIMBAL_FRAME_ID, error_message);
    last_tf_log_time_ = now;
    last_tf_error_message_ = error_message;
  }
}
