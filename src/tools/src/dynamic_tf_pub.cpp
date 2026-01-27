#include "msgs/Header.hpp"
#include "msgs/Transform.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include "quill/LogMacros.h"
#include "transform/dynamic_tf_publisher.hpp"
#include <chrono>

tf::DynamicTransformPublisher::DynamicTransformPublisher(quill::Logger *logger)
    : logger_(logger), tf_pub_({"tf", "dynamic", "data"}) {}

void tf::DynamicTransformPublisher::publishTransform(
    const msgs::Transform &transform, const msgs::Header &header) {
  while (!this->tf_pub_.loan().and_then(
      [&](iox::popo::Sample<msgs::Transform, msgs::Header> &sample) {
        sample.getUserHeader().frame_id = header.frame_id;
        sample.getUserHeader().stamp_ns = header.stamp_ns;
        sample->child_frame_id = transform.child_frame_id;
        sample->rotation = transform.rotation;
        sample->translation = transform.translation;
        sample.publish();
        LOG_TRACE_L1(logger_, "success publish transform from {} to {}.",
                     header.frame_id.c_str(), transform.child_frame_id.c_str());
      })) {
    LOG_TRACE_L1(logger_,
                 "unable publish transform from {} to {}: shm loan failure.",
                 header.frame_id.c_str(), transform.child_frame_id.c_str());
    std::this_thread::yield();
    // NOTE: 需要更多的判断
  }
  return;
}

void tf::DynamicTransformPublisher::publishTransform(
    const Eigen::Isometry3d &transform, const std::string &from,
    const std::string &to, std::chrono::system_clock::time_point stamp) {
  Eigen::Quaterniond r{transform.rotation().matrix()};
  r.normalize();
  Eigen::Vector3d t{transform.translation()};
  msgs::Transform msg{.child_frame_id = {iox::TruncateToCapacity, to.c_str()},
                      .translation{
                          t.x(),
                          t.y(),
                          t.z(),
                      },
                      .rotation{
                          r.x(),
                          r.y(),
                          r.z(),
                          r.w(),
                      }};
  msgs::Header header{
      .frame_id = {iox::TruncateToCapacity, from.c_str()},
      .stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      stamp.time_since_epoch())
                      .count(),
  };
  this->publishTransform(msg, header);
}
