// Copyright (c) 2026 F'fprintf(stderr, "\n");. All Rights Reserved.
#include "transform_aggregator.hpp"
#include "basic/time_tools.hpp"
#include "configs.hpp"
#include "iceoryx_posh/internal/popo/base_subscriber.hpp"
#include "imu_integrator.hpp"
#include "math/angle_tools.hpp"
#include "msgs/Basic.hpp"
#include "msgs/Header.hpp"
#include "msgs/ImuData.hpp"
#include "msgs/Transform.hpp"
#include "types/ImuData.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include "iox/string.hpp"
#include "quill/LogMacros.h"
#include "quill/core/ThreadContextManager.h"

#include <cmath>
#include <cstdlib>
#include <sstream>

tf::TransformAggregator::TransformAggregator(
    quill::Logger *logger, const TransformAggregatorConfigs &configs)
    : logger_(logger), configs_(configs),
      imu_integrator_(logger, configs_.imu_conf),
      imu_data_sub_({"imu_data",
                     {iox::TruncateToCapacity, configs.imu_name.c_str()},
                     "data"},
                    {1, 0}),
      imu_reset_sub_({"imu_control",
                      {iox::TruncateToCapacity, configs.imu_name.c_str()},
                      "reset"}),
      tf_pub_({"tf", "dynamic", "data"}),
      imu_vel_pub_({"imu_velocity",
                    {iox::TruncateToCapacity, configs.imu_name.c_str()},
                    "data"}) {
  LOG_INFO(logger_, "TransformAggregator start!");
  this->imu_quaternion0_ = tools::rpyToQuaterniond(
      {configs_.imu_initial_angle.roll, configs_.imu_initial_angle.pitch,
       configs_.imu_initial_angle.yaw});
  this->imu_translation0_ = {configs_.imu_initial_trans.x,
                             configs_.imu_initial_trans.y,
                             configs_.imu_initial_trans.z};
  this->imu_vel0_ = {configs_.imu_initial_vel.x, configs_.imu_initial_vel.y,
                     configs_.imu_initial_vel.z};
  this->imu_integrator_.init(imu_quaternion0_, imu_translation0_, imu_vel0_);
  this->imu_listener_
      .attachEvent(imu_reset_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onImuResetReceivedCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger_, " unable to attach imu reset event!");
        std::exit(EXIT_FAILURE);
      });
  LOG_INFO(logger_, "success attech imu reset event.");
  this->imu_listener_
      .attachEvent(imu_data_sub_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       onImuDataReceivedCallback, *this))
      .or_else([&](auto) {
        LOG_CRITICAL(logger_, " unable to attach imu data event!");
        std::exit(EXIT_FAILURE);
      });
  LOG_INFO(logger_, "success attech imu data event.");
}
void tf::TransformAggregator::onImuDataReceivedCallback(
    iox::popo::Subscriber<msgs::ImuData, msgs::Header> *subscriber,
    TransformAggregator *self) {
  while (subscriber->take().and_then(
      [subscriber,
       self](const iox::popo::Sample<const msgs::ImuData, const msgs::Header>
                 &imu_data_sample) {
        if (subscriber->getServiceDescription().getInstanceIDString() ==
            iox::capro::IdString_t{iox::TruncateToCapacity,
                                   self->configs_.imu_name.c_str()}) {
          types::ImuData imu_data{imu_data_sample};
          auto [transform, vel] = self->imu_integrator_.update(
              imu_data.orientation, imu_data.linear_acceleration,
              imu_data.angular_velocity,
              tools::nanoSecToChronoPoint(
                  imu_data_sample.getUserHeader().stamp_ns));
          LOG_TRACE_L1(self->logger_, "transform: \n{}",
                       (std::stringstream{} << transform.matrix()).str());
          // transform.translation() = Eigen::Vector3d::Identity();
          // FIXME: 似乎直接积分得到的平移就爆掉了，所以优化时速度会爆掉
          self->imu_quaternion0_ = Eigen::Quaterniond{transform.rotation()};
          // self->imu_vel0_ = vel;
          // NOTE:
          // 为了避免云台/速度非0时使用全0值重置坐标系变换后出现“imu斜着上天”的情况
          // 在保证imu非收到重置请求一直由imu_data回调驱动积分的情况下，
          // 时刻保存当前最新的变换，好在收到重置请求时用作imu初值（即只重置平移）
          self->tf_pub_.loan()
              .and_then([&](iox::popo::Sample<msgs::Transform, msgs::Header>
                                &tf_sample) {
                tf_sample.getUserHeader().frame_id =
                    iox::string<10>{iox::TruncateToCapacity,
                                    self->configs_.map_frame_id.c_str()};
                tf_sample.getUserHeader().stamp_ns =
                    imu_data_sample.getUserHeader().stamp_ns;
                tf_sample->child_frame_id =
                    imu_data_sample.getUserHeader().frame_id;
                tf_sample->quaterniond.w = self->imu_quaternion0_.w();
                tf_sample->quaterniond.y = self->imu_quaternion0_.y();
                tf_sample->quaterniond.x = self->imu_quaternion0_.x();
                tf_sample->quaterniond.z = self->imu_quaternion0_.z();
                tf_sample->translate.x = transform.translation().x();
                tf_sample->translate.y = transform.translation().y();
                tf_sample->translate.z = transform.translation().z();
                tf_sample.publish();
                LOG_DEBUG(self->logger_, "transform {} to imu published.",
                          self->configs_.map_frame_id);
              })
              .or_else([&](auto) {
                LOG_ERROR(self->logger_,
                          "transform {} to imu publish failured.",
                          self->configs_.map_frame_id);
              });
          self->imu_vel_pub_.loan()
              .and_then([&](iox::popo::Sample<msgs::Vector3d, msgs::Header>
                                &vel_sample) {
                vel_sample.getUserHeader().frame_id =
                    imu_data_sample.getUserHeader().frame_id;
                vel_sample.getUserHeader().stamp_ns =
                    imu_data_sample.getUserHeader().stamp_ns;
                vel_sample->x = vel.x();
                vel_sample->y = vel.y();
                vel_sample->z = vel.z();
                vel_sample.publish();
                LOG_DEBUG(self->logger_, "imu_velocity published.");
              })
              .or_else([&](auto) {
                LOG_ERROR(self->logger_, "imu_velocity publish failured!");
              });
        }
      })) {
  } // end of while
}

void tf::TransformAggregator::onImuResetReceivedCallback(
    iox::popo::Subscriber<int, msgs::Header> *subscriber,
    TransformAggregator *self) {
  while (subscriber->take().and_then([subscriber, self](auto &sample) {
    auto instance_string =
        subscriber->getServiceDescription().getInstanceIDString();
    auto event_string = subscriber->getServiceDescription().getEventIDString();
    if (instance_string ==
            iox::capro::IdString_t{iox::TruncateToCapacity,
                                   self->configs_.imu_name.c_str()} &&
        event_string == "reset") {
      // std::scoped_lock lk{self->imu_reset_mtx_};
      self->imu_integrator_.init(self->imu_quaternion0_,
                                 self->imu_translation0_, self->imu_vel0_);
      LOG_INFO(self->logger_, "transform {} to imu reset!",
               self->configs_.map_frame_id);
    }
  })) {
  }
}
