#pragma once

#include "configs.hpp"
#include "imu_integrator.hpp"
#include "msgs/Basic.hpp"
#include "msgs/Header.hpp"
#include "msgs/ImuData.hpp"
#include "msgs/Transform.hpp"

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "quill/Logger.h"

#include <mutex>
namespace tf {
class TransformAggregator {
public:
  TransformAggregator(quill::Logger *logger,
                      const TransformAggregatorConfigs &configs);

private:
  static void onImuDataReceivedCallback(
      iox::popo::Subscriber<msgs::ImuData, msgs::Header> *subscriber,
      TransformAggregator *self);
  static void onImuResetReceivedCallback(
      iox::popo::Subscriber<int, msgs::Header> *subscriber,
      TransformAggregator *self);
  quill::Logger *logger_;
  TransformAggregatorConfigs configs_;
  ImuIntegrator imu_integrator_;
  iox::popo::Listener imu_listener_;
  // NOTE:
  // 不需要mutex,让两个sub绑定到同一个listener里，两者的回调就天然线程安全了
  iox::popo::Subscriber<msgs::ImuData, msgs::Header> imu_data_sub_;
  iox::popo::Subscriber<int, msgs::Header> imu_reset_sub_;
  iox::popo::Publisher<msgs::Transform, msgs::Header> tf_pub_;
  iox::popo::Publisher<msgs::Vector3d, msgs::Header> imu_vel_pub_;
  Eigen::Quaterniond imu_quaternion0_;
  Eigen::Vector3d imu_translation0_;
  Eigen::Vector3d imu_vel0_;
};
} // namespace tf
