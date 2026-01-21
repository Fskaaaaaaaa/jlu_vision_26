#include "imu.hpp"
#include "configs.hpp"
#include "iceoryx_posh/popo/sample.hpp"
#include "msgs/Header.hpp"
#include "msgs/ImuData.hpp"
#include "xrobot.hpp"

#include "iox/string.hpp"
#include "quill/LogMacros.h"
#include "quill/core/ThreadContextManager.h"

#include <memory>

hardware::Imu::Imu(quill::Logger *logger, const ImuConfigs &configs)
    : logger_(logger), configs_(configs),
      imu_data_pub_({"imu_data",
                     {iox::TruncateToCapacity, configs_.imu_name.c_str()},
                     "data"}) {
  LOG_INFO(logger_, "imu start!");
  switch (configs_.imu_type) {
  case ImuType::xr:
    this->imu_ = std::make_unique<XRobot>(logger, configs_.imu_config);
    break;
  case ImuType::dm:
    break;
  default:
    LOG_CRITICAL(logger_, "Unknown camera type!");
    std::exit(EXIT_FAILURE);
  }
}

void hardware::Imu::publishImuData() {
  // return;
  this->imu_data_pub_.loan()
      .and_then([&](iox::popo::Sample<msgs::ImuData, msgs::Header> &sample) {
        if (this->imu_->getImuData(sample))
          sample.publish();
      })
      .or_else(
          [&](auto) { LOG_ERROR(logger_, "unable to loan mem, retrying..."); });
}
