#pragma once

#include "configs.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "msgs/Header.hpp"
#include "msgs/ImuData.hpp"

#include <iceoryx_posh/popo/sample.hpp>
#include <quill/Logger.h>

#include <memory>

namespace hardware {

class ImuBase {
public:
  virtual ~ImuBase() = default;
  virtual bool
  getImuData(iox::popo::Sample<msgs::ImuData, msgs::Header> &sample) = 0;
};

class Imu {
public:
  Imu(quill::Logger *logger, const ImuConfigs &configs);
  void publishImuData();

private:
  quill::Logger *logger_;
  ImuConfigs configs_;
  std::unique_ptr<ImuBase> imu_;
  iox::popo::Publisher<msgs::ImuData, msgs::Header> imu_data_pub_;
};
} // namespace hardware
