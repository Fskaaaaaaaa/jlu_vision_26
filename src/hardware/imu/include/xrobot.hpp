#pragma once

#include "configs.hpp"
#include "imu.hpp"
#include "quill/Logger.h"

namespace hardware {

class XRobot : public ImuBase {
public:
  XRobot(quill::Logger *logger, ImuConfig config);
  void populateAndPublish(
      iox::popo::Sample<msgs::ImuData, msgs::Header> &sample) override;

private:
  quill::Logger *logger_;
  ImuConfig config_;
  int serial_fd_;
  uint64_t first_imu_time_;
  uint64_t first_local_time_;
  bool has_sync_;
};

} // namespace hardware
