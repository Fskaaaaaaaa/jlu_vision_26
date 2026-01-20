#pragma once

#include "msgs/Header.hpp"
#include "msgs/ImuData.hpp"

#include <gtsam/base/Vector.h>
#include <iceoryx_posh/popo/sample.hpp>

namespace types {
struct ImuData {
  ImuData(
      const iox::popo::Sample<const msgs::ImuData, const msgs::Header> &sample);
  gtsam::Vector4 orientation;
  gtsam::Vector3 angular_velocity;
  gtsam::Vector3 linear_acceleration;
};
} // namespace types
