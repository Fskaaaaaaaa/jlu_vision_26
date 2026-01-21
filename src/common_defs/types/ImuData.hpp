#pragma once

#include "msgs/Header.hpp"
#include "msgs/ImuData.hpp"

#include <Eigen/Dense>
#include <iceoryx_posh/popo/sample.hpp>

namespace types {
struct ImuData {
  ImuData(
      const iox::popo::Sample<const msgs::ImuData, const msgs::Header> &sample);
  Eigen::Quaterniond orientation;
  Eigen::Vector3d angular_velocity;
  Eigen::Vector3d linear_acceleration;
};
} // namespace types
