// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

#include "msgs/Header.hpp"
#include "msgs/Transform.hpp"
#include "types/Basic.hpp"

#include <Eigen/Dense>
#include <array>
#include <iceoryx_posh/popo/sample.hpp>

#include <chrono>

namespace types {
struct Transform {
  Transform(const iox::popo::Sample<const msgs::Transform, const msgs::Header>
                &sample);
  Eigen::Isometry3d getIsometry3d();
  std::string parent_frame_id;
  std::string child_frame_id;
  std::chrono::time_point<std::chrono::system_clock> stamp;
  Eigen::Vector3d tvec;
  Eigen::Quaterniond quaterniond;
};
} // namespace types
