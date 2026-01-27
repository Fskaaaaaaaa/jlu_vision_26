// Copyright (c) 2026 Gunmu. All Rights Reserved.
#pragma once

#include "msgs/Header.hpp"
#include "msgs/Transform.hpp"

#include "iceoryx_posh/popo/publisher.hpp"
#include "quill/Logger.h"
#include <Eigen/Dense>
#include <chrono>

namespace tf {
class DynamicTransformPublisher {
public:
  DynamicTransformPublisher(quill::Logger *logger);
  void publishTransform(const Eigen::Isometry3d &transform,
                        const std::string &from, const std::string &to,
                        std::chrono::system_clock::time_point stamp);
  void publishTransform(const msgs::Transform &transform,
                        const msgs::Header &header);

private:
  quill::Logger *logger_;
  iox::popo::Publisher<msgs::Transform, msgs::Header> tf_pub_;
};
} // namespace tf
