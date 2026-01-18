// Copyright (c) 2026 F. All Rights Reserved.
#include "configs.hpp"
#include "msgs/Header.hpp"
#include "msgs/Transform.hpp"

#include <iceoryx_posh/popo/publisher.hpp>
#include <quill/Logger.h>

namespace tf {
class StaticBroudcaster {
public:
  StaticBroudcaster(quill::Logger *logger,
                    const StaticBroudcasterConfig &config);
  void publishTransforms();

private:
  quill::Logger *logger_;
  StaticBroudcasterConfig config_;
  iox::popo::Publisher<msgs::Transform, msgs::Header> static_tf_puber_;
};
} // namespace tf
