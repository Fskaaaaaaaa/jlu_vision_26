// Copyright (c) 2026 F. All Rights Reserved.
#pragma once
#include "types/Transform.hpp"

#include <quill/core/LogLevel.h>
#include <vector>

namespace tf {
struct StaticBroudcasterConfig {
  std::vector<types::TransformConfig> transforms = {};
};
struct StaticBroudcasterConfigs {
  quill::LogLevel log_level = quill::LogLevel::Info;
  int publish_interval_ms = 200;
  StaticBroudcasterConfig static_broudcaster_conf;
};
} // namespace tf
