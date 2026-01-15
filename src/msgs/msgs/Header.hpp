// Copyright(c) 2026 Fsk.All Rights Reserved.
#pragma once
#include "iox/string.hpp"
#include <cstdint>

namespace msgs {
struct Header {
  iox::string<10> frame_id;  // 一个消息只有在确定的坐标系内才有意义
  std::uint64_t stamp_ns{0}; // 时间戳
};
} // namespace msgs
