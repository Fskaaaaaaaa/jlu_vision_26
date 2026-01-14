// Copyright(c) 2026 Fsk.All Rights Reserved.
#pragma once
// #include "iox/string.hpp"
#include <cstdint>

namespace msgs {
struct Header {
  // iox::string<20> frame_id{"defaut"};
  std::uint64_t time_stamp_ns{0};
};
} // namespace msgs
