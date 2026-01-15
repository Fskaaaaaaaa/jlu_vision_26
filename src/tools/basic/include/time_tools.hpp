// Copyright (c) 2026 F. All Rights Reserved.

#include <chrono>
#include <cstdint>

namespace tools {
inline std::uint64_t getTimeNowNanoSec() {
  std::uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
  return ns;
}
inline std::chrono::system_clock::time_point
nanoSecToChronoPoint(std::uint64_t stamp) {
  std::chrono::nanoseconds ns(stamp);
  std::chrono::system_clock::time_point tp(ns);
  return tp;
}
} // namespace tools
