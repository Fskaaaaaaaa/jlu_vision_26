// Copyright (c) 2026 F. All Rights Reserved.

#include <chrono>
#include <cstdint>

namespace tools {
inline long getTimeNowNanoSec() {
  long ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
  return ns;
}
inline std::chrono::system_clock::time_point nanoSecToChronoPoint(long stamp) {
  std::chrono::nanoseconds ns(stamp);
  std::chrono::system_clock::time_point tp(ns);
  return tp;
}

inline long
chronoPointToNanoSec(const std::chrono::system_clock::time_point &stamp) {
  long ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                stamp.time_since_epoch())
                .count();
  return ns;
};
} // namespace tools
