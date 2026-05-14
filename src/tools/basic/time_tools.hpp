// Copyright (c) 2026 F. All Rights Reserved.
#pragma once

#include <chrono>
#include <iomanip>

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

inline std::string getTimeNowStr() {
  auto now = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::localtime(&now_c);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S-") << std::setw(3)
      << std::setfill('0') << ms.count();
  return oss.str();
}
} // namespace tools
