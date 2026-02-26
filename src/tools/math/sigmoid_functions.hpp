#pragma once

#include <cmath>

namespace tools {

inline double logisticFunction(double x) {
  if (x > 0)
    return 1.0 / (1.0 + std::exp(-x));
  else
    return std::exp(x) / (1.0 + std::exp(x));
}

inline double logisticFunction(double x, double min, double max) {
  return (logisticFunction(x) * (max - min)) + min;
}

inline double logisticInverse(double y, double min, double max) {
  return std::log((y - min) / (max - y));
}

// NOTE: 需要传入的是逻辑函数的输出值
inline double logisticDerivative(double y) { return y * (1.0 - y); }

inline double logisticDerivative(double y, double min, double max) {
  return (y - min) * (max - y) / (max - min);
}

} // namespace tools
