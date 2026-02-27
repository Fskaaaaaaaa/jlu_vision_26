#pragma once

#include <cmath>
#include <numbers>

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

inline double signalFunction(double x) {
  if (x > 1e-6)
    return 1;
  if (x < 1e-6)
    return -1;
  return 0;
}

inline double errorFunction(double x, double min, double max) {
  return (std::erf(x) + 1.0) / 2.0 * (max - min) + min;
}

inline double errorInverse(double y) {
  constexpr double a = 8 * (std::numbers::pi - 3) /
                       (3 * std::numbers::pi * (4 - std::numbers::pi));
  constexpr double inv_a = 1 / a;
  double b = 2 * std::numbers::inv_pi * inv_a;
  double c = std::log(1 - y * y);
  // NOTE: 为误差函数反函数的解析近似，详见
  // https://zh.wikipedia.org/wiki/%E8%AF%AF%E5%B7%AE%E5%87%BD%E6%95%B0
  double x =
      signalFunction(y) *
      std::sqrt(std::sqrt((b + c / 2) * (b + c / 2) - c / a) - (b + c / 2));
  return x;
}

inline double errorInverse(double y, double min, double max) {
  return errorInverse((2.0 * y - 2 * min) / (max - min) - 1);
}

inline double standardNormalCDF(double x, double standard_deviation = 1) {
  return errorFunction(x / (standard_deviation * std::numbers::sqrt2), 0, 1);
}

} // namespace tools
