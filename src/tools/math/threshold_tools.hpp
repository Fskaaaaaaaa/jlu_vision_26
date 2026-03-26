#pragma once

#include <cmath>
#include <numbers>

#include "sigmoid_functions.hpp"

namespace tools {

inline double limitZeroThres(double in, double zero_thres, double max) {
  return std::min(in < zero_thres ? 0. : in, max);
}

// NOTE: 用于归一化恒正的误差（假设其正态分布）
class ErrorNormalizer {
public:
  ErrorNormalizer(double error0, double p0) {
    double x = errorInverse(p0);
    sigma_ = error0 / (std::numbers::sqrt2 * x);
  }
  double operator()(double error) {
    // 标准正态回到error函数
    return 2.0 * standardNormalCDF(error, sigma_) - 1.0;
  }
  double sigma_;
};

class HysteresisComparator {
public:
  HysteresisComparator(double thres_high, double thres_low,
                       bool reverse_trigger)
      : thres_high(thres_high), thres_low(thres_low),
        reverse_trigger(reverse_trigger) {}
  bool operator()(double input, bool reset = false) {
    if (reset)
      last_output = false;
    double thresh;
    if (reverse_trigger)
      thresh = (last_output == true) ? thres_high : thres_low;
    else
      thresh = (last_output == true) ? thres_low : thres_high;
    return reverse_trigger ? (last_output = input < thresh)
                           : (last_output = input > thresh);
  }
  double thres_high;
  double thres_low;
  bool reverse_trigger;
  bool last_output;
};

} // namespace tools
