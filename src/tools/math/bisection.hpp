// Copyright (c) 2025 Feng. All Rights Reserved.
#pragma once

#include <utility>
namespace rm_ultra_tools {

class Bisection {
public:
  template <typename ValueT, class Func>
  static std::pair<ValueT, ValueT> find(ValueT left, ValueT right,
                                        Func &&cost_function,
                                        const int &iterations_num) {
    for (int i = 0; i < iterations_num; ++i) {
      ValueT mid = (left + right) / ValueT(2);
      if (cost_function(mid) < ValueT(0)) {
        left = mid;
      } else {
        right = mid;
      }
    }
    return std::make_pair((left + right) / ValueT(2), right - left);
  }

  template <typename ValueT, class Func>
  static std::pair<ValueT, ValueT>
  find(ValueT left, ValueT right, Func &&cost_function,
       const int &iterations_num, const ValueT &min_error) {
    for (int i = 0; i < iterations_num; ++i) {
      ValueT mid = (left + right) / ValueT(2);
      if ((right - left) < min_error) {
        break;
      }
      if (cost_function(mid) < ValueT(0)) {
        left = mid;
      } else {
        right = mid;
      }
    }
    return std::make_pair((left + right) / ValueT(2), right - left);
  }
};
} // namespace rm_ultra_tools
