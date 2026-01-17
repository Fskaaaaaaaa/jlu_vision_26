#pragma once

#include <utility>

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

namespace rm_ultra_tools {

template <class F> auto diff(F &&f) {
  return [f = std::forward<F>(f)]<class... Args>(Args &&...x) mutable {
    constexpr auto N = sizeof...(Args);
    using Scalar = Eigen::AutoDiffScalar<Eigen::VectorXd>;
    auto ans = [&]<std::size_t... I>(std::index_sequence<I...>) -> Scalar {
      return f(Scalar(static_cast<double>(x), N, I)...);
    }(std::make_index_sequence<N>{});
    return std::make_pair(ans.value(), ans.derivatives());
  };
}

// template <class F> auto diff(F &&f, int n) {
//   return [f = std::forward<F>(f)]<class... Args>(Args &&...x) mutable {
//     constexpr auto N = sizeof...(Args);
//     using Scalar = Eigen::AutoDiffScalar<Eigen::VectorXd>;
//     auto ans = [&]<std::size_t... I>(std::index_sequence<I...>) -> Scalar {
//       return f(Scalar(static_cast<double>(x), N, I)...);
//     }(std::make_index_sequence<N>{});
//     return std::tuple<typename>(ans.value(), ans.derivatives());
//   };
// }

// namespace detial {
// template <double... Vals> struct wrapper {
//   static constexpr std::size_t N = sizeof...(Vals);
//   using ADScalar = Eigen::AutoDiffScalar<Eigen::VectorXd>;
//   using ADVec = Eigen::Matrix<ADScalar, N, 1>;
//   template <class F> static auto eval(F &&f) {
//     ADVec x;
//     double vals[] = {Vals...};
//     for (std::size_t i = 0; i < N; ++i)
//       x[i].value() = vals[i];
//     for (std::size_t i = 0; i < N; ++i)
//       x[i].derivatives() = Eigen::VectorXd::Unit(N, i);
//     ADScalar y = f(x); // lambda 只看到一个参数，可索引
//     struct Result {
//       double value;
//       Eigen::VectorXd grad;
//     };
//     return Result{y.value(), y.derivatives()};
//   }
// };
// } // namespace detial
//
// template <double... Vals, class F> inline auto constDiff(F &&f) {
//   return detial::wrapper<Vals...>::eval(std::forward<F>(f));
// } // 瞎搞出来的只能求函数对编译期常量的导数的废物函数

} // namespace rm_ultra_tools
