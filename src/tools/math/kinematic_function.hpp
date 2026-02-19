// #pragma once
//
// #include "ekf_models.hpp"
//
// #include <Eigen/Dense>
//
// #include <functional>
// #include <utility>
//
// namespace rm_ultra_tools {
//
// template <int SysDim, typename ReturnType> class KinematicFunction {
// public:
//   using Vec_n = Eigen::Vector<double, SysDim>;
//   using FuncType = std::function<ReturnType(const Vec_n &x, double dt)>;
//   KinematicFunction() = delete;
//   KinematicFunction(const std::string &name, const Vec_n &sys_state,
//                     FuncType &&kfunction)
//       : name_(name), sys_state_(sys_state),
//         function_(std::forward<FuncType>(kfunction)) {}
//
//   ReturnType operator()(double t) const {
//     return this->function_(sys_state_, t);
//   }
//
//   // 配合隐式转换的枚举使用
//   template <int IDX> double get() const {
//     static_assert(IDX >= 0 && IDX < SysDim, "Index out of bounds");
//     return sys_state_(IDX);
//   };
//
//   std::string getName() const { return name_; }
//   Vec_n getState() const { return sys_state_; }
//   FuncType getFunction() const { return function_; }
//
// protected:
//   std::string name_;
//   Eigen::Vector<double, SysDim> sys_state_;
//   FuncType function_;
// };
//
// class TargetKinematicFuncXyza
//     : public KinematicFunction<ekf::TARTET_X_DIM, Eigen::Vector4d> {
// public:
//   TargetKinematicFuncXyza() = delete;
//   TargetKinematicFuncXyza(const std::string &name, const Vec_n &sys_state,
//                           FuncType &&kfunction);
//   // TODO TBD
//
// private:
// };
//
// class TargetKinematicFuncXyzRpy
//     : public KinematicFunction<ekf::TARTET_X_DIM,
//                                std::pair<Eigen::Vector3d, Eigen::Vector3d>> {
// public:
//   TargetKinematicFuncXyzRpy() = delete;
//   TargetKinematicFuncXyzRpy(const TargetKinematicFuncXyza &func_xyza,
//                             double pitch, double roll = 0);
//
// private:
// };
//
// // template <int N, typename ReturnType>
// // KinematicFunction(const std::string &, const Eigen::Vector<double, N> &,
// //                   std::function<ReturnType(double t)>)
// //     -> KinematicFunction<N, ReturnType>;
// // template <int N, typename F>
// // KinematicFunction(const std::string &, const Eigen::Vector<double, N> &,
// F)
// //     -> KinematicFunction<N, std::invoke_result_t<F, double>>;
//
// } // namespace rm_ultra_tools
