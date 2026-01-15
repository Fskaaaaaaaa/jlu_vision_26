// #include "rm_ultra_tools/kinematic_function.hpp"
// #include "rm_ultra_tools/coordinate_tools.hpp"
// #include <Eigen/src/Core/Matrix.h>
// #include <utility>
//
// namespace rm_ultra_tools {
//
// TargetKinematicFuncXyza::TargetKinematicFuncXyza(const std::string &name,
//                                                  const Vec_n &sys_state,
//                                                  FuncType &&kfunction)
//     : KinematicFunction<ekf::TARTET_X_DIM, Eigen::Vector4d>(
//           name, sys_state, std::move(kfunction)) {}
//
// TargetKinematicFuncXyzRpy::TargetKinematicFuncXyzRpy(
//     const TargetKinematicFuncXyza &func_xyza, double pitch, double roll)
//     : KinematicFunction<ekf::TARTET_X_DIM,
//                         std::pair<Eigen::Vector3d, Eigen::Vector3d>>(
//           func_xyza.getName(), func_xyza.getState(), {}) {
//   this->function_ = [func = func_xyza.getFunction(), roll,
//                      pitch](const Vec_n &x, double dt) {
//     Eigen::Vector4d xyza = func(x, dt);
//     return xyzaToXyzRpy(xyza, roll, pitch);
//   };
// }
// } // namespace rm_ultra_tools
