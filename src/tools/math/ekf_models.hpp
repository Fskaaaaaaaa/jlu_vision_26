// // Copyright (c) 2025 Feng. All Rights Reserved.
// #pragma once
// #include "angle_tools.hpp"
// #include "yaml_config_loader.hpp"
//
// #include <Eigen/Dense>
//
// #include <functional>
// #include <utility>
// #include <vector>
//
// namespace rm_ultra_tools {
//
// namespace ekf {
// template <int SysDim, int ObsDim> class ExtendedKalmanFilter {
// public:
//   using Mat_nxn = Eigen::Matrix<double, SysDim, SysDim>;
//   using Mat_nxm = Eigen::Matrix<double, SysDim, ObsDim>;
//   using Mat_mxn = Eigen::Matrix<double, ObsDim, SysDim>;
//   using Mat_mxm = Eigen::Matrix<double, ObsDim, ObsDim>;
//   using Vec_n = Eigen::Vector<double, SysDim>;
//   using Vec_m = Eigen::Vector<double, ObsDim>;
//   ExtendedKalmanFilter() = delete;
//   ExtendedKalmanFilter(std::vector<int> angle_pos_x = {},
//                        std::vector<int> angle_pos_z = {});
//
//   std::function<Vec_n(const Vec_n &x, double dt)> f;
//   std::function<Mat_nxn(const Vec_n &x, double dt)> get_f_jacobian;
//   // 匀速运动是lti系统，F和JF相同且不依赖x,但为了通用性，参数x还是留着吧
//   std::function<Mat_nxn(const Vec_n &x, double dt)> get_q;
//   std::function<Vec_m(const Vec_n &x)> h;
//   std::function<Mat_mxn(const Vec_n &x)> get_h_jacobian;
//   std::function<Mat_mxm(const Vec_m &z)> get_r;
//
//   Mat_nxn P;
//   Vec_n x;
//   // 不区分x和p的先验后验，可直接连续多次predict
//
//   void init(const Vec_n &x0, const Mat_nxn &p0 = Mat_nxn::Identity());
//   void setState(const Vec_n &state);
//   Vec_n getState();
//   Vec_n update(const Vec_m &obs);
//   Vec_n predict(double dt);
//   // std::function<Vec_n(double dt)> getKinematicEquation() const;
//
// private:
//   Vec_n limitAngleX(Vec_n x);
//   std::vector<int> angle_pos_x_;
//   Vec_m limitAngleZ(Vec_m z);
//   std::vector<int> angle_pos_z_;
// };
//
// template <int SysDim, int ObsDim>
// ExtendedKalmanFilter<SysDim, ObsDim>::ExtendedKalmanFilter(
//     std::vector<int> angle_pos_x, std::vector<int> angle_pos_z)
//     : angle_pos_x_(std::move(angle_pos_x)),
//       angle_pos_z_(std::move(angle_pos_z)) {
//   this->x = Vec_n::Zero();
//   this->P = Mat_nxn::Identity();
// }
//
// template <int SysDim, int ObsDim>
// void ExtendedKalmanFilter<SysDim, ObsDim>::init(const Vec_n &x0,
//                                                 const Mat_nxn &p0) {
//   this->P = p0;
//   this->x = x0;
//   return;
// }
//
// template <int SysDim, int ObsDim>
// void ExtendedKalmanFilter<SysDim, ObsDim>::setState(const Vec_n &state) {
//   this->x = state;
//   return;
// }
//
// template <int SysDim, int ObsDim>
// auto ExtendedKalmanFilter<SysDim, ObsDim>::getState() -> Vec_n {
//   return this->x.eval();
// }
//
// template <int SysDim, int ObsDim>
// ExtendedKalmanFilter<SysDim, ObsDim>::Vec_n
// ExtendedKalmanFilter<SysDim, ObsDim>::update(const Vec_m &z) {
//   Vec_n x_prior = x;
//   Mat_mxn JH = this->get_h_jacobian(x);
//   Mat_mxm R = this->get_r(z);
//   Mat_nxm K = P * JH.transpose() * (JH * P * JH.transpose() + R).inverse();
//   Mat_nxn I = Eigen::Matrix<double, SysDim, SysDim>::Identity();
//   x = limitAngleX(x_prior + K * (limitAngleZ(z - h(x_prior))));
//   // x = x_add(x, K * z_subtract(z, h(x)));
//   // ^同济爷为了限制计算在最小弧度角上，把相减和相加都设置成lamda了
//   // 这里用了个折中的方法实现，需要在构造函数传递第几个参数是角度
//   P = (I - K * JH) * P * (I - K * JH).transpose() + K * R * K.transpose();
//
//   return x.eval();
// }
//
// template <int SysDim, int ObsDim>
// ExtendedKalmanFilter<SysDim, ObsDim>::Vec_n
// ExtendedKalmanFilter<SysDim, ObsDim>::predict(double dt) {
//   Mat_nxn JF = this->get_f_jacobian(x, dt);
//   Mat_nxn Q = this->get_q(x, dt);
//   x = limitAngleX(f(x, dt));
//   P = JF * P * JF.transpose() + Q;
//   return x.eval();
// }
//
// // template <int SysDim, int ObsDim>
// // auto ExtendedKalmanFilter<SysDim, ObsDim>::getKinematicEquation() const
// //     -> std::function<Vec_n(double dt)> {
// //   return
// //       [this](const Vec_n &x, double dt) { return limitAngleX(this->f(x,
// dt));
// //       };
// // }
//
// template <int SysDim, int ObsDim>
// ExtendedKalmanFilter<SysDim, ObsDim>::Vec_n
// ExtendedKalmanFilter<SysDim, ObsDim>::limitAngleX(Vec_n x) {
//   for (const auto &pos : angle_pos_x_) {
//     if (pos < SysDim)
//       x(pos) = limitRadian(x(pos));
//   }
//   return x.eval();
// }
//
// template <int SysDim, int ObsDim>
// ExtendedKalmanFilter<SysDim, ObsDim>::Vec_m
// ExtendedKalmanFilter<SysDim, ObsDim>::limitAngleZ(Vec_m z) {
//   for (const auto &pos : angle_pos_z_) {
//     if (pos < ObsDim)
//       z(pos) = limitRadian(z(pos));
//   }
//   return z.eval();
// }
//
// constexpr int TARTET_X_DIM = 11;
// constexpr int TARTET_Z_DIM = 4;
// class TargetEkf : public ExtendedKalmanFilter<TARTET_X_DIM, TARTET_Z_DIM> {
// public:
//   enum {
//     Z_X,
//     Z_Y,
//     Z_Z,
//     Z_YAW,
//     OBS_DIM,
//   };
//   TargetEkf() = delete;
//   TargetEkf(ParamGetter param_getter, const std::string &name = "robot_ekf",
//             std::vector<int> angle_pos_x = {},
//             std::vector<int> angle_pos_z = {});
//
//   virtual Vec_n reset(const Vec_m &obs) = 0;
//   virtual Vec_n update(const Vec_m &obs, int armor_index) = 0;
//   virtual std::vector<std::function<Eigen::Vector4d(const Vec_n &x, double
//   dt)>> getKinematicFuncXyza() = 0;
//
// protected:
//   ParamGetter param_getter_;
//   std::string name_;
//   std::function<std::function<Vec_m(const Vec_n &)>(int armor_index)> get_h_;
//   std::function<std::function<Mat_mxn(const Vec_n &)>(int armor_index)>
//       get_get_h_jacobian_;
// };
//
// class RobotEkf : public TargetEkf {
// public:
//   enum {
//     X_X,
//     X_VX,
//     X_Y,
//     X_VY,
//     X_Z,
//     X_VZ,
//     X_YAW,
//     X_VYAW,
//     X_R1, // 前后半径
//     X_R2, // 左右半径
//     X_DZ,
//     X_DIM
//   };
//
//   RobotEkf() = delete;
//   RobotEkf(ParamGetter param_getter, const std::string &name = "robot_ekf",
//            std::vector<int> angle_pos_x = {},
//            std::vector<int> angle_pos_z = {});
//
//   Vec_n reset(const Vec_m &obs) override;
//   Vec_n update(const Vec_m &obs, int armor_index) override;
//   std::vector<std::function<Eigen::Vector4d(const Vec_n &x, double)>>
//   getKinematicFuncXyza() override;
//
// private:
//   enum class Param {
//     q_sigma2_x,
//     q_sigma2_y,
//     q_sigma2_z,
//     q_sigma2_yaw,
//     q_sigma2_r,
//     q_sigma2_dz,
//     r_xyz_factor,
//     r_yaw,
//     initial_radius,
//     max_radius,
//     min_radius,
//     kine_equ_vx_zero_thres,
//     kine_equ_vy_zero_thres,
//     kine_equ_vz_zero_thres,
//     kine_equ_vyaw_zero_thres,
//     kine_equ_max_vx,
//     kine_equ_max_vy,
//     kine_equ_max_vz,
//     kine_equ_max_vyaw,
//   };
// };
//
// class OutpostEkf : public TargetEkf {
// public:
//   enum {
//     X_X,
//     X_VX,
//     X_Y,
//     X_VY,
//     X_Z,
//     X_VZ,
//     X_YAW,
//     X_VYAW,
//     X_R,
//     X_DZ1, // 中间到低板子,负数
//     X_DZ2, // 中间到高板子,正数
//     X_DIM
//   };
//   OutpostEkf() = delete;
//   OutpostEkf(ParamGetter param_getter, const std::string &name =
//   "outpost_ekf",
//              std::vector<int> angle_pos_x = {},
//              std::vector<int> angle_pos_z = {});
//
//   Vec_n reset(const Vec_m &obs) override;
//   Vec_n update(const Vec_m &obs, int armor_index) override;
//   std::vector<std::function<Eigen::Vector4d(const Vec_n &x, double dt)>>
//   getKinematicFuncXyza() override;
//
// private:
//   static constexpr double OUTPOST_RADIUS = 0.2765;
//   // static constexpr double OUTPOST_DZ = 0.1;
//   // 确定的物理常量没必要运行时查找
//   enum class Param {
//     q_sigma2_x,
//     q_sigma2_y,
//     q_sigma2_z,
//     q_sigma2_yaw,
//     q_sigma2_r,
//     q_sigma2_dz,
//     r_xyz_factor,
//     r_yaw,
//     max_radius,
//     min_radius,
//   };
// };
//
// } // namespace ekf
//
// } // namespace rm_ultra_tools
