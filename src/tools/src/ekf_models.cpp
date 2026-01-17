// // Copyright (c) 2025 Feng. All Rights Reserved.
// #include "ekf_models.hpp"
// #include "angle_tools.hpp"
//
// #include <array>
// #include <functional>
// #include <numbers>
// #include <tuple>
// #include <utility>
// #include <vector>
//
// namespace rm_ultra_tools {
//
// ekf::TargetEkf::TargetEkf(ParamGetter param_getter, const std::string &name,
//                           std::vector<int> angle_pos_x,
//                           std::vector<int> angle_pos_z)
//     : ExtendedKalmanFilter(angle_pos_x, angle_pos_z),
//       param_getter_(param_getter), name_(name) {}
//
// ekf::RobotEkf::Vec_n ekf::RobotEkf::update(const Vec_m &obs, int armor_index)
// {
//   std::tie(this->h, this->get_h_jacobian) = std::make_pair(
//       this->get_h_(armor_index), get_get_h_jacobian_(armor_index));
//   Vec_n x_post = ExtendedKalmanFilter<X_DIM, OBS_DIM>::update(obs).eval();
//   // Prevent radius from spreading
//   auto r_max = this->param_getter_(name_)(Param::max_radius).get(0.5);
//   auto r_min = this->param_getter_(name_)(Param::min_radius).get(0.2);
//   x_post(X_R1) =
//       std::clamp(x_post(rm_ultra_tools::ekf::RobotEkf::X_R1), r_min, r_max);
//   x_post(X_R2) =
//       std::clamp(x_post(rm_ultra_tools::ekf::RobotEkf::X_R2), r_min, r_max);
//   this->setState(x_post);
//   return x_post;
// }
//
// std::vector<
//     std::function<Eigen::Vector4d(const ekf::RobotEkf::Vec_n &x, double)>>
// ekf::RobotEkf::getKinematicFuncXyza() {
//   auto min_vx =
//       this->param_getter_(name_)(Param::kine_equ_vx_zero_thres).get(0.05);
//   auto min_vy =
//       this->param_getter_(name_)(Param::kine_equ_vy_zero_thres).get(0.05);
//   auto min_vz =
//       this->param_getter_(name_)(Param::kine_equ_vz_zero_thres).get(0.05);
//   auto min_vyaw =
//       this->param_getter_(name_)(Param::kine_equ_vyaw_zero_thres).get(0.5);
//   auto max_vx = this->param_getter_(name_)(Param::kine_equ_max_vx).get(2.0);
//   auto max_vy = this->param_getter_(name_)(Param::kine_equ_max_vy).get(2.0);
//   auto max_vz = this->param_getter_(name_)(Param::kine_equ_max_vz).get(0.1);
//   auto max_vyaw =
//       this->param_getter_(name_)(Param::kine_equ_max_vyaw).get(15.0);
//   Vec_n state = this->x;
//   state(X_VX) = limitZeroThres(state(X_VX), min_vx, max_vx);
//   state(X_VY) = limitZeroThres(state(X_VY), min_vy, max_vy);
//   state(X_VZ) = limitZeroThres(state(X_VZ), min_vz, max_vz);
//   state(X_VYAW) = limitZeroThres(state(X_VYAW), min_vyaw, max_vyaw);
//   std::vector<std::function<Eigen::Vector4d(const Vec_n &x, double dt)>> fns;
//   for (auto &&idx : std::array{0, 1, 2, 3}) {
//     fns.emplace_back([idx, this](const Vec_n &x, double dt) ->
//     Eigen::Vector4d {
//       Vec_n state = f(x, dt);
//       std::vector<Eigen::Vector4d> armors;
//       return this->get_h_(idx)(state).eval();
//     });
//   }
//   return fns;
// }
//
// ekf::RobotEkf::RobotEkf(ParamGetter param_getter, const std::string &name,
//                         std::vector<int> angle_pos_x,
//                         std::vector<int> angle_pos_z)
//     : TargetEkf(param_getter, name, angle_pos_x, angle_pos_z) {
//   this->f = [](const Vec_n &x, double dt) {
//     Vec_n x_prior = x;
//     x_prior(X_X) = x(X_X) + dt * x(X_VX);
//     x_prior(X_Y) = x(X_Y) + dt * x(X_VY);
//     x_prior(X_Z) = x(X_Z) + dt * x(X_VZ);
//     x_prior(X_YAW) = x(X_YAW) + dt * x(X_VYAW);
//     return x_prior;
//   };
//
//   this->get_f_jacobian = [](const Vec_n &x, double dt) {
//     Mat_nxn F;
//     F.setIdentity();
//     F(X_X, X_VX) = dt;     // x = x + vx * dt
//     F(X_Y, X_VY) = dt;     // y = y + vy * dt
//     F(X_Z, X_VZ) = dt;     // z = z + vz * dt
//     F(X_YAW, X_VYAW) = dt; // theta = theta + omega * dt
//     return F;
//   };
//
//   this->get_q = [this](const Vec_n &x_prior, double dt) {
//     auto s2qx = this->param_getter_(name_)(Param::q_sigma2_x).get(20.0);
//     auto s2qy = this->param_getter_(name_)(Param::q_sigma2_y).get(20.0);
//     auto s2qz = this->param_getter_(name_)(Param::q_sigma2_z).get(20.0);
//     auto s2qyaw = this->param_getter_(name_)(Param::q_sigma2_yaw).get(100.0);
//     auto s2qr = this->param_getter_(name_)(Param::q_sigma2_r).get(800.0);
//     auto s2qdz = this->param_getter_(name_)(Param::q_sigma2_dz).get(800.0);
//
//     double q_x_x = pow(dt, 4) / 4 * s2qx, q_x_vx = pow(dt, 3) / 2 * s2qx,
//            q_vx_vx = pow(dt, 2) * s2qx;
//     double q_y_y = pow(dt, 4) / 4 * s2qy, q_y_vy = pow(dt, 3) / 2 * s2qy,
//            q_vy_vy = pow(dt, 2) * s2qy;
//     double q_z_z = pow(dt, 4) / 4 * s2qz, q_z_vz = pow(dt, 3) / 2 * s2qz,
//            q_vz_vz = pow(dt, 2) * s2qz;
//     // double q_z_z = pow(dt, 4) / 4 * s2qx, q_z_vz = pow(dt, 3) / 2 * s2qx,
//     //        q_vz_vz = pow(dt, 2) * s2qz;
//     double q_yaw_yaw = pow(dt, 4) / 4 * s2qyaw,
//            q_yaw_vyaw = pow(dt, 3) / 2 * s2qx,
//            q_vyaw_vyaw = pow(dt, 2) * s2qyaw;
//     double q_r = pow(dt, 4) / 4 * s2qr;
//     double q_d_zc = pow(dt, 4) / 4 * s2qdz;
//     Mat_nxn Q;
//     // clang-format off
//     //    xc      v_xc    yc      v_yc    zc      v_zc    yaw         v_yaw
//     r1      r2     dz Q <<  q_x_x,  q_x_vx, 0,      0,      0,      0, 0, 0,
//     0,      0,     0,
//           q_x_vx, q_vx_vx,0,      0,      0,      0,      0,          0, 0,
//           0,     0, 0,      0,      q_y_y,  q_y_vy, 0,      0,      0, 0, 0,
//           0,     0, 0,      0,      q_y_vy, q_vy_vy,0,      0,      0, 0, 0,
//           0,     0, 0,      0,      0,      0,      q_z_z,  q_z_vz, 0, 0, 0,
//           0,     0, 0,      0,      0,      0,      q_z_vz, q_vz_vz,0, 0, 0,
//           0,     0, 0,      0,      0,      0,      0,      0, q_yaw_yaw,
//           q_yaw_vyaw, 0,      0,     0, 0,      0,      0,      0,      0, 0,
//           q_yaw_vyaw, q_vyaw_vyaw,0,      0,     0, 0,      0,      0, 0, 0,
//           0,      0,          0,          q_r,    0,     0, 0,      0, 0, 0,
//           0,      0,      0,          0,          0,      q_r,   0, 0, 0, 0,
//           0,      0,      0,      0,          0,          0,      0, q_d_zc;
//     // clang-format on
//     return Q;
//   };
//
//   this->get_r = [this](const Vec_m &z) {
//     // auto r_xyz_factor = getParam(Param::robot_ekf_r_xyz_factor, 0.05);
//     auto r_xyz_factor =
//         this->param_getter_(name_)(Param::r_xyz_factor).get(0.05);
//     // auto r_yaw = getParam(Param::robot_ekf_r_yaw, 0.02);
//     auto r_yaw = this->param_getter_(name_)(Param::r_yaw).get(0.02);
//     Mat_mxm R;
//     R.diagonal() << std::abs(r_xyz_factor * z(Z_X)),
//         std::abs(r_xyz_factor * z(Z_Y)), std::abs(r_xyz_factor * z(Z_Z)),
//         r_yaw;
//     return R;
//   };
//
//   this->get_h_ = [](int armor_index) -> std::function<Vec_m(const Vec_n &)> {
//     // 由编号从机器人yaw得到装甲板朝向、半径和高度
//     // 对侧面的一对装甲板使用r2dz
//     return [armor_index](const Vec_n &x) -> Vec_m {
//       auto armor_yaw =
//           limitRadian(x(X_YAW) + armor_index * std::numbers::pi / 2.);
//       auto use_r2_dz = armor_index % 2 == 1;
//       auto [r, armor_z] = use_r2_dz ? std::make_pair(x(X_R2), x(X_Z) +
//       x(X_DZ))
//                                     : std::make_pair(x(X_R1), x(X_Z));
//       auto armor_x = x(X_X) - r * std::cos(armor_yaw);
//       auto armor_y = x(X_Y) - r * std::sin(armor_yaw);
//       Vec_m hx{armor_x, armor_y, armor_z, armor_yaw};
//       return hx;
//     };
//   };
//   this->get_get_h_jacobian_ =
//       [](int armor_index) -> std::function<Mat_mxn(const Vec_n &)> {
//     return [armor_index](const Vec_n &x) -> Mat_mxn {
//       auto armor_yaw =
//           limitRadian(x(X_YAW) + armor_index * std::numbers::pi / 2.);
//       auto use_r2_dz = armor_index % 2 == 1;
//       auto r = use_r2_dz ? x(X_R2) : x(X_R1);
//       Mat_mxn JH;
//       JH.setZero();
//       JH(Z_X, X_X) = 1.;
//       JH(Z_Y, X_Y) = 1.;
//       JH(Z_Z, X_Z) = 1.;
//       JH(Z_YAW, X_YAW) = 1.;
//       JH(Z_X, X_YAW) = r * std::sin(armor_yaw);
//       JH(Z_Y, X_YAW) = -r * std::cos(armor_yaw);
//       JH(Z_X, X_R1) = use_r2_dz ? 0. : -std::cos(armor_yaw);
//       JH(Z_X, X_R2) = use_r2_dz ? -std::cos(armor_yaw) : 0.;
//       JH(Z_Y, X_R1) = use_r2_dz ? 0. : -std::sin(armor_yaw);
//       JH(Z_Y, X_R2) = use_r2_dz ? -std::sin(armor_yaw) : 0.;
//       JH(Z_Z, X_DZ) = use_r2_dz ? 1. : 0.;
//       return JH;
//     };
//   };
// }
//
// auto ekf::RobotEkf::reset(const Vec_m &obs) -> Vec_n {
//   auto [x, y, z, yaw] = std::make_tuple(obs.x(), obs.y(), obs.z(), obs.w());
//   double r = this->param_getter_(name_)(Param::initial_radius).get(0.2);
//   double xc = x + r * std::cos(yaw);
//   double yc = y + r * std::sin(yaw);
//   Eigen::Vector<double, 11> x0;
//   x0 << xc, 0, yc, 0, z, 0, yaw, 0, r, r, 0;
//   this->init(x0);
//   return x0.eval();
// }
//
// ekf::OutpostEkf::OutpostEkf(ParamGetter param_getter, const std::string
// &name,
//                             std::vector<int> angle_pos_x,
//                             std::vector<int> angle_pos_z)
//     : TargetEkf(param_getter, name, angle_pos_x, angle_pos_z) {
//   this->f = [](const Vec_n &x, double dt) {
//     Vec_n x_prior = x;
//     x_prior(X_X) = x(X_X) + dt * x(X_VX);
//     x_prior(X_Y) = x(X_Y) + dt * x(X_VY);
//     x_prior(X_Z) = x(X_Z) + dt * x(X_VZ);
//     x_prior(X_YAW) = x(X_YAW) + dt * x(X_VYAW);
//     return x_prior;
//   };
//   this->get_f_jacobian = [](const Vec_n &x, double dt) {
//     Mat_nxn F;
//     F.setIdentity();
//     F(X_X, X_VX) = dt;     // x = x + vx * dt
//     F(X_Y, X_VY) = dt;     // y = y + vy * dt
//     F(X_Z, X_VZ) = dt;     // z = z + vz * dt
//     F(X_YAW, X_VYAW) = dt; // theta = theta + omega * dt
//     return F;
//   };
//   this->get_q = [this](const Vec_n &x_prior, double dt) {
//     auto s2qx = this->param_getter_(name_)(Param::q_sigma2_x).get(10.0);
//     auto s2qy = this->param_getter_(name_)(Param::q_sigma2_y).get(10.0);
//     auto s2qz = this->param_getter_(name_)(Param::q_sigma2_z).get(10.0);
//     auto s2qyaw = this->param_getter_(name_)(Param::q_sigma2_yaw).get(0.1);
//     auto s2qr = this->param_getter_(name_)(Param::q_sigma2_r).get(0.0001);
//     auto s2qdz = this->param_getter_(name_)(Param::q_sigma2_dz).get(0.0001);
//
//     double q_x_x = pow(dt, 4) / 4 * s2qx, q_x_vx = pow(dt, 3) / 2 * s2qx,
//            q_vx_vx = pow(dt, 2) * s2qx;
//     double q_y_y = pow(dt, 4) / 4 * s2qy, q_y_vy = pow(dt, 3) / 2 * s2qy,
//            q_vy_vy = pow(dt, 2) * s2qy;
//     double q_z_z = pow(dt, 4) / 4 * s2qz, q_z_vz = pow(dt, 3) / 2 * s2qz,
//            q_vz_vz = pow(dt, 2) * s2qz;
//     // double q_z_z = pow(dt, 4) / 4 * s2qx, q_z_vz = pow(dt, 3) / 2 * s2qx,
//     //        q_vz_vz = pow(dt, 2) * s2qz;
//     double q_yaw_yaw = pow(dt, 4) / 4 * s2qyaw,
//            q_yaw_vyaw = pow(dt, 3) / 2 * s2qx,
//            q_vyaw_vyaw = pow(dt, 2) * s2qyaw;
//     double q_r = pow(dt, 4) / 4 * s2qr;
//     double q_d_zc = pow(dt, 4) / 4 * s2qdz;
//     Mat_nxn Q;
//     // clang-format off
//     //    xc      v_xc    yc      v_yc    zc      v_zc    yaw         v_yaw
//     r1      dz1     dz2 Q <<  q_x_x,  q_x_vx, 0,      0,      0,      0, 0,
//     0,          0,      0,     0,
//           q_x_vx, q_vx_vx,0,      0,      0,      0,      0,          0, 0,
//           0,     0, 0,      0,      q_y_y,  q_y_vy, 0,      0,      0, 0, 0,
//           0,     0, 0,      0,      q_y_vy, q_vy_vy,0,      0,      0, 0, 0,
//           0,     0, 0,      0,      0,      0,      q_z_z,  q_z_vz, 0, 0, 0,
//           0,     0, 0,      0,      0,      0,      q_z_vz, q_vz_vz,0, 0, 0,
//           0,     0, 0,      0,      0,      0,      0,      0, q_yaw_yaw,
//           q_yaw_vyaw, 0,      0,     0, 0,      0,      0,      0,      0, 0,
//           q_yaw_vyaw, q_vyaw_vyaw,0,      0,     0, 0,      0,      0, 0, 0,
//           0,      0,          0,          q_r,    0,     0, 0,      0, 0, 0,
//           0,      0,      0,          0,          0,      q_d_zc,   0, 0, 0,
//           0,      0,      0,      0,      0,          0,          0,      0,
//           q_d_zc;
//     // clang-format on
//     return Q;
//   };
//   this->get_r = [this](const Vec_m &z) {
//     auto r_xyz_factor =
//         this->param_getter_(name_)(Param::r_xyz_factor).get(0.05);
//     auto r_yaw = this->param_getter_(name_)(Param::r_yaw).get(0.02);
//     Mat_mxm R;
//     R.diagonal() << std::abs(r_xyz_factor * z(Z_X)),
//         std::abs(r_xyz_factor * z(Z_Y)), std::abs(r_xyz_factor * z(Z_Z)),
//         r_yaw;
//     return R;
//   };
//   this->get_h_ = [](int armor_index) -> std::function<Vec_m(const Vec_n &)> {
//     // 由编号从机器人yaw得到装甲板朝向、半径和高度
//     // 对侧面的一对装甲板使用r2dz
//     return [armor_index](const Vec_n &x) -> Vec_m {
//       auto armor_yaw =
//           limitRadian(x(X_YAW) + armor_index * 2. * std::numbers::pi / 3.);
//       double dz = 0;
//       if (armor_index == 0) {
//         dz = 0;
//       } else if (armor_index == 1) {
//         dz = x(X_DZ1);
//       } else if (armor_index == 2) {
//         dz = x(X_DZ2);
//       }
//       auto [r, armor_z] = std::make_pair(x(X_R), x(X_Z) + dz);
//       auto armor_x = x(X_X) - r * std::cos(armor_yaw);
//       auto armor_y = x(X_Y) - r * std::sin(armor_yaw);
//       Vec_m hx{armor_x, armor_y, armor_z, armor_yaw};
//       return hx;
//     };
//   };
//   this->get_get_h_jacobian_ =
//       [](int armor_index) -> std::function<Mat_mxn(const Vec_n &)> {
//     return [armor_index](const Vec_n &x) -> Mat_mxn {
//       auto armor_yaw =
//           limitRadian(x(X_YAW) + armor_index * 2. * std::numbers::pi / 3.);
//       auto r = x(X_R);
//       Mat_mxn JH;
//       JH.setZero();
//       JH(Z_X, X_X) = 1.;
//       JH(Z_Y, X_Y) = 1.;
//       JH(Z_Z, X_Z) = 1.;
//       JH(Z_YAW, X_YAW) = 1.;
//       JH(Z_X, X_YAW) = r * std::sin(armor_yaw);
//       JH(Z_Y, X_YAW) = -r * std::cos(armor_yaw);
//       JH(Z_X, X_R) = -std::cos(armor_yaw);
//       JH(Z_Y, X_R) = -std::sin(armor_yaw);
//       JH(Z_Z, X_DZ1) = armor_index == 1 ? 1. : 0.;
//       JH(Z_Z, X_DZ2) = armor_index == 2 ? 1. : 0.;
//       return JH;
//     };
//   };
// }
//
// ekf::OutpostEkf::Vec_n ekf::OutpostEkf::update(const Vec_m &obs,
//                                                int armor_index) {
//   std::tie(this->h, this->get_h_jacobian) = std::make_pair(
//       this->get_h_(armor_index), get_get_h_jacobian_(armor_index));
//   // 前哨站转速特判
//   if (std::fabs(x(X_YAW)) > 2.) {
//     x(X_YAW) = x(X_YAW) > 0 ? 0.8 * std::numbers::pi : -0.8 *
//     std::numbers::pi;
//   }
//   Vec_n x_post = ExtendedKalmanFilter<X_DIM, OBS_DIM>::update(obs).eval();
//   // Prevent radius from spreading
//   auto r_max = this->param_getter_(name_)(Param::max_radius).get(0.28);
//   auto r_min = this->param_getter_(name_)(Param::min_radius).get(0.26);
//   x_post(rm_ultra_tools::ekf::OutpostEkf::X_R) =
//       std::clamp(x_post(rm_ultra_tools::ekf::RobotEkf::X_R1), r_min, r_max);
//   this->setState(x_post);
//   return x_post;
// }
//
// auto ekf::OutpostEkf::getKinematicFuncXyza()
//     -> std::vector<std::function<Eigen::Vector4d(const Vec_n &x, double dt)>>
//     {
//   std::vector<std::function<Eigen::Vector4d(const Vec_n &x, double)>> fns;
//   for (auto &&idx : std::array{0, 1, 2}) {
//     fns.emplace_back([idx, this](const Vec_n &x, double dt) ->
//     Eigen::Vector4d {
//       Vec_n state_pre = f(x, dt);
//       std::vector<Eigen::Vector4d> armors;
//       return this->get_h_(idx)(state_pre);
//     });
//   }
//   return fns;
// }
//
// auto ekf::OutpostEkf::reset(const Vec_m &obs) -> Vec_n {
//   auto [x, y, z, yaw] = std::make_tuple(obs.x(), obs.y(), obs.z(), obs.w());
//   double xc = x + OUTPOST_RADIUS * std::cos(yaw);
//   double yc = y + OUTPOST_RADIUS * std::sin(yaw);
//   Eigen::Vector<double, 11> x0;
//   x0 << xc, 0, yc, 0, z, 0, yaw, 0, OUTPOST_RADIUS, 0., 0.;
//   this->init(x0);
//   return x0;
// }
//
// } // namespace rm_ultra_tools
