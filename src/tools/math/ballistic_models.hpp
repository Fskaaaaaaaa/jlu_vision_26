#pragma once

#include <cmath>
#include <tuple>
#include <utility>
namespace tools {
namespace ballistic_models {

auto rk45SingleStep(auto &&pos2d, auto &&time_step, auto &&k, auto &&g) {
  auto &&[distance0, height0, pitch0, v0] = pos2d;
  auto pitch_theta = pitch0;
  auto v0_d = v0;
  auto time_step_ = time_step;
  auto k1_v = (-k * std::pow(v0_d, 2) - g * std::sin(pitch_theta)) * time_step_;
  auto k1_theta =
      (-g * std::cos(pitch_theta) / v0_d) *
      time_step_; // 由于大弹丸很难产生马格努斯效应（横向摩擦轮怎么可能产生后向旋转），故忽略升力
  // 其实这里和公式是不完全相同的，未考虑t随步长的改变，因为t在推导过程中简化约去了

  auto k1_v_2 = v0_d + k1_v / 4.0;
  auto k1_theta_2 = pitch_theta + k1_theta / 4.0;

  auto k2_v =
      (-k * std::pow(k1_v_2, 2) - g * std::sin(k1_theta_2)) * time_step_;
  auto k2_theta = (-g * std::cos(k1_theta_2) / k1_v_2) * time_step_;
  auto k12_v_3 = v0_d + 3.0 / 32.0 * k1_v + 9.0 / 32.0 * k2_v;
  auto k12_theta_3 =
      pitch_theta + 3.0 / 32.0 * k1_theta + 9.0 / 32.0 * k2_theta;

  auto k3_v =
      (-k * std::pow(k12_v_3, 2) - g * std::sin(k12_theta_3)) * time_step_;
  auto k3_theta = (-g * cos(k12_theta_3) / k12_v_3) * time_step_;
  auto k123_v_4 = v0_d + 1932.0 / 2179.0 * k1_v - 7200.0 / 2179.0 * k2_v +
                  7296.0 / 2179.0 * k3_v;
  auto k123_theta_4 = pitch_theta + 1932.0 / 2179.0 * k1_theta -
                      7200.0 / 2179.0 * k2_theta + 7296.0 / 2179.0 * k3_theta;

  auto k4_v =
      (-k * std::pow(k123_v_4, 2) - g * std::sin(k123_theta_4)) * time_step_;
  auto k4_theta = (-g * std::cos(k123_theta_4) / k123_v_4) * time_step_;
  auto k1234_v_5 = v0_d + 439.0 / 216.0 * k1_v - 8.0 * k2_v +
                   3680.0 / 513.0 * k3_v - 845.0 / 4140.0 * k4_v;
  auto k1234_theta_5 = pitch_theta + 439.0 / 216.0 * k1_theta - 8.0 * k2_theta +
                       3680.0 / 513.0 * k3_theta - 845.0 / 4140.0 * k4_theta;

  auto k5_v =
      (-k * std::pow(k1234_v_5, 2) - g * std::sin(k1234_theta_5)) * time_step_;
  auto k5_theta = (-g * cos(k1234_theta_5) / k1234_v_5) * time_step_;
  auto k12345_v_6 = v0_d - 8.0 / 27.0 * k1_v + 2.0 * k2_v -
                    3544.0 / 2565.0 * k3_v + 1859.0 / 4104.0 * k4_v -
                    11.0 / 40.0 * k5_v;
  auto k12345_theta_6 = pitch_theta - 8.0 / 27.0 * k1_theta + 2.0 * k2_theta -
                        3544.0 / 2565.0 * k3_theta +
                        1859.0 / 4104.0 * k4_theta - 11.0 / 40.0 * k5_theta;

  auto k6_v = (-k * std::pow(k12345_v_6, 2) - g * std::sin(k12345_theta_6)) *
              time_step_;
  auto k6_theta = (-g * std::cos(k12345_theta_6) / k12345_v_6) * time_step_;
  auto vclass_5 = v0_d + 16.0 / 135.0 * k1_v + 6656.0 / 12825.0 * k3_v +
                  28561.0 / 56430.0 * k4_v - 9.0 / 50.0 * k5_v +
                  2.0 / 55.0 * k6_v;
  auto thetaclass_5 = pitch_theta + 16.0 / 135.0 * k1_theta +
                      6656.0 / 12825.0 * k3_theta +
                      28561.0 / 56430.0 * k4_theta - 9.0 / 50.0 * k5_theta +
                      2.0 / 55.0 * k6_theta;

  return std::make_tuple(
      distance0 + vclass_5 * std::cos(thetaclass_5) * time_step_,
      height0 + vclass_5 * std::sin(thetaclass_5) * time_step_, thetaclass_5,
      vclass_5);
}

enum Pose {
  DISTANCE,
  HIEGHT,
  PITCH,
  VELOCITY,
};

auto getPos2DByT(auto &&pos0, auto &&time, auto &&time_step, auto &&k,
                 auto &&g) {
  auto result2d = pos0;
  auto time_d = decltype(time){};
  while (time_d + time_step < time) {
    result2d = rk45SingleStep(result2d, time_step, k, g);
    time_d += time_step;
  }
  auto diff = time - time_d;
  return diff > time_step / 10. ? rk45SingleStep(result2d, time - time_d, k, g)
                                : result2d;
}
auto transformPos2DToXyz(auto &&pos2d, auto &&yaw_rad) {
  auto &&pos2d_fwd = std::forward<decltype(pos2d)>(pos2d);
  auto &&[distance, height, pitch, v] = pos2d_fwd;
  auto &&yaw_fwd = std::forward<decltype(yaw_rad)>(yaw_rad);
  auto x = distance * std::cos(yaw_fwd);
  auto y = distance * std::sin(yaw_fwd);
  auto z = height;
  return std::make_tuple(std::forward<decltype(x)>(x),
                         std::forward<decltype(y)>(y),
                         std::forward<decltype(z)>(z));
}

auto getXyzByT(auto &&pos0, auto &&yaw, auto &&time, auto &&time_step, auto &&k,
               auto &&g) {
  auto pos2d = getPos2DByT(
      std::forward<decltype(pos0)>(pos0), std::forward<decltype(time)>(time),
      std::forward<decltype(time_step)>(time_step),
      std::forward<decltype(k)>(k), std::forward<decltype(g)>(g));
  auto xyz = transformPos2DToXyz(pos2d, std::forward<decltype(yaw)>(yaw));
  return xyz;
}

auto transformPos2DGimbelToBarrel(auto &&distance, auto &&height,
                                  auto &&gimbal_pitch,
                                  auto &&length_gimbal2barrel) {
  distance += length_gimbal2barrel * std::cos(gimbal_pitch);
  height += length_gimbal2barrel * std::sin(gimbal_pitch);
  return std::make_pair(distance, height);
}

} // namespace ballistic_models
} // namespace tools
