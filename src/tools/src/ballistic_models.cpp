#include "math/ballistic_models.hpp"

#include <cmath>

tools::ballistic::BallisticState3D::BallisticState3D(
    const BallisticState2D &state2d, double yaw)
    : BallisticState2D(state2d), yaw(yaw) {
  position.x() = std::cos(yaw) * distance;
  position.y() = std::sin(yaw) * distance;
  position.z() = height;
}

bool tools::ballistic::isExceedTargetRange(
    const BallisticState2D &bullet_state2d, double target_distance,
    double target_height) {
  return std::hypot(bullet_state2d.distance, bullet_state2d.height) >=
         std::hypot(target_distance, target_height);
}

bool tools::ballistic::isExceedTargetRange(
    const BallisticState2D &bullet_pos2d, const Eigen::Vector3d &target_pos3d) {
  Eigen::Vector2d eg_target2d{std::hypot(target_pos3d.x(), target_pos3d.y()),
                              target_pos3d.z()};
  Eigen::Vector2d eg_bullet_pos2d{bullet_pos2d.distance, bullet_pos2d.height};
  return eg_bullet_pos2d.norm() >= eg_target2d.norm();
}

bool tools::ballistic::isExceedTargetRange(
    const BallisticState3D &bullet_pos3d, const Eigen::Vector3d &target_pos3d) {
  return isExceedTargetRange(static_cast<BallisticState2D>(bullet_pos3d),
                             target_pos3d);
}

tools::ballistic::BallisticState2D
tools::ballistic::rk45::rk45SingleStep(const BallisticState2D &state2d,
                                       double time_step, double k, double g) {
  auto [distance0, height0, pitch0, v0] = state2d;
  auto pitch_theta = pitch0;
  auto v0_d = v0;
  auto time_step_ = time_step;
  auto k1_v = (-k * std::pow(v0_d, 2) - g * std::sin(pitch_theta)) * time_step_;
  auto k1_theta = (-g * std::cos(pitch_theta) / v0_d) * time_step_;
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
  return {distance0 + vclass_5 * std::cos(thetaclass_5) * time_step_,
          height0 + vclass_5 * std::sin(thetaclass_5) * time_step_,
          thetaclass_5, vclass_5};
}

tools::ballistic::BallisticState2D
tools::ballistic::rk45::getState2DByT(const BallisticState2D &state0,
                                      double time, double time_step, double k,
                                      double g) {
  auto result2d = state0;
  double time_d{0.0};
  while (time_d + time_step < time) {
    result2d = rk45SingleStep(result2d, time_step, k, g);
    time_d += time_step;
  }
  auto diff = time - time_d;
  return diff > time_step / 10. ? rk45SingleStep(result2d, time - time_d, k, g)
                                : result2d;
}

tools::ballistic::BallisticState3D
tools::ballistic::rk45::getState3DByT(const BallisticState2D state0, double yaw,
                                      double time, double time_step, double k,
                                      double g) {
  auto state2d = getState2DByT(state0, time, time_step, k, g);
  BallisticState3D state3d{state2d, yaw};
  return state3d;
}

tools::ballistic::BallisticState2D
tools::ballistic::parabola::getState2DByT(const BallisticState2D &state0,
                                          double time, double g) {
  auto [distance0, height0, pitch0, v0] = state0;
  auto vx = v0 * std::cos(pitch0);
  auto vy0 = v0 * std::sin(pitch0);
  auto vy = vy0 - g * time;
  auto x = vx * time + distance0;
  auto y = vy0 * time - 0.5 * g * time * time + height0;
  auto pitch = std::atan2(vy, vx);
  auto v = std::hypot(vx, vy);
  return {x, y, pitch, v};
}

tools::ballistic::BallisticState3D
tools::ballistic::parabola::getState3DByT(const BallisticState2D state0,
                                          double yaw, double time, double g) {
  auto state2d = getState2DByT(state0, time, g);
  BallisticState3D state3d{state2d, yaw};
  return state3d;
}
