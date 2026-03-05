#pragma once

#include <Eigen/Core>

namespace tools {
namespace ballistic {

struct BallisticState2D {
  double distance;
  double height;
  double pitch;
  double velocity;
  BallisticState2D() = default;
  BallisticState2D(double distance, double height, double pitch, double v)
      : distance(distance), height(height), pitch(pitch), velocity(v) {}
};

struct BallisticState3D : public BallisticState2D {
  Eigen::Vector3d position;
  double yaw;
  BallisticState3D(const BallisticState2D &state2d, double yaw);
};

bool isExceedTargetRange(const BallisticState2D &bullet_state2d,
                         double target_distance, double target_height);
bool isExceedTargetRange(const BallisticState2D &bullet_state2d,
                         const Eigen::Vector3d &target_pos3d);
bool isExceedTargetRange(const BallisticState3D &bullet_state3d,
                         const Eigen::Vector3d &target_pos3d);

namespace rk45 {

BallisticState2D rk45SingleStep(const BallisticState2D &state2d,
                                double time_step, double k, double g);
BallisticState2D getState2DByT(const BallisticState2D &state0, double time,
                               double time_step, double k, double g);
BallisticState3D getState3DByT(const BallisticState2D state0, double yaw,
                               double time, double time_step, double k,
                               double g);
} // namespace rk45

namespace parabola {
BallisticState2D getState2DByT(const BallisticState2D &state0, double time,
                               double g);
BallisticState3D getState3DByT(const BallisticState2D state0, double yaw,
                               double time, double g);
} // namespace parabola
} // namespace ballistic
} // namespace tools
