#pragma once

#include "ballistic_models.hpp"

#include <Eigen/Eigen>

#include <optional>
#include <tuple>

namespace tools {

struct BallisticState2D {
  double distance;
  double height;
  double pitch;
  double velocity;
  BallisticState2D() = default;
  BallisticState2D(double distance, double height, double pitch, double v)
      : distance(distance), height(height), pitch(pitch), velocity(v) {}
  BallisticState2D(std::tuple<double, double, double, double> tuple)
      : distance(std::get<ballistic_models::DISTANCE>(tuple)),
        height(std::get<ballistic_models::HIEGHT>(tuple)),
        pitch(std::get<ballistic_models::PITCH>(tuple)),
        velocity(std::get<ballistic_models::VELOCITY>(tuple)) {}
  auto toTuple() const {
    return std::make_tuple(distance, height, pitch, velocity);
  }
};

struct BallisticParams {
  double g = 9.8;
  double k = 0.01903;
  double time_step = 0.0004;
  double barrel_length = 0.107;
};

struct PitchFlytime {
  double pitch;
  double fly_time;
};

class BallisticTrajectorySolver {
public:
  BallisticTrajectorySolver(const BallisticParams &params);
  BallisticState2D getPos2DByT(const BallisticState2D &pos0, double time) const;
  BallisticState2D rk45SingleStep(const BallisticState2D &pos0) const;
  Eigen::Vector3d getPosXyzByT(const BallisticState2D &pos0, double yaw,
                               double time) const;
  static bool isExceedTargetRange(const tools::BallisticState2D &bullet_pos2d,
                                  const Eigen::Vector3d &target_pos3d);
  static bool isExceedTargetRange(const Eigen::Vector3d &bullet_pos2d,
                                  const Eigen::Vector3d &target_pos3d);
  BallisticState2D
  transformPos2DGimbalToBarrel(const BallisticState2D &pos2d) const;
  std::optional<PitchFlytime> getAnalyticalAimingSolution(double v0, double d,
                                                          double h) const;

  BallisticParams params_;
};

} // namespace tools
