#pragma once

#include "ballistic_models.hpp"
#include "quill/Logger.h"

#include <Eigen/Eigen>

#include <optional>

namespace tools {

namespace ballistic {

struct BallisticConfig {
  double g = 9.8;
  double k = 0.01903;
  double barrel_length = 0.107;
  double time_step = 0.0004;
  double max_fly_time = 0.8;
  int max_pitch_iterate_count = 100;
  double min_pitch_error_m = 0.008;
  double gimbal_pitch_min_degree = -5.0;
  double gimbal_pitch_max_degree = 30.0;
};

struct PitchFlytime {
  double pitch;
  double fly_time;
};

enum class Method {
  rk45,
  parabola,
};

class BallisticTrajectorySolver {
public:
  BallisticTrajectorySolver(quill::Logger *logger,
                            const BallisticConfig &config)
      : logger_(logger), config_(config) {};

  std::optional<PitchFlytime>
  resolvePitchFlyTime(double distance, double height, double v0,
                      Method method = Method::parabola) const;
  BallisticState2D getBarrelStateFromPitch(double pitch, double v0) const;

private:
  std::optional<PitchFlytime> resolveRk45(double distance, double height,
                                          double v0) const;
  std::optional<PitchFlytime> resolveParabola(double distance, double height,
                                              double v0) const;
  quill::Logger *logger_;
  BallisticConfig config_;
};
} // namespace ballistic

} // namespace tools
