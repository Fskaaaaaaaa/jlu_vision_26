#pragma once

#include "configs.hpp"
#include "types.hpp"

#include "quill/Logger.h"
#include <ceres/ceres.h>

#include <array>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>

namespace auto_buff {
class BuffFitter {
public:
  BuffFitter(quill::Logger *logger, const BuffFitterConfig &config);

  std::optional<BuffParamVRollDeltaTime>
  update(double dt_last_update_to_image, double buff_roll, double buff_vroll);

  void reset();

private:
  quill::Logger *logger_;
  BuffFitterConfig config_;

  std::optional<std::array<double, 4>>
  fitCurve(const std::deque<IntervalTimeBuffRoll> &data_history_queue,
           const std::array<double, 4> &initial_params, double vroll) const;

  std::jthread fitting_thread_;

  mutable std::mutex data_mtx_;
  double dt_start_to_last_update_{0};
  double dt_start_to_last_fit_{0};
  double vroll_;
  std::deque<IntervalTimeBuffRoll> data_history_queue_;
  std::array<double, 4> fitting_param_;
  bool fitting_ok_{false};
};

} // namespace auto_buff
