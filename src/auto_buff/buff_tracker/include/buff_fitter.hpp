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

  std::optional<std::array<double, 5>> update(double dt_last_update_to_image,
                                              double buff_roll);
  void reset();

private:
  quill::Logger *logger_;
  BuffFitterConfig config_;

  std::optional<std::array<double, 5>>
  fitCurve(const std::deque<IntervalTimeBuffRoll> &data_history_queue,
           const std::array<double, 5> &initial_params,
           BuffDirection direction) const;

  std::jthread fitting_thread_;

  mutable std::mutex data_mtx_;
  double start_time_{0};
  double last_update_time_{0};
  BuffDirection direction_{BuffDirection::Unknown};
  std::deque<IntervalTimeBuffRoll> data_history_queue_;
  std::array<double, 5> fitting_param_;
  bool fitting_ok_{false};
};

} // namespace auto_buff
