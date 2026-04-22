#include "buff_fitter.hpp"
#include "configs.hpp"
#include "types.hpp"

#include "iox/signal_watcher.hpp"
#include "quill/LogMacros.h"

#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <tuple>

namespace auto_buff {
struct BigRuneFittingCost {
  BigRuneFittingCost(double x, double y, int m)
      : x_(x), y_(y), mov_(static_cast<double>(m)) {}
  template <typename T> bool operator()(const T *const p, T *residual) const {
    residual[0] = y_ - getBuffCurve(x_, p[0], p[1], p[2], p[3], p[4], mov_);
    return true;
  }
  const double x_, y_;
  const double mov_;
};
} // namespace auto_buff

auto_buff::BuffFitter::BuffFitter(quill::Logger *logger,
                                  const BuffFitterConfig &config)
    : logger_(logger), config_(config) {
  this->reset();
  this->fitting_thread_ = std::jthread{[this]() {
    LOG_INFO(logger_, "[BuffFitter]: Fitting thread start!");
    while (!iox::hasTerminationRequested()) {
      // 获取快照
      auto [data_snapshot, params_snapshot, direction_snapshot] =
          std::invoke([this]() {
            std::scoped_lock lk{data_mtx_};
            auto data = data_history_queue_;
            auto params = fitting_param_;
            auto direction = direction_;
            return std::tuple{data, params, direction};
          });
      // 进行拟合
      auto start_time = std::chrono::system_clock::now();
      auto result_opt =
          fitCurve(data_snapshot, params_snapshot, direction_snapshot);
      auto finish_time = std::chrono::system_clock::now();
      LOG_DEBUG(logger_, "[BuffFitter]: Fitting time: {} ms",
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    finish_time - start_time)
                    .count());
      // 更新优化结果
      if (result_opt.has_value()) {
        std::scoped_lock lk{data_mtx_};
        this->fitting_param_ = result_opt.value();
        this->fitting_ok_ = true; // NOTE: 在此处更新拟合成功
        LOG_TRACE_L1(logger_, "[BuffFitter]: Fitting praram updated!");
      }
      std::this_thread::sleep_for(
          std::chrono::milliseconds{config_.curve_fitting_interval_time_ms});
    }
    LOG_INFO(logger_, "[BuffFitter]: Fitting thread stop!");
  }};
}

std::optional<std::array<double, 5>>
auto_buff::BuffFitter::update(double dt_last_update_to_image,
                              double buff_roll) {
  std::scoped_lock lk{data_mtx_};
  data_history_queue_.emplace_back(IntervalTimeBuffRoll{
      .interval_time = last_update_time_ + dt_last_update_to_image,
      .buff_roll = buff_roll,
  });
  if (data_history_queue_.size() < config_.queue_lower_limit)
    return std::nullopt;
  if (data_history_queue_.size() > config_.queue_upper_limit)
    data_history_queue_.pop_front();

  double angle_diff = data_history_queue_.back().buff_roll -
                      data_history_queue_.front().buff_roll;
  direction_ =
      angle_diff < 0 ? BuffDirection::ClockWise : BuffDirection::AntiClockWise;
  if (fitting_ok_)
    return fitting_param_;
  return std::nullopt;
}

void auto_buff::BuffFitter::reset() {
  std::scoped_lock lk{data_mtx_};
  this->start_time_ = 0;
  this->last_update_time_ = 0;
  this->direction_ = BuffDirection::Unknown;
  this->data_history_queue_.clear();
  this->fitting_param_ = {0.9125, 1.942, 2.090 - 0.9125, 0, 0};
  this->fitting_ok_ = false; // NOTE: 在此处重置拟合状态
}

std::optional<std::array<double, 5>> auto_buff::BuffFitter::fitCurve(
    const std::deque<IntervalTimeBuffRoll> &data_history_queue,
    const std::array<double, 5> &initial_params,
    BuffDirection direction) const {
  if (data_history_queue.size() < config_.queue_lower_limit)
    return std::nullopt;
  ceres::Problem problem;
  auto param = initial_params;
  auto param_ptr = param.data();
  std::for_each( // 添加每个采样点的约束
      data_history_queue.begin(), data_history_queue.end(),
      [&](const auto &data) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<BigRuneFittingCost, 1, 5>(
                new BigRuneFittingCost(data.interval_time, data.buff_roll,
                                       static_cast<int>(direction))),
            new ceres::CauchyLoss(0.5), param_ptr);
      });
  // 约束参数范围
  problem.SetParameterLowerBound(param_ptr, 0, 0.780 * 0.5);
  problem.SetParameterUpperBound(param_ptr, 0, 1.045 * 1.5);
  problem.SetParameterLowerBound(param_ptr, 1, 1.884 * 0.5);
  problem.SetParameterUpperBound(param_ptr, 1, 2.000 * 1.5);
  problem.SetParameterLowerBound(param_ptr, 2, (2.090 - 1.045) * 0.5);
  problem.SetParameterUpperBound(param_ptr, 2, (2.090 - 0.780) * 1.5);
  // 求解问题
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  return param;
}
