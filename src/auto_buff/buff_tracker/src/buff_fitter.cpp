#include "buff_fitter.hpp"
#include "configs.hpp"
#include "types.hpp"

#include "iox/signal_watcher.hpp"
#include "quill/LogMacros.h"

#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <tuple>

namespace auto_buff {
struct BigRuneFittingCost {
  BigRuneFittingCost(double x, double y, double m)
      : x_(x), y_(y), mov_(m > 0 ? 1 : -1) {}
  template <typename T> bool operator()(const T *const p, T *residual) const {
    const T pred = getBuffCurvePoint(x_, p[0], p[1], p[2], p[3], mov_);
    // Use angular distance in unit circle space to avoid wrap-around jumps.
    residual[0] = ceres::sin(T(y_)) - ceres::sin(pred);
    residual[1] = ceres::cos(T(y_)) - ceres::cos(pred);
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
            auto vroll = vroll_;
            return std::tuple{data, params, vroll};
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

std::optional<auto_buff::BuffParamVRollDeltaTime>
auto_buff::BuffFitter::update(double dt_last_update_to_image, double buff_roll,
                              double buff_vroll) {
  std::scoped_lock lk{data_mtx_};
  data_history_queue_.emplace_back(IntervalTimeBuffRoll{
      .dt_from_start = dt_start_to_last_update_ + dt_last_update_to_image,
      .buff_roll = buff_roll,
  });
  dt_start_to_last_update_ += dt_last_update_to_image;
  if (data_history_queue_.size() < config_.queue_lower_limit)
    return std::nullopt;
  if (data_history_queue_.size() > config_.queue_upper_limit)
    data_history_queue_.pop_front();

  double angle_diff = data_history_queue_.back().buff_roll -
                      data_history_queue_.front().buff_roll;
  vroll_ = buff_vroll;
  if (fitting_ok_)
    return {{
        .param = fitting_param_,
        .vroll = vroll_,
        .dt_from_start = dt_start_to_last_update_,
    }};
  return std::nullopt;
}

void auto_buff::BuffFitter::reset() {
  std::scoped_lock lk{data_mtx_};
  this->dt_start_to_last_update_ = 0;
  this->vroll_ = 0;
  this->data_history_queue_.clear();
  this->fitting_param_ = {0.9125, 1.942, 0, 0};
  this->fitting_ok_ = false; // NOTE: 在此处重置拟合状态
}

std::optional<std::array<double, 4>> auto_buff::BuffFitter::fitCurve(
    const std::deque<IntervalTimeBuffRoll> &data_history_queue,
    const std::array<double, 4> &initial_params, double vroll) const {
  if (data_history_queue.size() < config_.queue_lower_limit)
    return std::nullopt;
  ceres::Problem problem;
  auto param = initial_params;
  auto param_ptr = param.data();
  std::for_each( // 添加每个采样点的约束
      data_history_queue.begin(), data_history_queue.end(),
      [&](const auto &data) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<BigRuneFittingCost, 2, 4>(
                new BigRuneFittingCost(data.dt_from_start, data.buff_roll,
                                       vroll)),
            new ceres::CauchyLoss(0.5), param_ptr);
      });
  // 约束参数范围
  problem.SetParameterLowerBound(param_ptr, 0,
                                 0.780 * config_.param_lower_bound_scale);
  problem.SetParameterUpperBound(param_ptr, 0,
                                 1.045 * config_.param_upper_bound_scale);
  problem.SetParameterLowerBound(param_ptr, 1,
                                 1.884 * config_.param_lower_bound_scale);
  problem.SetParameterUpperBound(param_ptr, 1,
                                 2.000 * config_.param_upper_bound_scale);
  // problem.SetParameterLowerBound(
  //     param_ptr, 2, (2.090 - 1.045) * config_.param_lower_bound_scale);
  // problem.SetParameterUpperBound(
  //     param_ptr, 2, (2.090 - 0.780) * config_.param_upper_bound_scale);
  // 求解问题
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  return param;
}
