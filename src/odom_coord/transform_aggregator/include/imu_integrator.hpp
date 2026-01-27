#pragma once

#include "configs.hpp"
#include "msgs/Basic.hpp"

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <optional>
#include <quill/Logger.h>

#include <chrono>
#include <memory>

namespace tf {

struct IsometryVel {
  Eigen::Isometry3d T;
  Eigen::Vector3d vel;
};
// 不含iox成分，纯粹的gtsam积分优化类
// 对外不要曝露gtsam成分
class ImuIntegrator {
public:
  ImuIntegrator(quill::Logger *logger, const ImuIntegratorConfig &config);
  void init(const Eigen::Quaterniond &quaternion0,
            const Eigen::Vector3d &translation0,
            const Eigen::Vector3d &v0 = {0, 0, 0},
            const std::chrono::system_clock::time_point &stamp =
                std::chrono::system_clock::now());
  IsometryVel update(const Eigen::Quaterniond &measured_ori,
                     const Eigen::Vector3d &measured_acc,
                     const Eigen::Vector3d &measured_omega,
                     const std::chrono::system_clock::time_point &stamp);

private:
  quill::Logger *logger_;
  ImuIntegratorConfig config_;
  std::shared_ptr<gtsam::PreintegratedImuMeasurements::Params> accum_params_;
  gtsam::PreintegratedImuMeasurements accum_;
  gtsam::ISAM2 isam_;
  std::optional<gtsam::NonlinearFactorGraph> init_graph_;
  gtsam::NavState realtime_state_;
  gtsam::Values optimized_values_;
  // NOTE: 分别缓存高频积分结果和低频优化结果
  int time_index_;
  std::chrono::system_clock::time_point last_update_stamp_;
  std::chrono::system_clock::time_point last_optimize_stamp_;
};
} // namespace tf
