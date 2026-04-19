#pragma once

#include "configs.hpp"
#include "quill/Logger.h"
#include "types.hpp"

#include <gtsam/base/types.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <mutex>
#include <optional>
#include <utility>
#include <vector>
namespace auto_buff {

class BuffTarget {
public:
  BuffTarget(quill::Logger *logger, const BuffBladeMatchConfig &match_conf,
             const BuffBladeNoiseConfig &blade_config,
             const cv::Mat &camera_matrix,
             const cv::Mat &distortion_coefficients);
  virtual TrackState::State
  track(const std::vector<BuffBlade> &blades,
        const std::chrono::system_clock::time_point &stamp,
        const Eigen::Isometry3d &T_camera_to_odom) = 0;
  virtual double get(const std::string &str) const;

  virtual std::pair<BuffState, TrackState> getTargetTrackState() const = 0;

protected:
  quill::Logger *logger_;
  static BuffState getTargetStateFromBlade(const BladePositionRPYPoints &blade);
  std::optional<BladePositionRPYPoints> solvePNP(const BuffBlade &blade) const;
  std::vector<std::pair<BladePositionRPYPoints, BuffBladeIndex>>
  matchBlades(const BuffState &state,
              const std::vector<BladePositionRPYPoints> &blades_camera,
              const Eigen::Isometry3d &T_camera_to_odom) const;
  void addBladeValuesFactors(
      gtsam::Values &values, gtsam::NonlinearFactorGraph &graph,
      const std::vector<std::pair<BladePositionRPYPoints, BuffBladeIndex>>
          &blades_camera_indexs,
      const Eigen::Isometry3d &T_camera_to_odom, std::uint64_t k) const;

private:
  std::optional<BuffMatchResult>
  matchBlade(const BuffState &state,
             const BladePositionRPYPoints &blade_odom) const;
  void addBladeReprojValuesFactors(gtsam::Values &values,
                                   gtsam::NonlinearFactorGraph &graph,
                                   gtsam::Key blade_pose_key,
                                   const BladePositionRPYPoints &blade_camera,
                                   std::uint64_t k) const;
  BuffBladeMatchConfig match_config_;
  BuffBladeNoiseConfig blade_config_;

  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;
};

class SmallBuffTarget : public BuffTarget {
public:
  SmallBuffTarget(quill::Logger *logger, const SmallBuffConfig &config,
                  const cv::Mat &camera_matrix,
                  const cv::Mat &distortion_coefficients);
  std::pair<BuffState, TrackState> getTargetTrackState() const override;
  double get(const std::string &str) const override;
  TrackState::State track(const std::vector<BuffBlade> &blades,
                          const std::chrono::system_clock::time_point &stamp,
                          const Eigen::Isometry3d &T_camera_to_odom) override;

private:
  std::pair<SmallBuffState, TrackState::State>
  update(const std::vector<BladePositionRPYPoints> &blades_camera, double dt,
         const Eigen::Isometry3d &T_camera_to_odom) const;
  void addMotionValuesFactors(gtsam::Values &values,
                              gtsam::NonlinearFactorGraph &graph,
                              const SmallBuffState &state, std::uint64_t k,
                              double dt) const;
  SmallBuffState predictBuffState(double dt) const;
  // 用来注入到BuffState里，最好设成static
  static SmallBuffState predictBuffState(const BuffState &state, double vroll,
                                         double dt);

  // quill::Logger *logger_; 在基类里
  SmallBuffConfig config_;

  mutable std::mutex state_mtx_;
  mutable gtsam::ISAM2 isam2_;
  SmallBuffState target_state_;
  TrackState track_state_;
};

class BigBuffTarget : private BuffTarget {
public:
private:
};

} // namespace auto_buff
