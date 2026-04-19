#pragma once

#include "msgs/BuffBlade.hpp"
#include "msgs/Header.hpp"
#include "types/BuffBladeType.hpp"
#include "types/EnemyColor.hpp"

#include "iceoryx_posh/popo/sample.hpp"
#include "opencv2/core/types.hpp"
#include <Eigen/Dense>
#include <gtsam/geometry/Rot2.h>
#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <functional>
#include <vector>

namespace auto_buff {

// NOTE:
//                ^ z
//                |
//                0
//                |
//        1       |        4
//                |
// <--------------x
// y
//           2          3
enum class BuffBladeIndex {
  _0 = 0,
  _1,
  _2,
  _3,
  _4,
};

// Rune object points
// r_tag, bottom_right, top_right, top_left, bottom_left
inline const std::vector<cv::Point3f> BUFF_BLADE_OBJ_POINTS{
    cv::Point3f(0, 0, 0) / 1000,        cv::Point3f(0, -186, 541.5) / 1000,
    cv::Point3f(0, -160, 858.5) / 1000, cv::Point3f(0, 160, 858.5) / 1000,
    cv::Point3f(0, 186, 541.5) / 1000,
};
// 风车旋转中心到击打中心的距离
constexpr auto BUFF_RADIUS{0.7};
inline const cv::Point3f BUFF_BLADE_HIT_OBJ_POINT(0, 0, BUFF_RADIUS);

enum class BuffPointPosition {
  Center,
  BottomRight,
  TopRight,
  TopLeft,
  BottomLeft,
};

struct BuffBladePoints {
  cv::Point2f r_center;
  cv::Point2f bottom_right;
  cv::Point2f top_right;
  cv::Point2f top_left;
  cv::Point2f bottom_left;
};

// 从msg构造，用于接受信息 0
struct BuffBlade {
  BuffBlade() = default;
  BuffBlade(const iox::popo::Sample<const msgs::BuffBlade, const msgs::Header>
                &sample);
  std::string frame_id;
  std::chrono::system_clock::time_point stamp;
  bool heart_beat;
  types::EnemyColor color;
  types::BuffBladeType type;
  float confidence;
  BuffBladePoints points;
  // NOTE: 有需要的随时再添加
};

// 保留位置信息和是否需要激活（用于选择击打扇叶）
// 用在弹道解算中 2
struct BladePositionRoll {
  types::BuffBladeType type;
  Eigen::Vector3d position;
  // 竖直向上时roll为0，逆时针
  gtsam::Rot2 roll;
  // 从扇叶获得瞄准点
  Eigen::Vector3d getHitPosition() const;
};

// 添加了角点信息，用在Target中 1
struct BladePositionRPYPoints : BladePositionRoll {
  double pitch;
  double yaw;
  std::array<cv::Point2f, 5> points;
  Eigen::Quaterniond getRotation() const;
  BladePositionRPYPoints transform(const Eigen::Isometry3d &T) const;
};

struct BuffState {
  BuffState() = default;
  Eigen::Vector3d center_position = Eigen::Vector3d::Zero();
  double center_roll{0};
  std::array<bool, 5> inactivated_flag{false, false, false, false, false};
  // 大小符的旋转模式不同，设计一个类型擦除统一下
  BuffState getStateWithPredictFunc(
      std::function<BuffState(const BuffState &, double)> &&func) const;
  std::array<BladePositionRoll, 5> blades() const;

private:
  friend class Trajectory;  // 只允许轨迹规划类访问预测方法
  friend class TrackerNode; // 可视化绘制也需要预判
  BuffState predict(double dt) const;
  std::function<BuffState(const BuffState &self, double dt)> predict_fn_;
};

struct SmallBuffState : public BuffState {
  SmallBuffState() = default;
  SmallBuffState(const BuffState &state, double vroll);
  double center_vroll{0};
};

struct BigBuffState : public BuffState {
  double theta{0};
  double omega{0};
};

struct TrackState {
  enum class State {
    LOST,
    TEMPLOST,
    TRACKING,
  } state{State::LOST};
  std::uint64_t k{0};
  std::chrono::system_clock::time_point stamp_last_update;
  std::chrono::system_clock::time_point stamp_last_tracking;
};

struct BuffMatchResult {
  BuffBladeIndex index;
  double distance;
  double roll_diff;
};

struct YawPitchFlyTimeIndex {
  double yaw;
  double pitch;
  double fly_time;
  BuffBladeIndex index;
};

struct BuffIndexPredictTime {
  BuffBladeIndex index;
  double predict_time;
};

} // namespace auto_buff
