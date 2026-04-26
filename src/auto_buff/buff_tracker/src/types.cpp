#include "types.hpp"
#include "basic/time_tools.hpp"
#include "math/angle_tools.hpp"
#include "types/BuffBladeType.hpp"
#include "types/EnemyColor.hpp"

#include <gtsam/geometry/Rot2.h>

#include <array>
#include <cmath>
#include <functional>
#include <numbers>

auto_buff::BuffBlade::BuffBlade(
    const iox::popo::Sample<const msgs::BuffBlade, const msgs::Header> &sample)
    : frame_id(sample.getUserHeader().frame_id.c_str()),
      stamp(tools::nanoSecToChronoPoint(sample.getUserHeader().stamp_ns)),
      heart_beat(sample->heart_beat),
      color(static_cast<types::EnemyColor>(sample->color)),
      type(static_cast<types::BuffBladeType>(sample->type)),
      confidence(sample->confidence),
      points({
          .r_center = {static_cast<float>(sample->points.r_center.x),
                       static_cast<float>(sample->points.r_center.y)},
          .bottom_right = {static_cast<float>(sample->points.bottom_right.x),
                           static_cast<float>(sample->points.bottom_right.y)},
          .top_right = {static_cast<float>(sample->points.top_right.x),
                        static_cast<float>(sample->points.top_right.y)},
          .top_left = {static_cast<float>(sample->points.top_left.x),
                       static_cast<float>(sample->points.top_left.y)},
          .bottom_left = {static_cast<float>(sample->points.bottom_left.x),
                          static_cast<float>(sample->points.bottom_left.y)},
      }) {}

Eigen::Vector3d auto_buff::BladePositionRoll::getHitPosition() const {
  auto z_hit = std::cos(this->roll.theta()) * BUFF_RADIUS + this->position.z();
  auto horizontal_bias = -std::sin(this->roll.theta()) * BUFF_RADIUS;
  auto center_aim_yaw = std::atan2(this->position.y(), this->position.x());
  // XXX: 这里需要再检查下
  auto x_hit = this->position.x() - horizontal_bias * std::sin(center_aim_yaw);
  auto y_hit = this->position.y() + horizontal_bias * std::cos(center_aim_yaw);
  return {x_hit, y_hit, z_hit};
}

auto_buff::BuffState auto_buff::BuffState::getStateWithPredictFunc(
    std::function<BuffState(const BuffState &, double)> &&func) const {
  auto state = *this;
  state.predict_fn_ = func;
  return state;
}

auto_buff::BuffState auto_buff::BuffState::predict(double dt) const {
  return predict_fn_(*this, dt);
}

std::array<auto_buff::BladePositionRoll, 5>
auto_buff::BuffState::blades() const {
  std::array<BladePositionRoll, 5> blades;
  for (int i = 0; i < 5; i++) {
    blades.at(i) = BladePositionRoll{
        .type = inactivated_flag.at(i) ? types::BuffBladeType::Inactivated
                                       : types::BuffBladeType::Activated,
        .position = center_position,
        .roll = center_roll + i * 2 * std::numbers::pi / 5,
    };
  }
  return blades;
}

Eigen::Quaterniond auto_buff::BladePositionRPYPoints::getRotation() const {
  return tools::rpyToQuaterniond({roll.theta(), pitch, yaw});
}

auto_buff::BladePositionRPYPoints
auto_buff::BladePositionRPYPoints::transform(const Eigen::Isometry3d &T) const {
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  pose.pretranslate(this->position);
  pose.rotate(this->getRotation());
  auto result = T * pose;
  auto rpy = tools::rotationMatrixToRPY(result.rotation());
  // HACK: 这里发生的拷贝太多了！！！
  BladePositionRPYPoints blade{*this};
  blade.position = result.translation();
  blade.roll = gtsam::Rot2::fromAngle(rpy(0));
  blade.pitch = rpy(1);
  blade.yaw = rpy(2);
  return blade;
}

auto_buff::SmallBuffState::SmallBuffState(const BuffState &state, double vroll)
    : BuffState(state), center_vroll(vroll) {}

auto_buff::BigBuffState::BigBuffState(const BuffState &state,
                                      double dt_from_start, double a,
                                      double omega, double b, double c,
                                      double d, double vroll)
    : BuffState(state), dt_from_start(dt_from_start), a(a), omega(omega), b(b),
      c(c), d(d), center_vroll(vroll) {}
