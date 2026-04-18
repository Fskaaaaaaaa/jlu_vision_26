#include "types.hpp"
#include "math/angle_tools.hpp"
#include "types/BuffBladeType.hpp"

#include <array>
#include <cmath>
#include <gtsam/geometry/Rot2.h>
#include <numbers>

Eigen::Vector3d auto_buff::BladePositionRoll::getHitPosition() const {
  auto z_hit = std::cos(this->roll.theta()) * BUFF_RADIUS + this->position.z();
  auto horizontal_bias = std::sin(this->roll.theta()) * BUFF_RADIUS;
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
        .roll = center_roll + i * std::numbers::pi / 5,
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
