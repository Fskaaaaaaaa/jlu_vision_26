#include "types.hpp"
#include "types/BuffBladeType.hpp"

#include <array>
#include <cmath>
#include <numbers>

Eigen::Vector3d auto_buff::BuffBladePositionRoll::getHitPosition() const {
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

std::array<auto_buff::BuffBladePositionRoll, 5> auto_buff::BuffState::blades() {
  std::array<BuffBladePositionRoll, 5> blades;
  for (int i = 0; i < 5; i++) {
    blades.at(i) = BuffBladePositionRoll{
        .type = inactivated_flag.at(i) ? types::BuffBladeType::Inactivated
                                       : types::BuffBladeType::Activated,
        .position = center_position,
        .roll = center_roll + i * std::numbers::pi / 5,
    };
  }
  return blades;
}
