#include "types.hpp"
#include "types/BuffBladeType.hpp"

#include <array>
#include <numbers>

Eigen::Vector3d auto_buff::BuffBladePositionRoll::getHitPosition() const {
  // TODO:
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
        .type = activated_flag.at(i) ? types::BuffBladeType::Activated
                                     : types::BuffBladeType::Inactivated,
        .position = center_position,
        .roll = center_roll + i * std::numbers::pi / 5,
    };
  }
  return blades;
}
