#pragma once

#include "configs.hpp"
#include "types.hpp"
#include <utility>
#include <vector>
namespace auto_buff {

class BuffTarget {
public:
  BuffTarget(const BuffBladeMatchConfig &match_conf);
  std::vector<std::pair<BuffBladePositionRollPoints, BuffBladeIndex>>
  matchBlades() const;
  static BuffBladeIndex matchBlade(const BuffState &state,
                                   const BuffBladePositionRollPoints &blade);

private:
  BuffBladeMatchConfig match_config_;
};

class SmallBuffTarget : public BuffTarget {
public:
  BuffState predictBuffState(double dt) const;

private:
  // 用来注入到BuffState里，最好设成static
  static SmallBuffState predictBuffState(const BuffState &state, double vroll);

  SmallBuffState target_state_;
};

class BigBuffTarget : private BuffTarget {
public:
private:
};

} // namespace auto_buff
