#pragma once

namespace types {
enum class ArmorType {
  one,
  two,
  three,
  four,
  sentry,
  outpost,
  base,
  negative,
};
struct Armor {
  ArmorType type;
};
} // namespace types
