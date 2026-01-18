#pragma once

namespace types {
enum class ArmorType {
  One,
  Two,
  Three,
  Four,
  Sentry,
  Outpost,
  Base,
  Negative,
};
struct Armor {
  ArmorType type;
};
} // namespace types
