// Copyright (c) 2026 Fsk. All Rights Reserved.
#pragma once
#include "Armor.hpp"
#include <cstdint>
#include <iceoryx_hoofs/cxx/vector.hpp>

namespace msgs {

template <std::uint64_t SIZE>
struct [[deprecated("Use continuous published armor instead.")]]
Armors {
  iox::cxx::vector<Armor, SIZE> armors;
};

using Armors_2 = Armors<3>;
using Armors_3 = Armors<3>;
using Armors_4 = Armors<4>;

} // namespace msgs
