// Copyright (c) 2026 Feng. All Rights Reserved.
#pragma once
#include "Bullet.hpp"
#include "iceoryx_hoofs/cxx/vector.hpp"
#include <cstdint>

namespace msgs {

template <std::uint64_t SIZE>
struct [[deprecated("Use continuous published bullet instead.")]] Bullets {
  iox::cxx::vector<Bullet, SIZE> bullets;
};

using Bullets_3 = Bullets<3>;
using Bullets_5 = Bullets<5>;
using Bullets_10 = Bullets<10>;
} // namespace msgs
