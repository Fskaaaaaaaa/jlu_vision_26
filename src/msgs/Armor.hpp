// Copyright (c) 2026 Fsk. All Rights Reserved.
#pragma once
#include <iceoryx_hoofs/cxx/string.hpp>
#include <iceoryx_hoofs/cxx/vector.hpp>

namespace msgs {
struct Armor {
  iox::cxx::string<10> armor_number;
  double distance_to_image_center;
  iox::cxx::vector<double, 3> rvec;
  iox::cxx::vector<double, 3> tvec;
};
} // namespace msgs
