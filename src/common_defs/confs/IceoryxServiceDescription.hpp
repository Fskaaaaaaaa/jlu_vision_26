#pragma once

#include "iceoryx_posh/capro/service_description.hpp"

#include <string>

namespace confs {
struct IceoryxServiceDescription {
  std::string service;
  std::string instance;
  std::string event;
  iox::capro::ServiceDescription get();
};
} // namespace confs
