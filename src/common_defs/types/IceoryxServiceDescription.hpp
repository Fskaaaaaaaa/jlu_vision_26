#pragma once

#include "confs/IceoryxServiceDescription.hpp"
#include "iceoryx_posh/capro/service_description.hpp"

namespace types {

struct IceoryxServiceDescription {
  IceoryxServiceDescription(
      const confs::IceoryxServiceDescription &description);
  iox::capro::ServiceDescription description;
};

} // namespace types
